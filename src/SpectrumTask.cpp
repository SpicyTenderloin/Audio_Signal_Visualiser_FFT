#include "SpectrumTask.h"

#include <Arduino.h>
#include <new>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"

#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Display.h"
#include "VUMeter.h"
#include "AudioCapture.h"
#include "RealFFT.h"

// The display runs as a two-stage pipeline, so the next frame is being computed
// while the previous one is still being drawn:
//
//   compute_task (core 0)  takes the newest window of samples out of the
//                          capture ring buffer, windows it, runs the FFT and
//                          builds the power spectrum (or, in waveform mode,
//                          copies the raw samples), into one of two frame
//                          buffers.
//   draw_task    (core 1)  draws a finished frame, and owns everything else
//                          that touches the display: axes, HUD, the
//                          calibration screen.
//
// The frame time becomes the longer of the two stages instead of their sum.
// Two buffers circulate between the tasks through a pair of queues (free and
// ready), so compute never writes a buffer that is being drawn, and if drawing
// falls behind, compute simply waits for a free buffer.
//
// There is no frame-rate target. Compute takes the freshest window (the N
// samples ending at the newest one) whenever at least one new sample has
// arrived, so consecutive windows overlap by however much the frame time
// allows. The measured rate (frames drawn per second) is shown on the HUD.
//
// Core 1 also runs the Arduino loop task, which polls the buttons and serial
// port. That task gets a higher priority than drawing (see
// start_spectrum_task()), so button and serial handling preempt a draw within
// microseconds and are never held up by it.

// One finished frame, handed from the compute task to the draw task.
struct Frame
{
  float *prefix;        // FFT frames: running sums of per-bin power (FFT_MAX/2 floats)
  int16_t wave[PLOT_W]; // waveform frames: one centered sample per pixel column
  uint16_t N;           // FFT length this frame was computed with
  float refPow;         // full-scale power, the 0dBFS reference for this frame
  bool isFFT;
  uint32_t gen;         // gAxesGen when the frame was computed (see draw_task)
};

static Frame s_frames[2];
static QueueHandle_t s_freeQ = nullptr;  // Frame* buffers ready to be filled
static QueueHandle_t s_readyQ = nullptr; // Frame* buffers filled and waiting to be drawn

static void compute_task(void *pvParameters)
{
  uint32_t lastPos = capture_write_pos();

  // Ask the capture task to wake this one whenever new samples arrive.
  capture_set_notify_task(xTaskGetCurrentTaskHandle());

  for (;;)
  {
    if (gCalibrating || gPaused)
    {
      // Calibration replaces the normal display (the calibration code reads
      // straight out of the capture buffer); pause just holds the picture.
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (capture_write_pos() == lastPos)
    {
      // Nothing new yet: sleep until the capture task announces more samples,
      // or a few ms pass so a stalled capture is still noticed. Sleeping on the
      // notification instead of polling on the 1ms tick means a frame starts
      // the moment its data lands, not up to a tick later.
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
      continue;
    }

    Frame *f = nullptr;
    if (xQueueReceive(s_freeQ, &f, pdMS_TO_TICKS(20)) != pdTRUE)
      continue; // both buffers are with the drawer; look again shortly

    // Everything below is read only after a buffer is in hand, so the window is
    // the freshest available and the settings are current.
    const uint32_t gen = gAxesGen;
    const bool isFFT = (gDisplayMode == MODE_FFT);
    const uint16_t N = isFFT ? N_CHOICES[gNidx] : (uint16_t)PLOT_W;
    const uint32_t pos = capture_write_pos();
    lastPos = pos; // always take the freshest window rather than building a backlog

    const uint32_t frameStartUs = micros();
    const uint32_t start = pos - N;
    int16_t peakAbs = 0;

    f->gen = gen;
    f->isFFT = isFFT;
    f->N = N;

    if (isFFT)
    {
      // Read straight out of the capture ring buffer into fft_buf, windowing
      // as we go - no intermediate raw-sample buffer needed. The N real
      // samples are stored consecutively, which is exactly the packed
      // (x[2m] + j*x[2m+1]) layout realfft_forward() wants. Peak (for the VU
      // meter) is tracked from the same pass, pre-window.
      CaptureSpan spans[2];
      capture_spans(start, N, spans);
      const bool hann = gUseHann;
      uint16_t i = 0;
      for (int part = 0; part < 2; ++part)
      {
        const int16_t *src = spans[part].p;
        const uint16_t count = spans[part].n;
        for (uint16_t j = 0; j < count; ++j, ++i)
        {
          const int16_t s = src[j];
          const int16_t a = s >= 0 ? s : (int16_t)-s;
          if (a > peakAbs)
            peakAbs = a;

          float v = (float)s;
          if (hann)
            v *= window_buf[i];
          fft_buf[i] = v;
        }
      }

      const uint32_t fftStartUs = micros();
      realfft_forward(fft_buf, N);
      gLastFFTus = micros() - fftStartUs;

      // --- Build power spectrum and prefix sums (skip DC) ---
      // Only up to the highest bin draw_line_spectrum() can ever read (bins
      // beyond Fmax are never displayed) - with Fmax well under Nyquist,
      // which is the common case, this is a real fraction of N/2 skipped.
      const int Kvis = visible_bin_count(N);
      // A windowed full-scale sine's FFT peak is attenuated by the window's
      // coherent gain, so scale the reference by it too (gWindowGain == 1.0
      // when gUseHann is off) - otherwise 0dBFS is never reachable.
      const float refAmp = (N / 2.0f) * ADC_FS * (gUseHann ? gWindowGain : 1.0f);
      f->refPow = refAmp * refAmp; // power ref for dBFS

      realfft_power_prefix(fft_buf, N, Kvis, f->prefix);
    }
    else // MODE_WAVEFORM
    {
      for (uint16_t i = 0; i < N; i++)
      {
        const int16_t s = capture_sample_at(start + i);
        const int16_t a = s >= 0 ? s : (int16_t)-s;
        if (a > peakAbs)
          peakAbs = a;
        f->wave[i] = s;
      }
    }

    setVU(vu_from_peakAbs(peakAbs));

    gLastComputeUs = micros() - frameStartUs;

    xQueueSend(s_readyQ, &f, 0); // never full: only two buffers exist
  }
}

static void draw_task(void *pvParameters)
{
  uint32_t fpsWindowStart = 0;
  uint16_t fpsFrameCount = 0;
  uint32_t ratePos = capture_write_pos(); // capture position at the start of the rate window

  for (;;)
  {
    if (gCalibrating)
    {
      // Replaces the normal FFT/waveform display entirely while active -
      // capture_drain_task keeps filling the buffer in the background
      // (calibration_capture_point() reads straight out of it), it just
      // isn't run through the FFT/plot pipeline.
      draw_calibration_screen(gCalScreenDirty);
      gCalScreenDirty = false;
      vTaskDelay(pdMS_TO_TICKS(100)); // ~10Hz is plenty for a live readout
      continue;
    }

    // Wait for a finished frame, but only briefly, so axes, pause and
    // calibration state are still serviced when no frames are arriving.
    Frame *f = nullptr;
    const bool got = (xQueueReceive(s_readyQ, &f, pdMS_TO_TICKS(20)) == pdTRUE);

    if (gAxesDirty)
    {
      draw_axes(N_CHOICES[gNidx]);
      // Any frame computed before this redraw used the old settings; bumping
      // the generation makes the check below discard those.
      gAxesGen = gAxesGen + 1;
    }

    if (!got)
      continue;

    // Discard a frame from before the latest settings/mode change - it no
    // longer matches the axes on screen.
    const bool stale = (f->gen != gAxesGen) ||
                       (f->isFFT != (gDisplayMode == MODE_FFT)) ||
                       (f->isFFT && f->N != N_CHOICES[gNidx]);
    if (stale)
    {
      xQueueSend(s_freeQ, &f, 0);
      continue;
    }

    const uint32_t drawStartUs = micros();
    if (f->isFFT)
      draw_line_spectrum(f->N, f->prefix, f->refPow); // also redraws the HUD if gHUDDirty
    else
      draw_waveform(f->wave); // also redraws the HUD if gHUDDirty
    gLastDrawUs = micros() - drawStartUs;

    xQueueSend(s_freeQ, &f, 0); // hand the buffer back to compute

    // Measured frame rate (frames drawn) over a rolling 1s window. Redraws just
    // the small FPS box, not the whole HUD, so the rest of the status band
    // doesn't flicker once/sec.
    fpsFrameCount++;
    const uint32_t nowUs = micros();
    if (fpsWindowStart == 0)
      fpsWindowStart = nowUs;
    const uint32_t fpsElapsed = nowUs - fpsWindowStart;
    if (fpsElapsed >= 1000000UL)
    {
      gMeasuredFPS = (float)fpsFrameCount * 1e6f / (float)fpsElapsed;
      const uint32_t posNow = capture_write_pos();
      gMeasuredCaptureHz = (float)(posNow - ratePos) * 1e6f / (float)fpsElapsed;
      ratePos = posNow;
      fpsFrameCount = 0;
      fpsWindowStart = nowUs;
      draw_hud_fps();
    }
  }
}

void start_spectrum_task()
{
  // The two frame buffers' power-spectrum storage (see alloc_fft_buffers() for
  // why these are heap-allocated).
  for (int i = 0; i < 2; ++i)
  {
    s_frames[i].prefix = new (std::nothrow) float[FFT_MAX / 2];
    check_alloc(s_frames[i].prefix, "frame prefix buffer", (FFT_MAX / 2) * sizeof(float));
  }

  s_freeQ = xQueueCreate(2, sizeof(Frame *));
  s_readyQ = xQueueCreate(2, sizeof(Frame *));
  for (int i = 0; i < 2; ++i)
  {
    Frame *f = &s_frames[i];
    xQueueSend(s_freeQ, &f, 0);
  }

  // Both cores can now be busy flat out, so neither idle task is guaranteed to
  // run: take them off the task watchdog. Nothing here needs the idle tasks (no
  // tasks are deleted, no power management). This has to go through
  // esp_task_wdt_reconfigure(): deleting an idle task from the watchdog by hand
  // (what Arduino's disableCore0WDT() does) leaves its idle hook running, and it
  // then logs "task not found" on every idle pass.
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000; // keep the framework's timeout
  wdt.idle_core_mask = 0;                                // watch no idle tasks
  wdt.trigger_panic = true;
  esp_task_wdt_reconfigure(&wdt);

  xTaskCreatePinnedToCore(compute_task, "compute", 6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(draw_task, "draw", 8192, nullptr, 1, nullptr, 1);

  // This function runs inside the Arduino loop task, which shares core 1 with
  // the draw task. Raise the loop task above it so button and serial polling
  // always preempts a draw instead of waiting behind it. It spends nearly all
  // its time asleep, so drawing loses almost nothing.
  vTaskPrioritySet(nullptr, 2);
}
