#include "SpectrumTask.h"

#include <Arduino.h>

#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Display.h"
#include "VUMeter.h"
#include "AudioCapture.h"
#include "RealFFT.h"

// Runs forever on core 0: pulls the newest window of samples out of the
// capture ring buffer and redraws the plot - either the FFT spectrum or the
// raw waveform, depending on gDisplayMode.
//
// There is no frame-rate target. The loop runs flat out: each pass takes the
// freshest window (the N samples ending at the newest one) as soon as at
// least one new sample has arrived since the previous pass, so consecutive
// windows overlap by however much the frame time allows. A bigger FFT just
// makes each frame cost more; it doesn't change how the loop is paced. The
// measured rate is shown on the HUD.
//
// Because the loop never sleeps while there is data to show, core 0's idle
// task never gets to run and would trip the idle-task watchdog. Nothing here
// needs the idle task (no tasks are deleted, no power management), so it is
// taken off the watchdog instead of being fed by an artificial sleep that
// would cap the frame rate.
static void spectrum_task(void *pvParameters)
{
  static int16_t waveSamples[PLOT_W];
  uint32_t lastPos = capture_write_pos();
  uint32_t fpsWindowStart = 0;
  uint16_t fpsFrameCount = 0;

  disableCore0WDT();

  for (;;)
  {
    if (gCalibrating)
    {
      // Replaces the normal FFT/waveform display entirely while active -
      // capture_drain_task keeps filling the buffer in the background
      // (calibration_capture_point() reads straight out of it), it just
      // isn't run through the FFT/plot pipeline below.
      draw_calibration_screen(gCalScreenDirty);
      gCalScreenDirty = false;
      vTaskDelay(pdMS_TO_TICKS(100)); // ~10Hz is plenty for a live readout
      continue;
    }

    if (gAxesDirty)
      draw_axes(N_CHOICES[gNidx]);

    if (gPaused)
    {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    const bool isFFT = (gDisplayMode == MODE_FFT);
    const uint16_t N = isFFT ? N_CHOICES[gNidx] : (uint16_t)PLOT_W;

    uint32_t pos = capture_write_pos();
    if (pos == lastPos)
    {
      vTaskDelay(1); // nothing new to show yet; let capture run
      continue;
    }
    lastPos = pos; // always take the freshest window rather than building a backlog

    uint32_t frameStartUs = micros();
    uint32_t start = pos - N;
    int16_t peakAbs = 0;

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

      uint32_t fftStartUs = micros();
      realfft_forward(fft_buf, N);
      gLastFFTus = micros() - fftStartUs;

      // --- Build power spectrum and prefix sums (skip DC) ---
      // Only up to the highest bin draw_line_spectrum() can ever read (bins
      // beyond Fmax are never displayed) - with Fmax well under Nyquist,
      // which is the common case, this is a real fraction of N/2 skipped.
      int Kvis = visible_bin_count(N);
      // A windowed full-scale sine's FFT peak is attenuated by the window's
      // coherent gain, so scale the reference by it too (gWindowGain == 1.0
      // when gUseHann is off) - otherwise 0dBFS is never reachable.
      float refAmp = (N / 2.0f) * ADC_FS * (gUseHann ? gWindowGain : 1.0f);
      gRefPow = refAmp * refAmp; // power ref for dBFS

      realfft_power_prefix(fft_buf, N, Kvis, gPrefixPow);

      uint32_t drawStartUs = micros();
      draw_line_spectrum(N); // also redraws the HUD if gHUDDirty
      gLastDrawUs = micros() - drawStartUs;
    }
    else // MODE_WAVEFORM
    {
      for (uint16_t i = 0; i < N; i++)
      {
        int16_t s = capture_sample_at(start + i);
        int16_t a = s >= 0 ? s : (int16_t)-s;
        if (a > peakAbs)
          peakAbs = a;
        waveSamples[i] = s;
      }

      uint32_t drawStartUs = micros();
      draw_waveform(waveSamples); // also redraws the HUD if gHUDDirty
      gLastDrawUs = micros() - drawStartUs;
    }

    setVU(vu_from_peakAbs(peakAbs));

    gLastFrameUs = micros() - frameStartUs;

    // Measured frame rate over a rolling 1s window. Redraws just the small
    // FPS box, not the whole HUD, so the rest of the status band doesn't
    // flicker once/sec.
    fpsFrameCount++;
    uint32_t nowUs = micros();
    if (fpsWindowStart == 0)
      fpsWindowStart = nowUs;
    uint32_t fpsElapsed = nowUs - fpsWindowStart;
    if (fpsElapsed >= 1000000UL)
    {
      gMeasuredFPS = (float)fpsFrameCount * 1e6f / (float)fpsElapsed;
      fpsFrameCount = 0;
      fpsWindowStart = nowUs;
      draw_hud_fps();
    }
  }
}

void start_spectrum_task()
{
  xTaskCreatePinnedToCore(spectrum_task, "spectrum", 8192, nullptr, 1, nullptr, 0);
}
