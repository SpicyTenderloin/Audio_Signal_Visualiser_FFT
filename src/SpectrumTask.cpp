#include "SpectrumTask.h"

#include <Arduino.h>
#include "esp_dsp.h"
#include "dsps_fft2r.h"

#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Display.h"
#include "VUMeter.h"
#include "AudioCapture.h"

// Runs forever on core 0: pulls the newest (possibly overlapping) window of
// samples out of the capture ring buffer and redraws the plot - either the
// FFT spectrum or the raw waveform, depending on gDisplayMode.
//
// The hop (new samples required before the next window) is sized from the
// *target frame rate*, not from the window size: hop = Fs/gFPS, clamped to
// the window. That keeps the wait for fresh data roughly constant (~1/gFPS)
// no matter how large the window gets - a large FFT N automatically gets
// heavier overlap to compensate, instead of the frame rate collapsing as N
// grows. The waveform window is always exactly PLOT_W samples (one per
// pixel column), so the same clamp just bounds hop by the plot width there.
static void spectrum_task(void *pvParameters)
{
  static int16_t waveSamples[PLOT_W];
  uint32_t lastPos = capture_write_pos();
  uint32_t fpsWindowStart = 0;
  uint16_t fpsFrameCount = 0;

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
      vTaskDelay(pdMS_TO_TICKS(1000 / gFPS));
      continue;
    }

    const bool isFFT = (gDisplayMode == MODE_FFT);
    const uint16_t N = isFFT ? N_CHOICES[gNidx] : (uint16_t)PLOT_W;
    int hop = clampi((int)(gFs / gFPS), 1, N);

    uint32_t pos = capture_write_pos();
    if ((uint32_t)(pos - lastPos) < (uint32_t)hop)
    {
      vTaskDelay(1); // not enough new samples yet; let capture fill more
      continue;
    }
    lastPos = pos; // always take the freshest window rather than building a backlog

    uint32_t frameStartUs = micros();
    uint32_t start = pos - N;
    int16_t peakAbs = 0;

    if (isFFT)
    {
      // Read straight out of the capture ring buffer into fft_buf, windowing
      // as we go - no intermediate raw-sample buffer needed. Peak (for the
      // VU meter) is tracked from the same pass, pre-window.
      for (uint16_t i = 0; i < N; i++)
      {
        int16_t s = capture_sample_at(start + i);
        int16_t a = s >= 0 ? s : (int16_t)-s;
        if (a > peakAbs)
          peakAbs = a;

        float v = (float)s;
        if (gUseHann)
          v *= window_buf[i];
        fft_buf[2 * i] = v;
        fft_buf[2 * i + 1] = 0.0f;
      }

      uint32_t fftStartUs = micros();
      dsps_fft2r_fc32(fft_buf, N);
      dsps_bit_rev_fc32(fft_buf, N);
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

      gPrefixPow[0] = 0.0f; // so k0-1 works when k0==1
      for (int k = 1; k < Kvis; ++k)
      {
        float re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
        float p = re * re + im * im; // power (no sqrt)
        gPrefixPow[k] = gPrefixPow[k - 1] + p;
      }

      draw_line_spectrum(N); // also redraws the HUD if gHUDDirty
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

      draw_waveform(waveSamples); // also redraws the HUD if gHUDDirty
    }

    setVU(vu_from_peakAbs(peakAbs));

    gLastFrameUs = micros() - frameStartUs;

    // Measured (actual) frame rate over a rolling 1s window - distinct from
    // gFPS, the target cap. Redraws just the small FPS box, not the whole
    // HUD, so the rest of the status band doesn't flicker once/sec.
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

    // FPS cap. A real (yielding) delay, not delayMicroseconds()/busy-wait: with
    // the hop now sized to arrive right on the FPS cadence, this task is ready
    // to run almost every iteration, so a non-yielding wait here would pin
    // core 0 and starve its idle task, tripping the idle-task watchdog reset.
    vTaskDelay(pdMS_TO_TICKS(1000 / gFPS));
  }
}

void start_spectrum_task()
{
  xTaskCreatePinnedToCore(spectrum_task, "spectrum", 8192, nullptr, 1, nullptr, 0);
}
