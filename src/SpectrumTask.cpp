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
// samples out of the capture ring buffer, FFTs it, and redraws the plot.
//
// The hop (new samples required before the next window) is sized from the
// *target frame rate*, not from N: hop = Fs/gFPS, clamped to N. That keeps
// the wait for fresh data roughly constant (~1/gFPS) no matter how large N
// gets - a large N automatically gets heavier overlap to compensate, instead
// of the frame rate collapsing as N grows.
static void spectrum_task(void *pvParameters)
{
  uint32_t lastPos = capture_write_pos();
  uint32_t fpsWindowStart = 0;
  uint16_t fpsFrameCount = 0;

  for (;;)
  {
    if (gAxesDirty)
      draw_axes(N_CHOICES[gNidx]);

    if (gPaused)
    {
      vTaskDelay(pdMS_TO_TICKS(1000 / gFPS));
      continue;
    }

    const uint16_t N = N_CHOICES[gNidx];
    int hop = clampi((int)(gFs / gFPS), 1, N);

    uint32_t pos = capture_write_pos();
    if ((uint32_t)(pos - lastPos) < (uint32_t)hop)
    {
      vTaskDelay(1); // not enough new samples yet; let the ISR fill more
      continue;
    }
    lastPos = pos; // always take the freshest window rather than building a backlog

    uint32_t frameStartUs = micros();

    // Read straight out of the capture ring buffer into fft_buf, windowing
    // as we go - no intermediate raw-sample buffer needed. Peak (for the VU
    // meter) is tracked from the same pass, pre-window.
    uint32_t start = pos - N;
    int16_t peakAbs = 0;
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
    int Kny = N / 2;
    // A windowed full-scale sine's FFT peak is attenuated by the window's
    // coherent gain, so scale the reference by it too (gWindowGain == 1.0
    // when gUseHann is off) - otherwise 0dBFS is never reachable.
    float refAmp = (N / 2.0f) * ADC_FS * (gUseHann ? gWindowGain : 1.0f);
    gRefPow = refAmp * refAmp; // power ref for dBFS

    gPrefixPow[0] = 0.0f; // so k0-1 works when k0==1
    for (int k = 1; k < Kny; ++k)
    {
      float re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
      float p = re * re + im * im; // power (no sqrt)
      gPrefixPow[k] = gPrefixPow[k - 1] + p;
    }

    // Draw (batched)
    const float df = (float)gFs / (float)N;
    const int bpp = bins_per_point(N);
    const float df_eff = df * (float)bpp;
    draw_line_spectrum(N);

    if (gHUDDirty)
      draw_hud(N, df_eff);

    setVU(vu_from_peakAbs(peakAbs));

    gLastFrameUs = micros() - frameStartUs;

    // Measured (actual) frame rate over a rolling 1s window - distinct from
    // gFPS, the target cap. Nudges the HUD dirty so the on-screen number
    // stays roughly live without redrawing it every single frame.
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
      gHUDDirty = true;
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
