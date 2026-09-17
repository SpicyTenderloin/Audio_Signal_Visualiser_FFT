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
// Using overlapping windows (hop = N/gOverlap, instead of a fresh block of N
// samples every frame) decouples the frame rate from the FFT length, so a
// large N no longer means a sluggish, jumpy plot.
static void spectrum_task(void *pvParameters)
{
  static int16_t frame[FFT_MAX];
  uint32_t lastPos = capture_write_pos();
  uint32_t lastFrameMicros = 0;

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
    int hop = N / gOverlap;
    if (hop < 1)
      hop = 1;

    uint32_t pos = capture_write_pos();
    if ((uint32_t)(pos - lastPos) < (uint32_t)hop)
    {
      vTaskDelay(1); // not enough new samples yet; let the ISR fill more
      continue;
    }
    lastPos = pos; // always take the freshest window rather than building a backlog

    capture_read_window(frame, N, pos);

    // Peak (for the VU meter) over this window
    int16_t peakAbs = 0;
    for (uint16_t i = 0; i < N; i++)
    {
      int16_t a = frame[i] >= 0 ? frame[i] : (int16_t)-frame[i];
      if (a > peakAbs)
        peakAbs = a;
    }

    // FFT input: apply the window (if enabled) and build the complex signal
    for (uint16_t i = 0; i < N; i++)
    {
      float v = (float)frame[i];
      if (gUseHann)
        v *= window_buf[i];
      fft_buf[2 * i] = v;
      fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_buf, N);
    dsps_bit_rev_fc32(fft_buf, N);

    // --- Build power spectrum and prefix sums (skip DC) ---
    int Kny = N / 2;
    float refAmp = (N / 2.0f) * ADC_FS; // amplitude ref
    gRefPow = refAmp * refAmp;          // power ref for dBFS

    gPrefixPow[0] = 0.0f; // so k0-1 works when k0==1
    for (int k = 1; k < Kny; ++k)
    {
      float re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
      float p = re * re + im * im; // power (no sqrt)
      gPow[k] = p;
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

    // FPS cap
    uint32_t now = micros();
    uint32_t minDelta = 1000000UL / gFPS;
    if (lastFrameMicros && now - lastFrameMicros < minDelta)
      delayMicroseconds(minDelta - (now - lastFrameMicros));
    lastFrameMicros = micros();
  }
}

void start_spectrum_task()
{
  xTaskCreatePinnedToCore(spectrum_task, "spectrum", 8192, nullptr, 1, nullptr, 0);
}
