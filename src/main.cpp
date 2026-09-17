#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "esp_dsp.h"
#include "dsps_fft2r.h"
#include <math.h>

#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Display.h"
#include "VUMeter.h"
#include "AudioCapture.h"
#include "Settings.h"
#include "Controls.h"
#include "SerialConsole.h"

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("FFT Spectrum (Line) - HW Timer ADC, Fs default 15 kHz (serial-adjustable), Fmax=horizontal zoom"));
  print_controls();
  print_stats();
  print_prompt();

  init_controls();

  // VU pins
  pinMode(VU1, OUTPUT);
  pinMode(VU2, OUTPUT);
  pinMode(VU3, OUTPUT);
  pinMode(VU4, OUTPUT);
  pinMode(VU5, OUTPUT);
  pinMode(VU6, OUTPUT);
  setVU(0);

  // TFT SPI init — use 40 MHz for stability on all panels
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setSPISpeed(40000000);
  tft.setRotation(1);

  // Quick splash so we know the panel is alive
  tft.fillScreen(ILI9341_RED);
  delay(150);
  tft.fillScreen(COL_BG);

  // esp-dsp init (real FFT tables, sized for the largest N we support)
  dsps_fft2r_init_fc32(nullptr, FFT_MAX);

  // Window for the default FFT length
  make_window_for_N(N_CHOICES[gNidx]);

  // ADC + hardware timer capture (mic sampling)
  init_audio_capture();

  // Start with Fmax acting as horizontal zoom, clamped to Nyquist
  gFmaxFollowNyq = false;
  gFmaxHz = clampf(5000.0f, 50.0f, 0.5f * (float)gFs);

  draw_axes(N_CHOICES[gNidx]);
}

void loop()
{
  service_serial();
  pollButtons();

  if (gAxesDirty)
    draw_axes(N_CHOICES[gNidx]);
  if (gPaused)
  {
    delay(1000 / gFPS);
    return;
  }

  if (!bufReady)
  {
    delay(1);
    return;
  }

  // Grab ready buffer
  int16_t *useBuf;
  uint16_t N = N_CHOICES[gNidx];
  int16_t peakAbsCopy;
  portENTER_CRITICAL(&timerMux);
  useBuf = (int16_t *)readyBuf;
  readyBuf = nullptr;
  bufReady = false;
  peakAbsCopy = isrPeakAbs;
  portEXIT_CRITICAL(&timerMux);

  // FFT input: apply the window (if enabled) and build the complex signal
  for (uint16_t i = 0; i < N; i++)
  {
    float v = (float)useBuf[i];
    if (gUseHann)
      v *= window_buf[i];
    fft_buf[2 * i] = v;
    fft_buf[2 * i + 1] = 0.0f;
  }

  // FFT
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

  // HUD (Fs, N, Df, Hann)
  if (gHUDDirty)
  {
    draw_hud(N, df_eff);
  }

  // VU
  setVU(vu_from_peakAbs(peakAbsCopy));

  // FPS cap
  static uint32_t lastFrame = 0;
  uint32_t now = micros();
  uint32_t minDelta = 1000000UL / gFPS;
  if (lastFrame && now - lastFrame < minDelta)
  {
    delayMicroseconds(minDelta - (now - lastFrame));
  }
  lastFrame = micros();
}
