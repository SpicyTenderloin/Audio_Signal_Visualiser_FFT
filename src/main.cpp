#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "esp_dsp.h"
#include "dsps_fft2r.h"

#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Display.h"
#include "VUMeter.h"
#include "AudioCapture.h"
#include "Settings.h"
#include "Controls.h"
#include "SerialConsole.h"
#include "SpectrumTask.h"

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("FFT Spectrum (Line) - I2S/DMA ADC, Fs default 40 kHz (serial-adjustable), Fmax=horizontal zoom"));
  print_controls();
  print_stats();
  print_prompt();

  // Heap-allocate the FFT/window/capture buffers before anything touches
  // them - they're sized off FFT_MAX, which is too large for the linker's
  // static DRAM segment alone to fit as fixed arrays.
  alloc_fft_buffers();

  init_controls();

  // VU pins
  pinMode(VU1, OUTPUT);
  pinMode(VU2, OUTPUT);
  pinMode(VU3, OUTPUT);
  pinMode(VU4, OUTPUT);
  pinMode(VU5, OUTPUT);
  pinMode(VU6, OUTPUT);
  setVU(0);

  // TFT SPI init. 80 MHz is the ESP32 SPI peripheral's practical ceiling; drop
  // back to 40 MHz (or lower) if your wiring (long jumpers/breadboard) shows
  // display glitches/tearing at this speed.
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setSPISpeed(80000000);
  tft.setRotation(1);

  // Quick splash so we know the panel is alive
  tft.fillScreen(ILI9341_RED);
  delay(150);
  tft.fillScreen(COL_BG);

  // esp-dsp init (real FFT tables, sized for the largest N we support)
  dsps_fft2r_init_fc32(nullptr, FFT_MAX);

  // Window for the default FFT length
  make_window_for_N(N_CHOICES[gNidx]);

  // ADC + I2S/DMA capture (mic sampling into the circular buffer)
  init_audio_capture();

  // Start with Fmax acting as horizontal zoom, clamped to Nyquist
  gFmaxFollowNyq = false;
  gFmaxHz = clampf(2500.0f, 50.0f, 0.5f * (float)gFs);

  // FFT/draw runs on core 0 so button/serial polling on core 1 (loop())
  // never waits on FFT compute or SPI draw time. It performs the first
  // draw_axes() itself, since gAxesDirty starts true.
  start_spectrum_task();
}

void loop()
{
  service_serial();
  pollButtons();
  delay(1);
}
