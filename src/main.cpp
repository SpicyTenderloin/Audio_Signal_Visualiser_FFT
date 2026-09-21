#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

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
#include "Calibration.h"
#include "RealFFT.h"

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("FFT Spectrum (Line) - continuous ADC/DMA, Fmax=horizontal zoom, ADC oversampled and averaged down to Fs"));

  // Start with Fmax acting as horizontal zoom, at the default Fs and N
  // (DEFAULT_FS_HZ / DEFAULT_N, Config.h). Set before anything below reads
  // gFs/gNidx, so it takes effect from boot.
  gFmaxFollowNyq = false;
  gFmaxHz = DEFAULT_FMAX_HZ;
  gFs = DEFAULT_FS_HZ;
  // The waveform mode starts from the same rate; from here on the two modes'
  // Fs are independent (see toggleDisplayMode()).
  gInactiveFs = gFs;
  for (uint8_t i = 0; i < sizeof(N_CHOICES) / sizeof(N_CHOICES[0]); i++)
    if (N_CHOICES[i] == DEFAULT_N)
    {
      gNidx = i;
      break;
    }

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

  // FFT tables: esp-dsp's twiddles for the half-length complex FFT the real
  // FFT runs on, plus the post-processing twiddles (see RealFFT.h). esp-dsp's
  // table generator has a hard cap (CONFIG_DSP_MAX_FFT_SIZE, 4096 in this
  // build) on that half length - if FFT_MAX/2 ever exceeds it, or memory runs
  // out, init fails and every FFT after would run on uninitialized tables, so
  // halt with a clear message rather than continuing into an eventual crash.
  if (!realfft_init())
  {
    Serial.printf("FATAL: FFT table init failed for FFT_MAX=%d (out of memory, or FFT_MAX/2 "
                  "exceeds esp-dsp's CONFIG_DSP_MAX_FFT_SIZE). Halting.\r\n",
                  FFT_MAX);
    while (true)
      delay(1000);
  }

  // Window for the default FFT length
  make_window_for_N(N_CHOICES[gNidx]);

  // ADC continuous/DMA capture (mic sampling into the circular buffer)
  init_audio_capture();

  // Resolves which ADC calibration tier is active for this boot (saved
  // user calibration > chip's eFuse curve > naive ADC_VREF/ADC_FS ratio) -
  // after init_audio_capture() since it needs ADC1's channel/attenuation
  // already configured.
  calibration_init();

  // The FFT runs on core 0 and drawing on core 1, as a pipeline (see
  // SpectrumTask.cpp). The loop task that polls buttons and serial shares core
  // 1 with drawing, so start_spectrum_task() also raises its priority: it always
  // preempts a draw. The draw task performs the first draw_axes() itself, since
  // gAxesDirty starts true.
  start_spectrum_task();
}

void loop()
{
  service_serial();
  pollButtons();
  delay(1);
}
