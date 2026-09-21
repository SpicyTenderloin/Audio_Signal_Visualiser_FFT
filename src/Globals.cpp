#include "Globals.h"
#include <new>

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// gFs/gNidx/gFmaxHz's real starting values are set in setup() via
// recommend_fs_n() (best fidelity for the default Fmax) before anything
// reads them - these are just fallbacks in case that were ever skipped.
volatile uint32_t gFs = 40000;
uint32_t gInactiveFs = 40000; // the waveform mode's, until first switched to

const uint16_t N_CHOICES[9] = {32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};
volatile uint8_t gNidx = 5; // N=1024
volatile uint8_t gAgg = 1;
volatile float gFmaxHz = 2500.0f;
volatile bool gFmaxFollowNyq = false;

volatile float gYMax_dB = 0.0f; // default top
volatile float gYMin_dB = -40.0f; // default bottom

volatile XScaleMode gXScale = XS_LIN;
volatile YScaleMode gYScale = YS_DB;

bool gUseHann = true;
bool gPaused = false;
volatile float gWindowGain = 1.0f; // recomputed by make_window_for_N()

volatile DisplayMode gDisplayMode = MODE_FFT;
volatile float gWaveYRange = 2048.0f; // full ADC half-range by default

float *fft_buf = nullptr;
float *window_buf = nullptr;

volatile uint16_t gDC = 2048;

volatile CalSource gCalSource = CAL_NONE;
volatile float gCalGain = ADC_VREF / ADC_FS;
volatile float gCalOffset = 0.0f;

volatile bool gCalibrating = false;
bool gCalScreenDirty = true;
CalPoint gCalPoints[CAL_MAX_POINTS];
volatile uint8_t gCalPointCount = 0;
volatile float gCalTargetV = 0.0f;

bool gAxesDirty = true;
bool gHUDDirty = true;

float *gPrefixPow = nullptr;

volatile float gMeasuredFPS = 0.0f;
volatile uint32_t gLastFFTus = 0;
volatile uint32_t gLastDrawUs = 0;
volatile uint32_t gLastFrameUs = 0;
float gRefPow = 1.0f;

uint32_t fft_mode_fs()
{
  return gDisplayMode == MODE_FFT ? gFs : gInactiveFs;
}

void alloc_fft_buffers()
{
  fft_buf = new (std::nothrow) float[FFT_MAX];
  check_alloc(fft_buf, "fft_buf", FFT_MAX * sizeof(float));

  window_buf = new (std::nothrow) float[FFT_MAX];
  check_alloc(window_buf, "window_buf", FFT_MAX * sizeof(float));

  gPrefixPow = new (std::nothrow) float[FFT_MAX / 2];
  check_alloc(gPrefixPow, "gPrefixPow", (FFT_MAX / 2) * sizeof(float));
}

void check_alloc(const void *p, const char *what, size_t bytes)
{
  if (p)
    return;
  Serial.printf("FATAL: failed to allocate %u bytes for %s (free heap: %u bytes). Halting.\r\n",
                (unsigned)bytes, what, (unsigned)ESP.getFreeHeap());
  while (true)
    delay(1000);
}
