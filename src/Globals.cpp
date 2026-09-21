#include "Globals.h"
#include <new>

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// gFs/gNidx/gFmaxHz's real starting values are set in setup() (from
// DEFAULT_FS_HZ, DEFAULT_N and DEFAULT_FMAX_HZ) before anything reads them -
// these are just fallbacks in case that were ever skipped.
volatile uint32_t gFs = DEFAULT_FS_HZ;
uint32_t gInactiveFs = DEFAULT_FS_HZ; // the waveform mode's, until first switched to
volatile uint32_t gAvgReq = 0;         // 0 = automatic
uint32_t gInactiveAvgReq = 0;

const uint16_t N_CHOICES[9] = {32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};
volatile uint8_t gNidx = 7; // N=4096 (DEFAULT_N)
volatile uint8_t gAgg = 1;
volatile float gFmaxHz = DEFAULT_FMAX_HZ;
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

volatile uint32_t gAxesGen = 0;

volatile float gMeasuredFPS = 0.0f;
volatile float gMeasuredCaptureHz = 0.0f;
volatile uint32_t gAdcHwHz = 0;
volatile uint32_t gAdcAvg = 1;
volatile uint32_t gLastFFTus = 0;
volatile uint32_t gLastComputeUs = 0;
volatile uint32_t gLastDrawUs = 0;

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
