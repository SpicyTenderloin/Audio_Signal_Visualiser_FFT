#include "Globals.h"

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

volatile uint32_t gFs = 12000;
volatile uint16_t gFPS = 120;

const uint16_t N_CHOICES[8] = {32, 64, 128, 256, 512, 1024, 2048, 4096};
volatile uint8_t gNidx = 4; // default N=512 (index 4)
volatile uint8_t gAgg = 1;
volatile float gFmaxHz = 2500.0f;
volatile bool gFmaxFollowNyq = false;

volatile float gYMax_dB = -20.0f; // default top
volatile float gYMin_dB = -50.0f; // default bottom

volatile XScaleMode gXScale = XS_LIN;
volatile YScaleMode gYScale = YS_DB;

bool gUseHann = true;
bool gPaused = false;

float fft_buf[2 * FFT_MAX];
float window_buf[FFT_MAX];

volatile uint16_t gDC = 2048;

bool gAxesDirty = true;
bool gHUDDirty = true;

float gPow[FFT_MAX / 2];
float gPrefixPow[FFT_MAX / 2];
float gRefPow = 1.0f;
