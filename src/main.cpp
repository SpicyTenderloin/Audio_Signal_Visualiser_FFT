#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "esp_dsp.h"
#include "dsps_fft2r.h"
#include <math.h>

// ======== ESP32 ADC & Timer ========
#include "driver/adc.h"
#include "esp_adc_cal.h"

// -------------------- TFT Pins --------------------
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19

// -------------------- MIC (ADC) -------------------
#define MIC_CH ADC1_CHANNEL_0 // GPIO36

// -------------------- Buttons --------------------
#define BTN_AGG_DOWN 12 // aggregation down
#define BTN_AGG_UP 13   // aggregation up
#define BTN_N_DOWN 15   // N-
#define BTN_N_UP 2      // N+
#define BTN_PAUSE 0     // toggle pause
#define BTN_ZOOM_UP 4   // horizontal zoom in (+Fmax)
#define BTN_ZOOM_DN 16  // horizontal zoom out (-Fmax)

// -------------------- VU LEDs ---------------------
#define VU1 14
#define VU2 27
#define VU3 26
#define VU4 25
#define VU5 33
#define VU6 32

// -------------------- Display layout --------------
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

static const int LM = 30;
static const int RM = 16;
static const int TOP = 36;     // room for centered title
static const int HUD_H = 20;   // bottom HUD band
static const int BOT_GAP = 18; // gap for X labels above HUD

static const int PLOT_X = LM;
static const int PLOT_Y = TOP;
static const int PLOT_W = SCREEN_W - LM - RM;
static const int PLOT_H = SCREEN_H - TOP - HUD_H - BOT_GAP;
static const int BASE_Y = PLOT_Y + PLOT_H - 1;

// -------------------- Colors ----------------------
#define COL_BG ILI9341_BLACK
#define COL_AX ILI9341_WHITE
#define COL_TEXT ILI9341_WHITE
#define COL_GRID 0x2104
#define COL_GRID_MINOR 0x1082
#define COL_TITLE ILI9341_YELLOW
#define COL_LINE ILI9341_CYAN

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// -------------------- Settings --------------------
volatile uint32_t gFs = 12000; // Hz, default sample rate (serial-adjustable)
volatile uint16_t gFPS = 60;

static const uint16_t N_CHOICES[] = {32, 64, 128, 256, 512, 1024, 2048};
volatile uint8_t gNidx = 4;           // default N=512 (index 4)
volatile uint8_t gAgg = 1;            // aggregation (>=1) => bpp defaults to 1
volatile float gFmaxHz = 3500.0f;     // default horizontal max frequency
volatile bool gFmaxFollowNyq = false; // Fmax acts as zoom (independent of Nyquist)

// Y-axis range (in dBFS). Top is ymax, bottom is ymin.
volatile float gYMax_dB = -20.0f; // default top
volatile float gYMin_dB = -60.0f; // default bottom

enum XScaleMode
{
  XS_LIN = 0,
  XS_LOG = 1
};
volatile XScaleMode gXScale = XS_LIN;

bool gUseHann = true;
bool gPaused = false;

// -------------------- Buffers ---------------------
#define FFT_MAX 2048
static float fft_buf[2 * FFT_MAX];
static float window_buf[FFT_MAX];

// Ping-pong raw sample buffers (int16 centered)
static int16_t bufA[FFT_MAX];
static int16_t bufB[FFT_MAX];

// -------------------- ADC/DC ----------------------
volatile uint16_t gDC = 2048;

// -------------------- Dirty flags -----------------
bool gAxesDirty = true;
bool gHUDDirty = true;

// -------------------- Magnitude scaling -----------
const float ADC_FS = 4095.0f; // 12-bit ADC raw counts
inline float fft_mag_fullscale(uint16_t N) { return (N / 2.0f) * ADC_FS; }
inline float to_dBFS(float mag, uint16_t N)
{
  float ref = fft_mag_fullscale(N);
  if (mag <= 1e-9f)
    return -120.0f;
  return 20.0f * log10f(mag / ref);
}

// --- Fast draw (power + prefix sums) ---
static float gPow[FFT_MAX / 2];       // power per bin (k=0..Kny-1), skip 0 for display
static float gPrefixPow[FFT_MAX / 2]; // prefix sums for O(1) bin-range averages
static float gRefPow = 1.0f;          // full-scale power for dBFS reference

// -------------------- Helpers ---------------------
inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

void make_window_for_N(uint16_t N)
{
  for (uint16_t i = 0; i < N; ++i)
  {
    window_buf[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }
}

float nice_step_125(float rough)
{
  if (rough <= 0)
    return 1.0f;
  float decade = powf(10.0f, floorf(log10f(rough)));
  const float bases[3] = {1.0f, 2.0f, 5.0f};
  float best = bases[0] * decade, err = fabsf(rough - best);
  for (int i = 1; i < 3; i++)
  {
    float cand = bases[i] * decade;
    float e = fabsf(rough - cand);
    if (e < err)
    {
      err = e;
      best = cand;
    }
  }
  return best;
}

// -------------------- Axis helpers ----------------
static inline float span_y_db()
{
  float span = gYMax_dB - gYMin_dB;
  if (span < 10.0f)
    span = 10.0f;
  if (span > 140.0f)
    span = 140.0f;
  return span;
}

static inline int y_from_db(float dB)
{
  float top = gYMax_dB;
  float bot = gYMin_dB;
  if (bot > top)
  {
    float t = top;
    top = bot;
    bot = t;
  }

  float span = top - bot;
  if (span < 10.0f)
  {
    float mid = 0.5f * (top + bot);
    top = mid + 5.0f;
    bot = mid - 5.0f;
    span = 10.0f;
  }

  if (dB > top)
    dB = top;
  if (dB < bot)
    dB = bot;

  float t = (top - dB) / span;
  if (t < 0)
    t = 0;
  else if (t > 1)
    t = 1;

  int h = (int)roundf(t * (PLOT_H - 1));
  int y = PLOT_Y + h;
  if (y < PLOT_Y)
    y = PLOT_Y;
  if (y > BASE_Y)
    y = BASE_Y;
  return y;
}

static inline int bins_per_point(uint16_t N)
{
  float df = (float)gFs / (float)N;
  int Kvis = (int)floorf(fminf(gFmaxHz, 0.5f * (float)gFs) / df);
  if (Kvis < 2)
    Kvis = 2;
  float base = (float)Kvis / (float)PLOT_W;
  int bpp = (int)roundf(base * (float)gAgg);
  if (bpp < 1)
    bpp = 1;
  return bpp;
}

void format_freq_label(char *out, size_t n, float fHz)
{
  const float eps = 1e-3f;
  if (fHz < 1000.0f - eps)
  {
    snprintf(out, n, "%.0f", fHz);
  }
  else
  {
    float fk = fHz / 1000.0f;
    float nearest = roundf(fk);
    if (fabsf(fk - nearest) < 0.05f)
      snprintf(out, n, "%.0fk", nearest);
    else
      snprintf(out, n, "%.1fk", fk);
  }
}

// -------------------- UI / Axes -------------------
void draw_hud(uint16_t N, float df_eff); // forward

void draw_axes(uint16_t N)
{
  tft.fillScreen(COL_BG);

  // Title centered
  const char *title = "Audio Spectrum FFT";
  int16_t bx, by;
  uint16_t tw, th;
  tft.setTextSize(2);
  tft.getTextBounds(title, 0, 0, &bx, &by, &tw, &th);
  int tx = (SCREEN_W - (int)tw) / 2;
  int ty = PLOT_Y - 18;
  tft.setCursor(tx, ty);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.print(title);
  tft.setTextSize(1);

  // Plot box
  tft.drawFastHLine(PLOT_X, BASE_Y + 1, PLOT_W, COL_AX);
  tft.drawFastVLine(PLOT_X - 1, PLOT_Y, PLOT_H, COL_AX);

  // X ticks & labels (LIN or LOG)
  auto draw_xticks = [&]()
  {
    int labelY = BASE_Y + 10;
    int maxLabelY = SCREEN_H - HUD_H - 2;
    if (labelY > maxLabelY)
      labelY = maxLabelY;

    float nyq = (float)gFs * 0.5f;
    float fmax_draw = fminf(gFmaxHz, nyq);

    if (gXScale == XS_LIN)
    {
      float rough = fmax_draw / 6.0f;
      float major = nice_step_125(rough);
      float minor = major * 0.5f;

      for (float f = 0.0f; f <= fmax_draw + 0.01f * major; f += minor)
      {
        int x = PLOT_X + (int)roundf((f / fmax_draw) * (PLOT_W - 1));
        bool isMajor = fabsf(fmodf(f + 1e-3f, major)) < (0.02f * major);
        tft.drawFastVLine(x, BASE_Y + 1, isMajor ? 5 : 3, COL_AX);
      }
      for (float f = 0.0f; f <= fmax_draw + 0.01f * major; f += major)
      {
        int x = PLOT_X + (int)roundf((f / fmax_draw) * (PLOT_W - 1));
        char lab[12];
        format_freq_label(lab, sizeof(lab), f);
        int16_t lbx, lby;
        uint16_t ltw, lth;
        tft.getTextBounds(lab, 0, 0, &lbx, &lby, &ltw, &lth);
        int lx = x - (int)ltw / 2;
        if (lx < PLOT_X)
          lx = PLOT_X;
        if (lx + (int)ltw > PLOT_X + PLOT_W)
          lx = PLOT_X + PLOT_W - (int)ltw;
        tft.setCursor(lx, labelY);
        tft.setTextColor(COL_TEXT, COL_BG);
        tft.print(lab);
      }
    }
    else
    {
      float fmin = 10.0f;
      float fmax = fmaxf(fmax_draw, fmin * 1.01f);
      auto x_from_f = [&](float f) -> int
      {
        float t = logf(f / fmin) / logf(fmax / fmin);
        t = (t < 0) ? 0 : ((t > 1) ? 1 : t);
        return PLOT_X + (int)roundf(t * (PLOT_W - 1));
      };
      for (float decade = 10.0f; decade <= fmax * 1.001f; decade *= 10.0f)
      {
        const float mults[] = {1, 2, 3, 5};
        for (int i = 0; i < 4; i++)
        {
          float f = decade * mults[i];
          if (f < fmin || f > fmax)
            continue;
          int x = x_from_f(f);
          int len = (mults[i] == 1) ? 5 : 3;
          tft.drawFastVLine(x, BASE_Y + 1, len, COL_AX);
        }
      }
      struct Lab
      {
        float f;
        const char *s;
      };
      Lab labs[] = {{10, "10"}, {100, "100"}, {1000, "1k"}, {2000, "2k"}, {5000, "5k"}};
      for (auto &L : labs)
      {
        if (L.f > fmax)
          continue;
        int x = x_from_f(L.f);
        int16_t lbx, lby;
        uint16_t ltw, lth;
        tft.getTextBounds(L.s, 0, 0, &lbx, &lby, &ltw, &lth);
        int lx = x - (int)ltw / 2;
        if (lx < PLOT_X)
          lx = PLOT_X;
        if (lx + (int)ltw > PLOT_X + PLOT_W)
          lx = PLOT_X + PLOT_W - (int)ltw;
        tft.setCursor(lx, labelY);
        tft.setTextColor(COL_TEXT, COL_BG);
        tft.print(L.s);
      }
    }
  };
  draw_xticks();

  // Y ticks (dBFS), using explicit top/bottom
  {
    float top = gYMax_dB, bot = gYMin_dB;
    if (bot > top)
    {
      float t = top;
      top = bot;
      bot = t;
    }
    float span = top - bot;
    if (span < 10.0f)
    {
      float mid = 0.5f * (top + bot);
      top = mid + 5;
      bot = mid - 5;
    }

    int dTop = (int)ceilf(top);
    int dBot = (int)floorf(bot);

    for (int d = dTop; d >= dBot; d -= 2)
    {
      bool major = (d % 10 == 0);
      int y = y_from_db((float)d);

      if (major)
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID);
      else
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID_MINOR);

      int tickLen = major ? 6 : 3;
      tft.drawFastHLine(PLOT_X - tickLen, y, tickLen, COL_AX);

      if (major)
      {
        char lab[8];
        snprintf(lab, sizeof(lab), "%d", d);
        tft.setCursor(PLOT_X - 28, y - 3);
        tft.setTextColor(COL_TEXT, COL_BG);
        tft.print(lab);
      }
    }
  }

  gAxesDirty = false;
  gHUDDirty = true;
}

// -------------------- HUD (bottom banner, centered) ---------------
void draw_hud(uint16_t N, float df_eff)
{
  tft.fillRect(0, SCREEN_H - HUD_H, SCREEN_W, HUD_H, COL_BG);

  char hud[80];
  snprintf(hud, sizeof(hud), "Fs=%luHz  N=%u  Df=%.1fHz  Hann=%s",
           (unsigned long)gFs, N, df_eff, gUseHann ? "ON" : "OFF");

  tft.setTextSize(1);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(hud, 0, 0, &x1, &y1, &tw, &th);

  int tx = (SCREEN_W - (int)tw) / 2;
  if (tx < 0)
    tx = 0;
  int ty = SCREEN_H - HUD_H + (HUD_H - (int)th) / 2;
  if (ty < SCREEN_H - HUD_H)
    ty = SCREEN_H - HUD_H;
  if (ty > SCREEN_H - 8)
    ty = SCREEN_H - 8;

  tft.setCursor(tx, ty);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(hud);

  gHUDDirty = false;
}

// -------------------- VU Meter ---------------------
void setVU(uint8_t level)
{
  const uint8_t pins[6] = {VU1, VU2, VU3, VU4, VU5, VU6};
  for (uint8_t i = 0; i < 6; i++)
    digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}
uint8_t vu_from_peakAbs(int16_t peakAbs)
{
  float norm = (float)peakAbs / 2048.0f;
  if (norm > 1.5f)
    norm = 1.5f;
  if (norm > 1.00f)
    return 6;
  if (norm > 0.50f)
    return 5;
  if (norm > 0.25f)
    return 4;
  if (norm > 0.12f)
    return 3;
  if (norm > 0.06f)
    return 2;
  if (norm > 0.03f)
    return 1;
  return 0;
}

// -------------------- Hardware Timer Capture -------
volatile int16_t *volatile isrBuf = bufA;
volatile uint16_t isrIdx = 0;
volatile uint16_t isrN = 512; // match default N at startup
volatile bool bufReady = false;
volatile int16_t *volatile readyBuf = nullptr;

volatile int16_t isrPeakAbs = 0;

hw_timer_t *gTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer()
{
  int raw = adc1_get_raw(MIC_CH); // 0..4095
  int16_t centered = (int16_t)raw - (int16_t)gDC;
  int16_t a = centered >= 0 ? centered : -centered;

  portENTER_CRITICAL_ISR(&timerMux);
  if (a > isrPeakAbs)
    isrPeakAbs = a;
  isrBuf[isrIdx++] = centered;

  if (isrIdx >= isrN)
  {
    readyBuf = isrBuf;
    bufReady = true;
    isrBuf = (isrBuf == bufA) ? bufB : bufA;
    isrIdx = 0;
    isrPeakAbs = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void reprogram_timer(uint32_t fs)
{
  if (!gTimer)
    return;
  fs = clampi((int)fs, 2000, 40000);
  uint32_t ticks = (1000000UL + fs / 2) / fs; // 1 MHz base
  if (ticks < 1)
    ticks = 1;
  timerAlarmWrite(gTimer, ticks, true);
}

// -------------------- Horizontal zoom helpers ------
void setFmax_followNyq()
{
  gFmaxFollowNyq = true;
  gFmaxHz = 0.5f * (float)gFs;
  gAxesDirty = true;
  gHUDDirty = true;
}
void setFmax(float hz)
{
  gFmaxFollowNyq = false;
  float nyq = (float)gFs * 0.5f;
  gFmaxHz = clampf(hz, 50.0f, nyq);
  gAxesDirty = true;
  gHUDDirty = true;
}
void scaleFmax(bool up, float pct = 5.0f)
{
  float factor = up ? (1.0f + pct / 100.0f) : (1.0f - pct / 100.0f);
  setFmax(gFmaxHz * factor);
}

// -------------------- Sample rate setter -----------
void setFs(uint32_t fs)
{
  fs = clampi((int)fs, 2000, 40000);
  gFs = fs;
  reprogram_timer(gFs);
  float nyq = 0.5f * (float)gFs;
  if (gFmaxHz > nyq)
    gFmaxHz = nyq;
  gAxesDirty = true;
  gHUDDirty = true;
}

// Y range setters
void setYMax(float dB)
{
  if (dB > 20.0f)
    dB = 20.0f;
  if (dB < -60.0f)
    dB = -60.0f;
  gYMax_dB = dB;
  if (gYMin_dB > gYMax_dB - 10.0f)
    gYMin_dB = gYMax_dB - 10.0f;
  gAxesDirty = true;
  gHUDDirty = true;
}
void setYMin(float dB)
{
  if (dB < -140.0f)
    dB = -140.0f;
  if (dB > 10.0f)
    dB = 10.0f;
  gYMin_dB = dB;
  if (gYMax_dB < gYMin_dB + 10.0f)
    gYMax_dB = gYMin_dB + 10.0f;
  gAxesDirty = true;
  gHUDDirty = true;
}

// -------------------- Line rendering (O(width)) ---
void draw_line_spectrum(uint16_t N)
{
  tft.startWrite(); // batch the plot only
  tft.writeFillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, COL_BG);

  const float df = (float)gFs / (float)N;
  const int Kny = N / 2;

  float nyq = 0.5f * (float)gFs;
  float fmax = gFmaxHz;
  if (fmax > nyq)
    fmax = nyq;

  int Kvis = (int)floorf(fmax / df);
  if (Kvis > Kny - 1)
    Kvis = Kny - 1;
  if (Kvis < 2)
    Kvis = 2;

  const int bpp = bins_per_point(N);
  const float df_eff = df * (float)bpp;

  auto kc_for_x = [&](int i) -> int
  {
    if (gXScale == XS_LIN)
    {
      float t = (float)i / (float)(PLOT_W - 1);
      int kc = (int)roundf(t * (Kvis - 1));
      return clampi(kc, 1, Kvis - 1);
    }
    else
    {
      float fmin = 10.0f;
      float fmaxl = fmaxf(fmax, fmin * 1.01f);
      float t = (float)i / (float)(PLOT_W - 1);
      float f = fmin * powf(fmaxl / fmin, t);
      int k = (int)roundf(f / df);
      return clampi(k, 1, Kvis - 1);
    }
  };

  bool havePrev = false;
  int prevY = 0;

  for (int i = 0; i < PLOT_W; ++i)
  {
    int kc = kc_for_x(i);
    int half = bpp / 2;
    int k0 = kc - half;
    int k1 = k0 + bpp - 1;
    k0 = clampi(k0, 1, Kvis - 1);
    k1 = clampi(k1, 1, Kvis - 1);
    if (k1 < k0)
      k1 = k0;

    float sumP = gPrefixPow[k1] - gPrefixPow[k0 - 1];
    float meanP = sumP / (float)(k1 - k0 + 1);

    float dB = (meanP > 0.0f && gRefPow > 0.0f) ? 10.0f * log10f(meanP / gRefPow) : -120.0f;

    int x = PLOT_X + i;
    int y = y_from_db(dB);

    if (havePrev)
    {
      int y0 = (y < prevY) ? y : prevY;
      int h = abs(y - prevY) + 1;
      tft.writeFastVLine(x, y0, h, COL_LINE);
    }
    else
    {
      tft.writePixel(x, y, COL_LINE);
      havePrev = true;
    }
    prevY = y;
  }

  tft.endWrite(); // end plot batch

  if (gHUDDirty)
  {
    draw_hud(N, df_eff);
  }
}

// -------------------- Controls (buttons) -----------
struct Btn
{
  uint8_t pin;
  bool last;
  uint32_t t;
};
Btn bAggDn{BTN_AGG_DOWN, true, 0}, bAggUp{BTN_AGG_UP, true, 0},
    bNDn{BTN_N_DOWN, true, 0}, bNUp{BTN_N_UP, true, 0},
    bPause{BTN_PAUSE, true, 0}, bZoomUp{BTN_ZOOM_UP, true, 0}, bZoomDn{BTN_ZOOM_DN, true, 0};

constexpr uint16_t DEBOUNCE_MS = 25;

bool pressedEdge(Btn &b)
{
  bool nowPressed = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (nowPressed != b.last && (now - b.t) >= DEBOUNCE_MS)
  {
    b.last = nowPressed;
    b.t = now;
    return nowPressed; // true only on press edge
  }
  return false;
}

void setFPS(uint16_t v)
{
  gFPS = clampi(v, 10, 120);
  gHUDDirty = true;
}
void setNidx(uint8_t i)
{
  if (i >= sizeof(N_CHOICES) / sizeof(N_CHOICES[0]))
    i = (sizeof(N_CHOICES) / sizeof(N_CHOICES[0])) - 1;
  gNidx = i;
  uint16_t N = N_CHOICES[gNidx];
  make_window_for_N(N);
  portENTER_CRITICAL(&timerMux);
  isrN = N;
  isrIdx = 0;
  portEXIT_CRITICAL(&timerMux);
  gAxesDirty = true;
  gHUDDirty = true;
}
void setAgg(uint8_t v)
{
  if (v < 1)
    v = 1;
  if (v > 64)
    v = 64;
  gAgg = v;
  gHUDDirty = true;
}
void setXScale(uint8_t m)
{
  gXScale = (m ? XS_LOG : XS_LIN);
  gAxesDirty = true;
  gHUDDirty = true;
}

void pollButtons()
{
  if (pressedEdge(bPause))
    gPaused = !gPaused;

  // Aggregation
  if (pressedEdge(bAggDn))
    setAgg(gAgg > 1 ? gAgg - 1 : 1);
  if (pressedEdge(bAggUp))
    setAgg(gAgg + 1);

  // N
  if (pressedEdge(bNDn))
    setNidx(gNidx > 0 ? gNidx - 1 : 0);
  if (pressedEdge(bNUp))
    setNidx(gNidx + 1);

  // Horizontal zoom via Fmax (±5%)
  if (pressedEdge(bZoomDn))
    scaleFmax(false, 5.0f); // zoom out
  if (pressedEdge(bZoomUp))
    scaleFmax(true, 5.0f); // zoom in
}

// -------------------- Serial Console ---------------
static char cmdBuf[96];
static uint8_t cmdLen = 0;

void print_controls()
{
  Serial.println();
  Serial.println(F("=== Controls (Buttons) ==="));
  Serial.println(F("  PAUSE: toggle Play/Pause"));
  Serial.println(F("  AGG- / AGG+:   aggregation finer/coarser"));
  Serial.println(F("  N- / N+:       FFT length down/up"));
  Serial.println(F("  ZOOM- / ZOOM+: Horizontal zoom (Fmax -/+ 5%)"));
  Serial.println();
  Serial.println(F("=== Serial Commands (with examples) ==="));
  Serial.println(F("  help                # show this help"));
  Serial.println(F("  stats               # print current settings"));
  Serial.println(F("  fs=2000..40000      # set sample rate, e.g. fs=15000"));
  Serial.println(F("  n=32|64|128|256|512|1024|2048   # e.g. n=512"));
  Serial.println(F("  fps=10..120         # e.g. fps=60"));
  Serial.println(F("  agg=1..64           # e.g. agg=4 (coarser plot)"));
  Serial.println(F("  xscale=lin|log      # set X axis scale, e.g. xscale=lin"));
  Serial.println(F("  fmax=Hz             # set max plotted freq (horizontal zoom), e.g. fmax=3500"));
  Serial.println(F("  fmax=nyq            # make Fmax follow Nyquist (Fs/2) again"));
  Serial.println(F("  ymax=<dB>           # set top of Y axis in dBFS, e.g. ymax=-10"));
  Serial.println(F("  ymin=<dB>           # set bottom of Y axis in dBFS, e.g. ymin=-80"));
  Serial.println(F("  hann=0|1            # Hann window off/on, e.g. hann=1"));
  Serial.println(F("  pause               # toggle pause"));
  Serial.println();
}

void print_stats()
{
  uint16_t N = N_CHOICES[gNidx];
  int bpp = bins_per_point(N);
  float df_eff = (float)gFs / N * bpp;
  Serial.printf("Fs=%lu  N=%u  Df=%.2fHz  FPS=%u  BPP=%d  X=%s  Fmax=%.0fHz%s  Hann=%d\r\n",
                (unsigned long)gFs, N, df_eff, gFPS, bpp, (gXScale == XS_LIN ? "LIN" : "LOG"),
                gFmaxHz, gFmaxFollowNyq ? " (nyq)" : "", (int)gUseHann);
  Serial.printf("Y range: [%.1f, %.1f] dBFS\r\n", gYMin_dB, gYMax_dB);
}

void print_prompt() { Serial.print("> "); }

void apply_command(const char *s)
{
  if (!s || !*s)
    return;
  if (!strcmp(s, "help"))
  {
    print_controls();
  }
  else if (!strcmp(s, "stats"))
  {
    print_stats();
  }
  else if (!strncmp(s, "fs=", 3))
  {
    setFs((uint32_t)atoi(s + 3));
  }
  else if (!strncmp(s, "n=", 2))
  {
    uint16_t want = atoi(s + 2);
    for (uint8_t i = 0; i < sizeof(N_CHOICES) / sizeof(N_CHOICES[0]); i++)
      if (N_CHOICES[i] == want)
      {
        setNidx(i);
        break;
      }
  }
  else if (!strncmp(s, "fps=", 4))
  {
    setFPS(atoi(s + 4));
  }
  else if (!strncmp(s, "agg=", 4))
  {
    setAgg((uint8_t)atoi(s + 4));
  }
  else if (!strncmp(s, "xscale=", 7))
  {
    if (!strcasecmp(s + 7, "lin"))
      setXScale(0);
    else if (!strcasecmp(s + 7, "log"))
      setXScale(1);
  }
  else if (!strncmp(s, "fmax=", 5))
  {
    if (!strcasecmp(s + 5, "nyq"))
      setFmax_followNyq();
    else
      setFmax(atof(s + 5));
  }
  else if (!strncmp(s, "ymax=", 5))
  {
    setYMax((float)atof(s + 5));
  }
  else if (!strncmp(s, "ymin=", 5))
  {
    setYMin((float)atof(s + 5));
  }
  else if (!strncmp(s, "hann=", 5))
  {
    gUseHann = (atoi(s + 5) != 0);
    gAxesDirty = true;
    gHUDDirty = true;
  }
  else if (!strcmp(s, "pause"))
  {
    gPaused = !gPaused;
  }
  else
  {
    Serial.println(F("Unknown. Type 'help'."));
  }
}

void service_serial()
{
  while (Serial.available())
  {
    int c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      cmdBuf[cmdLen] = 0;
      Serial.print("\r\n");
      apply_command(cmdBuf);
      cmdLen = 0;
      print_stats();
      print_prompt();
    }
    else if (c == 8 || c == 127)
    { // backspace
      if (cmdLen > 0)
      {
        cmdLen--;
        Serial.print("\b \b");
      }
    }
    else if (c >= 32 && c < 127)
    {
      if (cmdLen < sizeof(cmdBuf) - 1)
      {
        cmdBuf[cmdLen++] = (char)c;
        Serial.write(c);
      }
    }
  }
}

// -------------------- Setup / Loop -----------------
static uint16_t quick_dc_estimate()
{
  uint32_t s = 0;
  for (int i = 0; i < 256; i++)
  {
    s += adc1_get_raw(MIC_CH);
    delayMicroseconds(100);
  }
  return (uint16_t)(s / 256);
}

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("FFT Spectrum (Line) - HW Timer ADC, Fs default 15 kHz (serial-adjustable), Fmax=horizontal zoom"));
  print_controls();
  print_stats();
  print_prompt();

  // Buttons
  pinMode(BTN_AGG_DOWN, INPUT_PULLUP);
  pinMode(BTN_AGG_UP, INPUT_PULLUP);
  pinMode(BTN_N_DOWN, INPUT_PULLUP);
  pinMode(BTN_N_UP, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_ZOOM_UP, INPUT_PULLUP);
  pinMode(BTN_ZOOM_DN, INPUT_PULLUP);

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

  // ADC1 config (fast path)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(MIC_CH, ADC_ATTEN_DB_12);
  gDC = quick_dc_estimate();

  // esp-dsp init
  dsps_fft2r_init_fc32(nullptr, FFT_MAX);

  // Window
  make_window_for_N(N_CHOICES[gNidx]);

  // Timer @ 1 MHz tick
  gTimer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(gTimer, &onTimer, true);
  reprogram_timer(gFs);
  timerAlarmEnable(gTimer);

  // Set ISR N
  portENTER_CRITICAL(&timerMux);
  isrN = N_CHOICES[gNidx];
  isrIdx = 0;
  isrBuf = bufA;
  readyBuf = nullptr;
  bufReady = false;
  isrPeakAbs = 0;
  portEXIT_CRITICAL(&timerMux);

  // Start with Fmax acting as horizontal zoom, clamped to Nyquist
  gFmaxFollowNyq = false;
  gFmaxHz = clampf(3500.0f, 50.0f, 0.5f * (float)gFs);

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

  // FFT input
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
