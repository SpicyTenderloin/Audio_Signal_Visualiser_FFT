#include "Display.h"
#include "Globals.h"
#include "DSPUtils.h"
#include <math.h>
#include <limits.h>

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

// Maps a linear amplitude fraction of full scale (0.0-1.0) to a plot row.
// Unlike y_from_db, this range is fixed (0-100%), not user-adjustable.
static inline int y_from_linear(float frac)
{
  if (frac > 1.0f)
    frac = 1.0f;
  if (frac < 0.0f)
    frac = 0.0f;

  int h = (int)roundf((1.0f - frac) * (PLOT_H - 1));
  int y = PLOT_Y + h;
  if (y < PLOT_Y)
    y = PLOT_Y;
  if (y > BASE_Y)
    y = BASE_Y;
  return y;
}

// Last frame's per-column plot row, so draw_line_spectrum() can erase only
// the pixels it actually touched instead of clearing the whole plot rect
// every frame. Invalidated whenever the plot area gets wiped some other way
// (a full draw_axes() redraw).
static int16_t s_prevLineY[PLOT_W];
static bool s_prevLineValid = false;

// The true background of each plot pixel, split into a per-row part (index
// 0 = PLOT_Y) and a per-column part (index 0 = PLOT_X), rebuilt by
// draw_axes() whenever it redraws. Horizontal gridlines are drawn after
// vertical ones there, so at an intersection the row color wins - see
// bg_at() below. Erasing the previous trace has to restore this, not flat
// COL_BG, or it punches gridline pixels out wherever the old trace crossed
// them.
static uint16_t s_rowBG[PLOT_H];
// COL_BG doubles as "no vertical gridline in this column" - no separate
// bool array needed, since a real gridline color is never COL_BG.
static uint16_t s_colVGridColor[PLOT_W];

// True background color at plot column i (0 = PLOT_X), row r (screen Y).
static inline uint16_t bg_at(int i, int r)
{
  uint16_t rowColor = s_rowBG[r - PLOT_Y];
  if (rowColor != COL_BG)
    return rowColor;
  if (s_colVGridColor[i] != COL_BG)
    return s_colVGridColor[i];
  return COL_BG;
}

// Draws the connected polyline described by yArr (one row per column) in a
// single flat color - used to draw the new trace.
static void draw_polyline(const int16_t *yArr, uint16_t color)
{
  bool havePrev = false;
  int prevY = 0;
  for (int i = 0; i < PLOT_W; ++i)
  {
    int y = yArr[i];
    int x = PLOT_X + i;
    if (havePrev)
    {
      int y0 = (y < prevY) ? y : prevY;
      int h = abs(y - prevY) + 1;
      tft.writeFastVLine(x, y0, h, color);
    }
    else
    {
      tft.writePixel(x, y, color);
      havePrev = true;
    }
    prevY = y;
  }
}

// Erases the connected polyline described by yArr by restoring each pixel's
// true background (s_rowBG) rather than flat COL_BG, so gridline rows the
// old trace crossed come back instead of staying punched out. Runs of
// identical background color are coalesced into one writeFastVLine call.
static void erase_polyline(const int16_t *yArr)
{
  bool havePrev = false;
  int prevY = 0;
  for (int i = 0; i < PLOT_W; ++i)
  {
    int y = yArr[i];
    int x = PLOT_X + i;
    int y0 = havePrev ? ((y < prevY) ? y : prevY) : y;
    int h = havePrev ? (abs(y - prevY) + 1) : 1;

    int runStart = y0;
    uint16_t runColor = bg_at(i, runStart);
    for (int r = y0 + 1; r < y0 + h; ++r)
    {
      uint16_t c = bg_at(i, r);
      if (c != runColor)
      {
        tft.writeFastVLine(x, runStart, r - runStart, runColor);
        runStart = r;
        runColor = c;
      }
    }
    tft.writeFastVLine(x, runStart, (y0 + h) - runStart, runColor);

    havePrev = true;
    prevY = y;
  }
}

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

  for (int i = 0; i < PLOT_W; ++i)
    s_colVGridColor[i] = COL_BG;

  // X ticks, labels, and vertical gridlines (LIN or LOG)
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

        uint16_t gridColor = isMajor ? COL_GRID : COL_GRID_MINOR;
        tft.drawFastVLine(x, PLOT_Y, PLOT_H, gridColor);
        int col = x - PLOT_X;
        if (col >= 0 && col < PLOT_W)
        {
          s_colVGridColor[col] = gridColor;
        }
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
          bool isMajor = (mults[i] == 1);
          int len = isMajor ? 5 : 3;
          tft.drawFastVLine(x, BASE_Y + 1, len, COL_AX);

          uint16_t gridColor = isMajor ? COL_GRID : COL_GRID_MINOR;
          tft.drawFastVLine(x, PLOT_Y, PLOT_H, gridColor);
          int col = x - PLOT_X;
          if (col >= 0 && col < PLOT_W)
          {
            s_colVGridColor[col] = gridColor;
          }
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

  // Y ticks. The topmost major tick's label carries the unit suffix (dB or
  // %, for full-scale amplitude), so the axis is self-labeling without
  // needing separate room for a unit caption.
  for (int r = 0; r < PLOT_H; ++r)
    s_rowBG[r] = COL_BG;

  if (gYScale == YS_DB)
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
    bool firstMajor = true;

    for (int d = dTop; d >= dBot; d -= 2)
    {
      bool major = (d % 10 == 0);
      int y = y_from_db((float)d);

      if (major)
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID);
      else
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID_MINOR);
      s_rowBG[y - PLOT_Y] = major ? COL_GRID : COL_GRID_MINOR;

      int tickLen = major ? 6 : 3;
      tft.drawFastHLine(PLOT_X - tickLen, y, tickLen, COL_AX);

      if (major)
      {
        char lab[12];
        snprintf(lab, sizeof(lab), firstMajor ? "%ddB" : "%d", d);
        firstMajor = false;
        tft.setCursor(PLOT_X - 28, y - 3);
        tft.setTextColor(COL_TEXT, COL_BG);
        tft.print(lab);
      }
    }
  }
  else // YS_LIN: fixed 0-100% of full-scale amplitude
  {
    const int majorStepPct = 20;
    const int minorStepPct = 10;
    bool firstMajor = true;

    for (int pct = 100; pct >= 0; pct -= minorStepPct)
    {
      bool major = (pct % majorStepPct == 0);
      int y = y_from_linear(pct / 100.0f);

      if (major)
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID);
      else
        tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID_MINOR);
      s_rowBG[y - PLOT_Y] = major ? COL_GRID : COL_GRID_MINOR;

      int tickLen = major ? 6 : 3;
      tft.drawFastHLine(PLOT_X - tickLen, y, tickLen, COL_AX);

      if (major)
      {
        char lab[12];
        snprintf(lab, sizeof(lab), firstMajor ? "%d%%" : "%d", pct);
        firstMajor = false;
        tft.setCursor(PLOT_X - 28, y - 3);
        tft.setTextColor(COL_TEXT, COL_BG);
        tft.print(lab);
      }
    }
  }

  s_prevLineValid = false; // whole screen just got wiped, nothing to erase next frame
  gAxesDirty = false;
  gHUDDirty = true;
}

// The HUD band is split into independently-redrawn regions so the once/sec
// FPS update only has to touch the few digits that actually change, not
// the "FPS:" label or the rest of the band - avoiding any visible flicker.
// All share the same fixed baseline, since text size 1 is a known 8px tall
// - no need for getTextBounds() just to vertically center it.
static const int HUD_TEXT_Y = SCREEN_H - HUD_H + (HUD_H - 8) / 2;
static const int HUD_FPS_LABEL_W = 24; // "FPS:" at 6px/char
static const int HUD_FPS_VAL_W = 24;   // up to 4 digits at 6px/char
static const int HUD_FPS_W = HUD_FPS_LABEL_W + HUD_FPS_VAL_W;
static const int HUD_FPS_X = SCREEN_W - HUD_FPS_W;
static const int HUD_FPS_VAL_X = HUD_FPS_X + HUD_FPS_LABEL_W;

void draw_hud(uint16_t N, float df_eff)
{
  tft.fillRect(0, SCREEN_H - HUD_H, HUD_FPS_X + HUD_FPS_LABEL_W, HUD_H, COL_BG);

  char hud[80];
  snprintf(hud, sizeof(hud), "Fs=%luHz  N=%u  Df=%.1fHz  Hann=%s",
           (unsigned long)gFs, N, df_eff, gUseHann ? "ON" : "OFF");

  tft.setTextSize(1);
  tft.setCursor(4, HUD_TEXT_Y);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(hud);

  tft.setCursor(HUD_FPS_X, HUD_TEXT_Y);
  tft.print("FPS:");

  gHUDDirty = false;

  // force=true: draw_axes() may have just wiped these pixels with a
  // full-screen clear, so the "unchanged" skip below can't apply here.
  draw_hud_fps(true);
}

void draw_hud_fps(bool force)
{
  // Skip the redraw entirely if the displayed (rounded) value hasn't
  // actually changed, so small float jitter around a whole number doesn't
  // still flicker the digits once/sec.
  static int s_lastShown = INT_MIN;
  int shown = (int)lroundf(gMeasuredFPS);
  if (!force && shown == s_lastShown)
    return;
  s_lastShown = shown;

  tft.fillRect(HUD_FPS_VAL_X, SCREEN_H - HUD_H, HUD_FPS_VAL_W, HUD_H, COL_BG);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", shown);

  tft.setTextSize(1);
  tft.setCursor(HUD_FPS_VAL_X, HUD_TEXT_Y);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(buf);
}

void draw_line_spectrum(uint16_t N)
{
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

  // Compute this frame's row per column first (no drawing yet), so the
  // actual SPI writes below only ever touch the pixels that changed.
  int16_t yArr[PLOT_W];
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

    int y;
    if (gYScale == YS_DB)
    {
      // meanP/gRefPow is a power ratio; 10*log10(power ratio) == 20*log10(amplitude ratio) == dBFS.
      float dB = (meanP > 0.0f && gRefPow > 0.0f) ? 10.0f * log10f(meanP / gRefPow) : -120.0f;
      y = y_from_db(dB);
    }
    else
    {
      float frac = (meanP > 0.0f && gRefPow > 0.0f) ? sqrtf(meanP / gRefPow) : 0.0f;
      y = y_from_linear(frac);
    }
    yArr[i] = (int16_t)y;
  }

  tft.startWrite(); // batch the plot only
  if (s_prevLineValid)
    erase_polyline(s_prevLineY);   // restore true background (incl. gridlines) where the old trace was...
  draw_polyline(yArr, COL_LINE);   // ...instead of clearing the whole plot rect
  tft.endWrite();                  // end plot batch

  for (int i = 0; i < PLOT_W; ++i)
    s_prevLineY[i] = yArr[i];
  s_prevLineValid = true;

  if (gHUDDirty)
  {
    draw_hud(N, df_eff);
  }
}
