#include "Display.h"
#include "Globals.h"
#include "DSPUtils.h"
#include <math.h>

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

  // Y ticks. The topmost major tick's label carries the unit suffix (dB or
  // %, for full-scale amplitude), so the axis is self-labeling without
  // needing separate room for a unit caption.
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

  gAxesDirty = false;
  gHUDDirty = true;
}

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

    int x = PLOT_X + i;
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
