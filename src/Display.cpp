#include "Display.h"
#include "Globals.h"
#include "DSPUtils.h"
#include <math.h>
#include <limits.h>
#include <string.h>

// -------------------- Axis helpers ----------------
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

// Maps a centered raw ADC sample to a plot row, against the waveform mode's
// adjustable ±gWaveYRange span (the time-domain analog of y_from_db).
static inline int y_from_amplitude(int16_t sample)
{
  float top = gWaveYRange;
  float bot = -gWaveYRange;
  float v = (float)sample;
  if (v > top)
    v = top;
  if (v < bot)
    v = bot;

  float t = (top - v) / (top - bot);
  int h = (int)roundf(t * (PLOT_H - 1));
  int y = PLOT_Y + h;
  if (y < PLOT_Y)
    y = PLOT_Y;
  if (y > BASE_Y)
    y = BASE_Y;
  return y;
}

// Last frame's per-column plot row, so draw_line_spectrum()/draw_waveform()
// can erase only the pixels they actually touched instead of clearing the
// whole plot rect every frame. Invalidated whenever the plot area gets
// wiped some other way (a full draw_axes() redraw).
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

// Column i's full vertical span for the trace: its own sample plus half the
// transition toward each neighbor (the midpoint between adjacent samples).
// Adjacent columns' spans therefore meet exactly at that shared midpoint
// with no gap, which is what a true sloped line between the two sample
// points would look like anyway when it can only be 1px wide - but unlike
// drawing that line directly, this stays one run per column (same cost as
// the old "whole riser on the new column" approach, no extra SPI calls),
// and reduces to a single point when a column has no vertical change.
static inline void column_span(const int16_t *yArr, int i, int *outY0, int *outH)
{
  int y = yArr[i];
  int lo = y, hi = y;
  if (i > 0)
  {
    int leftMid = (yArr[i - 1] + y) / 2;
    if (leftMid < lo)
      lo = leftMid;
    if (leftMid > hi)
      hi = leftMid;
  }
  if (i < PLOT_W - 1)
  {
    int rightMid = (y + yArr[i + 1]) / 2;
    if (rightMid < lo)
      lo = rightMid;
    if (rightMid > hi)
      hi = rightMid;
  }
  *outY0 = lo;
  *outH = hi - lo + 1;
}

// Draws the connected polyline described by yArr (one row per column) in a
// single flat color - used to draw the new trace.
static void draw_polyline(const int16_t *yArr, uint16_t color)
{
  for (int i = 0; i < PLOT_W; ++i)
  {
    int y0, h;
    column_span(yArr, i, &y0, &h);
    tft.writeFastVLine(PLOT_X + i, y0, h, color);
  }
}

// Erases the connected polyline described by yArr by restoring each pixel's
// true background (s_rowBG) rather than flat COL_BG, so gridline rows the
// old trace crossed come back instead of staying punched out. Runs of
// identical background color are coalesced into one writeFastVLine call.
static void erase_polyline(const int16_t *yArr)
{
  for (int i = 0; i < PLOT_W; ++i)
  {
    int x = PLOT_X + i;
    int y0, h;
    column_span(yArr, i, &y0, &h);

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
  }
}

// -------------------- X/Y tick drawing, per mode ----------------
// Both populate s_colVGridColor/s_rowBG as they go, same as each other, so
// the erase machinery above stays generic across modes.

// Row reserved for the Y-axis unit caption (see draw_axis_unit_labels()):
// whichever major Y-tick would land nearest the plot's vertical center has
// its text suppressed below (gridline and tick stub still draw as normal),
// freeing a collision-free row for the caption in an otherwise fully-used
// left margin.
static const int AXIS_UNIT_ROW_Y = PLOT_Y + PLOT_H / 2;
static inline bool is_axis_unit_row(int y) { return abs(y - AXIS_UNIT_ROW_Y) <= 5; }

static void draw_xticks_fft()
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
      if (lx < 0)
        lx = 0;
      // Right bound stays PLOT_X+PLOT_W (not SCREEN_W): the strip beyond it
      // is reserved for the X-axis unit caption, see draw_axis_unit_labels().
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
      if (lx < 0)
        lx = 0;
      if (lx + (int)ltw > PLOT_X + PLOT_W)
        lx = PLOT_X + PLOT_W - (int)ltw;
      tft.setCursor(lx, labelY);
      tft.setTextColor(COL_TEXT, COL_BG);
      tft.print(L.s);
    }
  }
}

// Draws a Y-tick label right-aligned against its tick stub (ending at
// PLOT_X - tickLen - 2), rather than left-anchored at a fixed offset. A
// fixed-left anchor only works as long as every label is short enough to
// fit before it starts overlapping the plot - true for FFT's 2-4 char dB/%
// labels, but not for the waveform axis's wider values. Anchoring to the
// tick instead means any label width stays clear of the plot, using
// however much of the margin it actually needs.
static void draw_ytick_label(const char *lab, int y, int tickLen)
{
  int w = (int)strlen(lab) * 6; // default GFX font: fixed 6px/char advance at size 1
  int x = PLOT_X - tickLen - 2 - w;
  if (x < 0)
    x = 0;
  tft.setCursor(x, y - 3);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(lab);
}

static void draw_yticks_fft()
{
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

      if (major && !is_axis_unit_row(y))
      {
        char lab[12];
        snprintf(lab, sizeof(lab), "%d", d);
        draw_ytick_label(lab, y, tickLen);
      }
    }
  }
  else // YS_LIN: fixed 0-100% of full-scale amplitude
  {
    const int majorStepPct = 20;
    const int minorStepPct = 10;

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

      if (major && !is_axis_unit_row(y))
      {
        char lab[12];
        snprintf(lab, sizeof(lab), "%d", pct);
        draw_ytick_label(lab, y, tickLen);
      }
    }
  }
}

// X ticks for the waveform view: time across the plot width (0..span),
// where span = PLOT_W samples at the current Fs. Structurally the same as
// the FFT view's linear frequency ticks, just in time instead of Hz.
static void draw_xticks_waveform()
{
  int labelY = BASE_Y + 10;
  int maxLabelY = SCREEN_H - HUD_H - 2;
  if (labelY > maxLabelY)
    labelY = maxLabelY;

  float spanMs = (float)PLOT_W / (float)gFs * 1000.0f;
  float rough = spanMs / 6.0f;
  float major = nice_step_125(rough);
  float minor = major * 0.5f;

  for (float t = 0.0f; t <= spanMs + 0.01f * major; t += minor)
  {
    int x = PLOT_X + (int)roundf((t / spanMs) * (PLOT_W - 1));
    bool isMajor = fabsf(fmodf(t + 1e-3f, major)) < (0.02f * major);
    tft.drawFastVLine(x, BASE_Y + 1, isMajor ? 5 : 3, COL_AX);

    uint16_t gridColor = isMajor ? COL_GRID : COL_GRID_MINOR;
    tft.drawFastVLine(x, PLOT_Y, PLOT_H, gridColor);
    int col = x - PLOT_X;
    if (col >= 0 && col < PLOT_W)
    {
      s_colVGridColor[col] = gridColor;
    }
  }
  // No unit suffix on any tick (unlike the old firstMajor="Xms" approach):
  // on a horizontal axis, a label that's a couple of characters wider than
  // its neighbors risks running straight into them - the FFT X-axis never
  // appended one for the same reason. The unit lives in its own caption
  // (draw_axis_unit_labels()) instead.
  for (float t = 0.0f; t <= spanMs + 0.01f * major; t += major)
  {
    int x = PLOT_X + (int)roundf((t / spanMs) * (PLOT_W - 1));
    char lab[12];
    format_time_label(lab, sizeof(lab), t);
    int16_t lbx, lby;
    uint16_t ltw, lth;
    tft.getTextBounds(lab, 0, 0, &lbx, &lby, &ltw, &lth);
    int lx = x - (int)ltw / 2;
    if (lx < 0)
      lx = 0;
    if (lx + (int)ltw > PLOT_X + PLOT_W)
      lx = PLOT_X + PLOT_W - (int)ltw;
    tft.setCursor(lx, labelY);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.print(lab);
  }
}

// Y ticks for the waveform view: centered amplitude, displayed in volts
// (±gWaveYRange raw ADC counts, converted via ADC_VREF/ADC_FS) since raw
// codes aren't a meaningful unit to read off a scope-style display. The
// "nice" step is computed in volts directly, then converted back to counts
// per tick only to find its pixel row - so gridlines land on round volt
// values (0.5V, 1.0V, ...) rather than round-but-arbitrary code counts.
// Steps by integer multiples of `minor` (rather than the FFT Y axis's
// fmodf-based major/minor test) so it stays exact on both sides of zero -
// fmodf's sign behavior on negative values would otherwise misjudge which
// ticks are major below the centerline.
static void draw_yticks_waveform()
{
  float rangeV = gWaveYRange * ADC_VREF / ADC_FS;
  float rough = rangeV / 3.0f;
  float major = nice_step_125(rough);
  float minor = major * 0.5f; // always exactly major/2, so "every 2nd step" below is exact

  int maxStep = (int)floorf(rangeV / minor);

  for (int step = maxStep; step >= -maxStep; --step)
  {
    float v = step * minor;                 // volts
    float counts = v * ADC_FS / ADC_VREF;    // back to raw counts, for the pixel row
    bool major_ = (step % 2 == 0);
    int y = y_from_amplitude((int16_t)roundf(counts));

    if (major_)
      tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID);
    else
      tft.drawFastHLine(PLOT_X, y, PLOT_W, COL_GRID_MINOR);
    s_rowBG[y - PLOT_Y] = major_ ? COL_GRID : COL_GRID_MINOR;

    int tickLen = major_ ? 6 : 3;
    tft.drawFastHLine(PLOT_X - tickLen, y, tickLen, COL_AX);

    if (major_ && !is_axis_unit_row(y))
    {
      char lab[12];
      format_volts_label(lab, sizeof(lab), v, major);
      draw_ytick_label(lab, y, tickLen);
    }
  }
}

// Unit captions for each axis, placed in the plot's own margins rather than
// the title row (crowded, and unrelated to the title itself):
//   - Y-axis unit: horizontally centered in the left margin, on the row
//     draw_yticks_*() left clear via is_axis_unit_row() - so it reads as
//     "this whole column of numbers is volts/dB/%" without overlapping any
//     of them.
//   - X-axis unit: right margin, same row as the X tick labels - those are
//     kept clear of this strip by the PLOT_X+PLOT_W clamp in
//     draw_xticks_fft()/draw_xticks_waveform().
static void draw_axis_unit_labels()
{
  const char *yUnit = (gDisplayMode == MODE_FFT) ? (gYScale == YS_DB ? "dB" : "%") : "V";
  const char *xUnit = (gDisplayMode == MODE_FFT) ? "Hz" : "ms";
  tft.setTextColor(COL_TEXT, COL_BG);

  int yw = (int)strlen(yUnit) * 6;
  int yx = (PLOT_X - yw) / 2;
  if (yx < 0)
    yx = 0;
  tft.setCursor(yx, AXIS_UNIT_ROW_Y - 3);
  tft.print(yUnit);

  int labelY = BASE_Y + 10;
  int maxLabelY = SCREEN_H - HUD_H - 2;
  if (labelY > maxLabelY)
    labelY = maxLabelY;
  int xw = (int)strlen(xUnit) * 6;
  tft.setCursor(SCREEN_W - 2 - xw, labelY);
  tft.print(xUnit);
}

void draw_axes(uint16_t N)
{
  tft.fillScreen(COL_BG);

  // Title centered
  const char *title = (gDisplayMode == MODE_FFT) ? "Spectrum Analyser" : "Waveform Analyser";
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
  for (int r = 0; r < PLOT_H; ++r)
    s_rowBG[r] = COL_BG;

  if (gDisplayMode == MODE_FFT)
  {
    draw_xticks_fft();
    draw_yticks_fft();
  }
  else
  {
    draw_xticks_waveform();
    draw_yticks_waveform();
  }
  draw_axis_unit_labels();

  s_prevLineValid = false; // whole screen just got wiped, nothing to erase next frame
  gAxesDirty = false;
  gHUDDirty = true;
}

// The HUD is a row of evenly-spaced fields, centered as a whole - the field
// set depends on gDisplayMode. The FPS value is a fixed-width slot
// (HUD_FPS_VAL_CHARS digits) within the last field, so its position stays
// put and draw_hud_fps() can redraw just those digits once/sec without
// recomputing (or flickering) the rest of the row. The default GFX font
// advances a fixed HUD_CHAR_W px/char at text size 1, so field widths can
// be computed from string length alone.
static const int HUD_TEXT_Y = SCREEN_H - HUD_H + (HUD_H - 8) / 2;
static const int HUD_CHAR_W = 6;
static const int HUD_GAP = 10; // even spacing between fields
static const int HUD_FPS_VAL_CHARS = 3;

static int s_hudFpsValX = 0; // set by draw_hud(), reused by draw_hud_fps()

void draw_hud()
{
  tft.fillRect(0, SCREEN_H - HUD_H, SCREEN_W, HUD_H, COL_BG);

  char f0[24], f1[20], f2[20], f3[12];
  const char *fields[4];
  int fieldCount;

  if (gDisplayMode == MODE_FFT)
  {
    uint16_t N = N_CHOICES[gNidx];
    float df = (float)gFs / (float)N;
    float df_eff = df * (float)bins_per_point(N);

    snprintf(f0, sizeof(f0), "Fs=%.1fkHz", (double)gFs / 1000.0);
    snprintf(f1, sizeof(f1), "N=%u", N);
    snprintf(f2, sizeof(f2), "Df=%.1fHz", (double)df_eff);
    snprintf(f3, sizeof(f3), "Hann=%s", gUseHann ? "ON" : "OFF");
    fields[0] = f0;
    fields[1] = f1;
    fields[2] = f2;
    fields[3] = f3;
    fieldCount = 4;
  }
  else // MODE_WAVEFORM
  {
    float spanMs = (float)PLOT_W / (float)gFs * 1000.0f;
    snprintf(f0, sizeof(f0), "Fs=%.1fkHz", (double)gFs / 1000.0);
    snprintf(f1, sizeof(f1), "Span=%.1fms", (double)spanMs);
    fields[0] = f0;
    fields[1] = f1;
    fieldCount = 2;
  }

  static const char *fpsLabel = "FPS:";
  int fieldsW = 0;
  for (int i = 0; i < fieldCount; i++)
    fieldsW += (int)strlen(fields[i]) * HUD_CHAR_W;
  int wFpsLabel = (int)strlen(fpsLabel) * HUD_CHAR_W;
  int wFpsVal = HUD_FPS_VAL_CHARS * HUD_CHAR_W;

  int total = fieldsW + wFpsLabel + wFpsVal + HUD_GAP * fieldCount;
  int x = (SCREEN_W - total) / 2;
  if (x < 0)
    x = 0;

  tft.setTextSize(1);
  tft.setTextColor(COL_TEXT, COL_BG);
  for (int i = 0; i < fieldCount; i++)
  {
    tft.setCursor(x, HUD_TEXT_Y);
    tft.print(fields[i]);
    x += (int)strlen(fields[i]) * HUD_CHAR_W + HUD_GAP;
  }
  tft.setCursor(x, HUD_TEXT_Y);
  tft.print(fpsLabel);
  x += wFpsLabel;
  s_hudFpsValX = x; // value slot immediately follows the "FPS:" label

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

  tft.fillRect(s_hudFpsValX, SCREEN_H - HUD_H, HUD_FPS_VAL_CHARS * HUD_CHAR_W, HUD_H, COL_BG);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", shown);

  tft.setTextSize(1);
  tft.setCursor(s_hudFpsValX, HUD_TEXT_Y);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(buf);
}

void draw_line_spectrum(uint16_t N)
{
  const float df = (float)gFs / (float)N;

  float nyq = 0.5f * (float)gFs;
  float fmax = gFmaxHz;
  if (fmax > nyq)
    fmax = nyq;

  const int Kvis = visible_bin_count(N);
  const int bpp = bins_per_point(N);

  // Loop-invariant for the log-scale branch below (doesn't depend on the
  // per-column i), hoisted out so it's computed once per frame rather than
  // once per plot column.
  const float logFmin = 10.0f;
  const float logFmaxRatio = fmaxf(fmax, logFmin * 1.01f) / logFmin;

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
      float t = (float)i / (float)(PLOT_W - 1);
      float f = logFmin * powf(logFmaxRatio, t);
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
    erase_polyline(s_prevLineY); // restore true background (incl. gridlines) where the old trace was...
  draw_polyline(yArr, COL_LINE); // ...instead of clearing the whole plot rect
  tft.endWrite();                // end plot batch

  for (int i = 0; i < PLOT_W; ++i)
    s_prevLineY[i] = yArr[i];
  s_prevLineValid = true;

  if (gHUDDirty)
    draw_hud();
}

void draw_waveform(const int16_t *samples)
{
  int16_t yArr[PLOT_W];
  for (int i = 0; i < PLOT_W; ++i)
    yArr[i] = (int16_t)y_from_amplitude(samples[i]);

  tft.startWrite();
  if (s_prevLineValid)
    erase_polyline(s_prevLineY);
  draw_polyline(yArr, COL_LINE);
  tft.endWrite();

  for (int i = 0; i < PLOT_W; ++i)
    s_prevLineY[i] = yArr[i];
  s_prevLineValid = true;

  if (gHUDDirty)
    draw_hud();
}
