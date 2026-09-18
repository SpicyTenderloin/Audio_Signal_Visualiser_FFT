#include "Settings.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "AudioCapture.h"

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

void scaleFmax(bool up, float pct)
{
  float factor = up ? (1.0f + pct / 100.0f) : (1.0f - pct / 100.0f);
  setFmax(gFmaxHz * factor);
}

void setFs(uint32_t fs)
{
  fs = clampi((int)fs, 2000, 200000);
  gFs = fs;
  set_sample_rate(gFs);
  float nyq = 0.5f * (float)gFs;
  if (gFmaxHz > nyq)
    gFmaxHz = nyq;
  gAxesDirty = true;
  gHUDDirty = true;
}

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
  make_window_for_N(N_CHOICES[gNidx]);
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

void setYScale(uint8_t m)
{
  gYScale = (m ? YS_LIN : YS_DB);
  gAxesDirty = true;
  gHUDDirty = true;
}
