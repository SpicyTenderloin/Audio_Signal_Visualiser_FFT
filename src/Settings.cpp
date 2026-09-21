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
  fs = clampi((int)fs, FS_MIN_HZ, FS_MAX_HZ);
  gFs = fs;
  set_sample_rate(gFs);
  float nyq = 0.5f * (float)gFs;
  if (gFmaxHz > nyq)
    gFmaxHz = nyq;
  gAxesDirty = true;
  gHUDDirty = true;
}

void scaleFs(bool up, float pct)
{
  float factor = up ? (1.0f + pct / 100.0f) : (1.0f - pct / 100.0f);
  float newFs = (float)gFs * factor;

  // Zooming out (lower Fs = more time shown) can't be allowed to eat the
  // anti-aliasing margin the way it did before we caught that bug - floor
  // it at the same "Nyquist >= FIDELITY_OVERSAMPLE_MARGIN x fmax" limit
  // recommend_fs_n() uses, so the minimum zoom still keeps real headroom
  // above gFmaxHz instead of drifting into aliased territory. Zooming in
  // (raising Fs) is never a safety concern, so it's left uncapped up to
  // setFs()'s own FS_MAX_HZ ceiling.
  if (!up)
  {
    float floor = fmaxf((float)FS_MIN_HZ, min_alias_safe_fs(gFmaxHz));
    if (newFs < floor)
      newFs = floor;
  }

  setFs((uint32_t)newFs);
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

void setHann(bool on)
{
  gUseHann = on;
  gAxesDirty = true;
  gHUDDirty = true;
}

void toggleDisplayMode()
{
  gDisplayMode = (gDisplayMode == MODE_FFT) ? MODE_WAVEFORM : MODE_FFT;
  gAxesDirty = true; // the two modes' axes are entirely different
  gHUDDirty = true;  // ...and so is the HUD's field set
}

void scaleWaveYRange(bool up, float pct)
{
  float factor = up ? (1.0f - pct / 100.0f) : (1.0f + pct / 100.0f); // up = zoom in = smaller range
  float v = gWaveYRange * factor;
  if (v < 16.0f)
    v = 16.0f;
  if (v > 2048.0f)
    v = 2048.0f;
  gWaveYRange = v;
  gAxesDirty = true;
}
