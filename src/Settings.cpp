#include "Settings.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "AudioCapture.h"

void setFmax_followNyq()
{
  gFmaxFollowNyq = true;
  gFmaxHz = 0.5f * (float)fft_mode_fs();
  gAxesDirty = true;
  gHUDDirty = true;
}

void setFmax(float hz)
{
  gFmaxFollowNyq = false;
  float nyq = (float)fft_mode_fs() * 0.5f;
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
  // Fmax only exists in the spectrum mode and is limited by that mode's own
  // Fs, so a waveform-mode Fs change must not clamp it.
  if (gDisplayMode == MODE_FFT)
  {
    float nyq = 0.5f * (float)gFs;
    if (gFmaxHz > nyq)
      gFmaxHz = nyq;
  }
  gAxesDirty = true;
  gHUDDirty = true;
}

void setAvg(uint32_t k)
{
  gAvgReq = k;
  set_averaging(k);
}

// Highest signal frequency the waveform mode's zoom-out floor keeps
// alias-safe. A constant of its own rather than the spectrum mode's Fmax, so
// changing Fmax never shifts the waveform's zoom range. It equals the
// spectrum's original default Fmax (2.5kHz) and reflects today's ~3kHz analog
// filter. It does NOT yet reflect the planned 20kHz cutoff: honouring that here
// would raise the floor to Config.h's MIN_ALIAS_SAFE_FS_HZ and cap the waveform
// at about 5.7ms of time on screen, so the waveform's time base needs a
// different approach (e.g. a peak-detect mode) before the filter is changed.
static const float WAVE_ALIAS_SAFE_BANDWIDTH_HZ = 2500.0f;

void scaleFs(bool up, float pct)
{
  float factor = up ? (1.0f + pct / 100.0f) : (1.0f - pct / 100.0f);
  float newFs = (float)gFs * factor;

  // Zooming out (lower Fs = more time shown) can't be allowed to eat the
  // anti-aliasing margin - floor it at the same "Nyquist >=
  // FIDELITY_OVERSAMPLE_MARGIN x fmax" limit recommend_fs_n() uses, so the
  // minimum zoom still keeps real headroom above WAVE_ALIAS_SAFE_BANDWIDTH_HZ
  // instead of drifting into aliased territory. Zooming in (raising Fs) is never
  // a safety concern, so it's left uncapped up to setFs()'s own FS_MAX_HZ
  // ceiling.
  if (!up)
  {
    float floor = fmaxf((float)FS_MIN_HZ, min_alias_safe_fs(WAVE_ALIAS_SAFE_BANDWIDTH_HZ));
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
  // Each mode keeps its own Fs and ADC averaging. gFs/gAvgReq always hold the
  // active mode's values (everything else reads them directly), so on a switch,
  // swap them with the parked values of the mode we're switching to and
  // reprogram the capture hardware for the restored settings.
  uint32_t fs = gInactiveFs;
  gInactiveFs = gFs;
  gFs = fs;
  uint32_t avg = gInactiveAvgReq;
  gInactiveAvgReq = gAvgReq;
  gAvgReq = avg;
  set_sample_rate(gFs);
  set_averaging(gAvgReq);

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
