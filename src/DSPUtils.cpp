#include "DSPUtils.h"
#include "Globals.h"
#include <math.h>
#include <limits.h>
#include <stdlib.h>

void make_window_for_N(uint16_t N)
{
  const float step = 2.0f * (float)M_PI / (float)(N - 1);
  float sum = 0.0f;
  for (uint16_t i = 0; i < N; ++i)
  {
    window_buf[i] = 0.5f * (1.0f - cosf(step * i));
    sum += window_buf[i];
  }
  // Coherent gain (mean window value): a windowed full-scale sine's FFT peak
  // is attenuated by this factor vs. an unwindowed one, so the dBFS
  // reference has to be scaled by it too, or 0dBFS becomes unreachable
  // whenever the window is enabled.
  gWindowGain = sum / (float)N;
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

void format_time_label(char *out, size_t n, float ms)
{
  float a = fabsf(ms);
  if (a < 10.0f)
    snprintf(out, n, "%.2f", (double)ms);
  else if (a < 100.0f)
    snprintf(out, n, "%.1f", (double)ms);
  else
    snprintf(out, n, "%.0f", (double)ms);
}

void format_volts_label(char *out, size_t n, float v, float major)
{
  int decimals = 2;
  for (int d = 0; d <= 2; d++)
  {
    float scale = powf(10.0f, d);
    if (fabsf(roundf(major * scale) / scale - major) < 1e-4f)
    {
      decimals = d;
      break;
    }
  }
  snprintf(out, n, "%.*f", decimals, (double)v);
}

int bins_per_point(uint16_t N)
{
  int Kvis = visible_bin_count(N);
  float base = (float)Kvis / (float)PLOT_W;
  int bpp = (int)roundf(base * (float)gAgg);
  if (bpp < 1)
    bpp = 1;
  return bpp;
}

int compute_visible_bins(uint16_t N, float fs, float fmax)
{
  float df = fs / (float)N;
  int Kny = N / 2;
  float nyq = 0.5f * fs;
  float fmaxc = fminf(fmax, nyq);

  int Kvis = (int)floorf(fmaxc / df);
  if (Kvis > Kny - 1)
    Kvis = Kny - 1;
  if (Kvis < 2)
    Kvis = 2;
  return Kvis;
}

int visible_bin_count(uint16_t N)
{
  return compute_visible_bins(N, (float)gFs, gFmaxHz);
}

// The mic input has an analog low-pass ahead of the ADC, but it is not a brick
// wall: a Bode-plot sweep shows the response roughly flat to about 3kHz, then
// falling steeply - about 40dB below the 1kHz level by 6kHz, where the sweep
// stopped. Fs still can't just chase the bare minimum for resolution: the
// filter leaves real content between the displayed range and that roll-off,
// and it folds straight into the display as Nyquist is pushed toward it.
// Requiring Nyquist to sit at least this many times above fmax keeps a real
// margin before that happens. The cutoff is planned to move up to about 20kHz;
// then the analog filter no longer protects a narrower view from content between
// fmax and 20kHz, so recommend_fs_n() also never goes below MIN_ALIAS_SAFE_FS_HZ
// (2.4x the cutoff, Config.h) whatever fmax is - unless a digital low-pass is
// added before decimating, which would lift that floor.
//
// Because N only comes in powers of two, the margin actually achieved
// jumps in big discrete steps rather than scaling smoothly with this
// constant: for fmax=2.5kHz (the earlier default), N=1024 only reaches ~1.9x
// (visibly aliased - only content under ~500Hz stayed clean), N=2048
// reaches ~3.7x (tested - still noticeably worse than 4096, despite
// matching N=4096's Δf/window-duration exactly on paper; oversampling
// margin evidently affects noise/aliasing cleanliness on this hardware
// independently of matched frequency resolution), N=4096 reaches ~7.5x
// (tested - looks visibly cleaner). Tuned to land on the 4096 tier.
static const float FIDELITY_OVERSAMPLE_MARGIN = 4.0f;

float min_alias_safe_fs(float fmaxHz)
{
  return 2.0f * FIDELITY_OVERSAMPLE_MARGIN * fmaxHz;
}

// How far short of the best fit an N may fall and still be preferred for being
// smaller, in bins: about 7% of the plot width. A larger N only spends more FFT
// compute to fill a few more pixels.
static const int FIT_TOLERANCE_BINS = 20;

FsNRecommendation recommend_fs_n(float fmaxHz)
{
  const int numChoices = sizeof(N_CHOICES) / sizeof(N_CHOICES[0]);

  // The (Fs, N) that N-choice i would be run at, and how far its visible bin
  // count lands from the plot width (its score - lower is a better fit).
  auto candidate = [&](int i, int *scoreOut) -> FsNRecommendation
  {
    uint16_t N = N_CHOICES[i];
    // The Fs that would land the visible bin count exactly on PLOT_W for
    // this N (df == fmaxHz/PLOT_W, i.e. one bin per pixel across 0..fmax) -
    // but never below the anti-aliasing margin floor, nor below the floor the
    // analog filter's cutoff sets (MIN_ALIAS_SAFE_FS_HZ), even if that means
    // under-using the plot width at this N (reflected in a worse score, so a
    // larger N that can hit every constraint wins instead).
    float idealFsForResolution = fmaxHz * (float)N / (float)PLOT_W;
    float minFsForMargin = 2.0f * FIDELITY_OVERSAMPLE_MARGIN * fmaxHz;
    float idealFs = fmaxf(fmaxf(idealFsForResolution, minFsForMargin), (float)MIN_ALIAS_SAFE_FS_HZ);
    uint32_t fs = (uint32_t)clampf(idealFs, (float)FS_MIN_HZ, (float)FS_MAX_HZ);
    int kvis = compute_visible_bins(N, (float)fs, fmaxHz);
    *scoreOut = abs(kvis - PLOT_W);
    return {fs, N, kvis};
  };

  int bestScore = INT_MAX;
  for (int i = 0; i < numChoices; i++)
  {
    int score;
    candidate(i, &score);
    if (score < bestScore)
      bestScore = score;
  }

  // Take the smallest N that fits nearly as well as the best one: any larger N
  // just spends more FFT compute for headroom beyond what was actually asked for.
  for (int i = 0; i < numChoices; i++)
  {
    int score;
    FsNRecommendation c = candidate(i, &score);
    if (score <= bestScore + FIT_TOLERANCE_BINS)
      return c;
  }

  int unused;
  return candidate(0, &unused); // not reached: the best-scoring N always qualifies
}
