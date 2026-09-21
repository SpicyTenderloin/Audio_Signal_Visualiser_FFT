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

// This mic input has no analog anti-aliasing filter, so Fs can't just chase
// the bare minimum for resolution: without headroom, out-of-band noise
// folds straight into the displayed range as it's pushed toward Nyquist.
// Requiring Nyquist to sit at least this many times above fmax keeps a
// real margin before that happens.
//
// Because N only comes in powers of two, the margin actually achieved
// jumps in big discrete steps rather than scaling smoothly with this
// constant: for the default fmax=2.5kHz, N=1024 only reaches ~1.9x
// (visibly aliased - only content under ~500Hz stayed clean), N=2048
// reaches ~3.7x (tested - still noticeably worse than 4096, despite
// matching N=4096's Δf/window-duration exactly on paper; oversampling
// margin evidently affects noise/aliasing cleanliness on this hardware
// independently of matched frequency resolution), N=4096 reaches ~7.5x
// (tested - looks visibly cleaner). Tuned to land on the 4096 tier.
static const float FIDELITY_OVERSAMPLE_MARGIN = 4.0f;

FsNRecommendation recommend_fs_n(float fmaxHz)
{
  FsNRecommendation best{FS_MIN_HZ, N_CHOICES[0], 0};
  int bestScore = INT_MAX;
  int numChoices = sizeof(N_CHOICES) / sizeof(N_CHOICES[0]);

  for (int i = 0; i < numChoices; i++)
  {
    uint16_t N = N_CHOICES[i];
    // The Fs that would land the visible bin count exactly on PLOT_W for
    // this N (df == fmaxHz/PLOT_W, i.e. one bin per pixel across 0..fmax) -
    // but never below the anti-aliasing margin floor, even if that means
    // under-using the plot width at this N (reflected in a worse score,
    // so a larger N that can hit both constraints wins instead).
    float idealFsForResolution = fmaxHz * (float)N / (float)PLOT_W;
    float minFsForMargin = 2.0f * FIDELITY_OVERSAMPLE_MARGIN * fmaxHz;
    float idealFs = fmaxf(idealFsForResolution, minFsForMargin);
    uint32_t fs = (uint32_t)clampf(idealFs, (float)FS_MIN_HZ, (float)FS_MAX_HZ);
    int kvis = compute_visible_bins(N, (float)fs, fmaxHz);
    int score = abs(kvis - PLOT_W);

    // Prefer the closer match. On a tie, prefer the *smaller* N: any N
    // that reaches score 0 got there because idealFsForResolution alone
    // already cleared the margin floor - so every such N has already
    // satisfied the margin requirement on its own merits, and a larger
    // one just spends more FFT compute for headroom beyond what was
    // actually asked for.
    if (score < bestScore || (score == bestScore && N < best.N))
    {
      bestScore = score;
      best = {fs, N, kvis};
    }
  }

  return best;
}
