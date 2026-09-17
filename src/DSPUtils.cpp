#include "DSPUtils.h"
#include "Globals.h"
#include <math.h>

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

int bins_per_point(uint16_t N)
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
