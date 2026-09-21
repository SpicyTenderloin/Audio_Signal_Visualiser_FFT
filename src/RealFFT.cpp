#include "RealFFT.h"

#include <math.h>
#include <new>
#include "esp_dsp.h"
#include "dsps_fft2r.h"

// Post-processing twiddles: interleaved (cos, sin) of 2*pi*k/FFT_MAX for k in
// [0, FFT_MAX/2), in natural order. A shorter N reads every (FFT_MAX/N)-th
// entry, since 2*pi*k/N == 2*pi*(k*FFT_MAX/N)/FFT_MAX.
static float *s_tw = nullptr;

bool realfft_init()
{
  s_tw = new (std::nothrow) float[FFT_MAX];
  if (!s_tw)
    return false;
  for (int k = 0; k < FFT_MAX / 2; ++k)
  {
    const float ang = (float)(2.0 * M_PI * (double)k / (double)FFT_MAX);
    s_tw[2 * k] = cosf(ang);
    s_tw[2 * k + 1] = sinf(ang);
  }
  // esp-dsp only ever runs the half-length complex FFT.
  return dsps_fft2r_init_fc32(nullptr, FFT_MAX / 2) == ESP_OK;
}

void realfft_forward(float *z, uint16_t N)
{
  const int M = N / 2;
  dsps_fft2r_fc32(z, M);
  dsps_bit_rev_fc32(z, M);
}

// With Z = FFT_M(z) of the packed sequence z[m] = x[2m] + j*x[2m+1]
// (M = N/2), the transforms of the even and odd samples are
//   E[k] = (Z[k] + conj(Z[M-k])) / 2
//   O[k] = (Z[k] - conj(Z[M-k])) / 2j
// and the real signal's spectrum is X[k] = E[k] + exp(-j*2*pi*k/N) * O[k].
void realfft_power_prefix(const float *z, uint16_t N, int kmax, float *prefix)
{
  const int M = N / 2;
  const int twStride = 2 * (FFT_MAX / N); // floats per k step in s_tw

  prefix[0] = 0.0f;
  float acc = 0.0f;
  for (int k = 1; k < kmax; ++k)
  {
    const int j = M - k;
    const float zkr = z[2 * k], zki = z[2 * k + 1];
    const float zjr = z[2 * j], zji = z[2 * j + 1];

    const float er = 0.5f * (zkr + zjr);
    const float ei = 0.5f * (zki - zji);
    const float orr = 0.5f * (zki + zji);
    const float oi = 0.5f * (zjr - zkr);

    const float c = s_tw[k * twStride];
    const float s = s_tw[k * twStride + 1];

    const float xr = er + c * orr + s * oi;
    const float xi = ei + c * oi - s * orr;

    acc += xr * xr + xi * xi;
    prefix[k] = acc;
  }
}
