#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Real-input FFT ----------------------
// The mic signal is real, so a full complex FFT of N points spends half its
// work on a zero imaginary part. Instead the N real samples are packed
// pairwise into N/2 complex values (x[2m] + j*x[2m+1], which in memory is
// simply the samples laid out in order), run through esp-dsp's complex FFT
// of HALF the length, and split back into the true N-point spectrum by a
// cheap per-bin post-processing step. The result matches a full complex FFT
// to within float rounding (about 0.0002 dB in testing) at roughly half the
// cost - and esp-dsp's twiddle table only has to cover FFT_MAX/2, which is
// what lets FFT_MAX be twice the size esp-dsp's precompiled 4096 cap would
// otherwise allow.

// One-time setup: esp-dsp's twiddle table (sized for FFT_MAX/2) plus the extra
// twiddles the post-processing step needs. Returns false on any allocation or
// init failure. Call after alloc_fft_buffers() and before the first FFT.
bool realfft_init();

// In-place forward FFT of N real samples stored consecutively in z (N floats).
// N must be a power of two, 32 <= N <= FFT_MAX. Afterwards z holds the
// half-length complex spectrum; read the real spectrum through
// realfft_power_prefix().
void realfft_forward(float *z, uint16_t N);

// From a buffer just processed by realfft_forward(): fills prefix[0..kmax)
// with running sums of the power |X[k]|^2 of the N-point spectrum, skipping
// DC (prefix[0] = 0, prefix[k] = prefix[k-1] + |X[k]|^2 for k >= 1). kmax must
// be <= N/2. Only bins below kmax are ever computed, so a zoomed-in view
// (Fmax well under Nyquist) pays for just the bins it actually shows.
void realfft_power_prefix(const float *z, uint16_t N, int kmax, float *prefix);
