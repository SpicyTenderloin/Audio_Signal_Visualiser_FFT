#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Numeric helpers ---------------------
inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

// -------------------- Magnitude scaling -----------
inline float fft_mag_fullscale(uint16_t N) { return (N / 2.0f) * ADC_FS; }
inline float to_dBFS(float mag, uint16_t N)
{
  float ref = fft_mag_fullscale(N);
  if (mag <= 1e-9f)
    return -120.0f;
  return 20.0f * log10f(mag / ref);
}

// -------------------- Window function ---------------------
// Fills window_buf[0..N) with a Hann window for the given FFT length.
void make_window_for_N(uint16_t N);

// -------------------- Axis step / label helpers ------------
// Rounds `rough` to the nearest "nice" 1/2/5 x 10^n step, for axis ticks.
float nice_step_125(float rough);
// Formats a frequency in Hz as "123" or "1.2k" style text for axis labels.
void format_freq_label(char *out, size_t n, float fHz);

// -------------------- Plot bin aggregation ------------------
// Number of FFT bins averaged together per horizontal pixel, given N.
int bins_per_point(uint16_t N);
