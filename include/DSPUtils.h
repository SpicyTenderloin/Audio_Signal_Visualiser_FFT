#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Numeric helpers ---------------------
inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

// -------------------- Window function ---------------------
// Fills window_buf[0..N) with a Hann window for the given FFT length.
void make_window_for_N(uint16_t N);

// -------------------- Axis step / label helpers ------------
// Rounds `rough` to the nearest "nice" 1/2/5 x 10^n step, for axis ticks.
float nice_step_125(float rough);
// Formats a frequency in Hz as "123" or "1.2k" style text for axis labels.
void format_freq_label(char *out, size_t n, float fHz);
// Formats a time in milliseconds as plain digits (unit-free - callers add
// "ms") with precision scaled to magnitude, for axis labels.
void format_time_label(char *out, size_t n, float ms);

// -------------------- Plot bin aggregation ------------------
// Number of FFT bins averaged together per horizontal pixel, given N.
int bins_per_point(uint16_t N);

// -------------------- Visible bin range ------------------
// Number of FFT bins visible from bin 1 up to fmax (i.e. the highest bin
// index draw_line_spectrum() will ever read), for a hypothetical fs/N -
// a pure function of its arguments, not the live gFs/gFmaxHz, so it can be
// used for "what if" queries (see recommend_fs_n()) without touching state.
int compute_visible_bins(uint16_t N, float fs, float fmax);
// Same, but for the current gFs/gFmaxHz - what SpectrumTask/Display.cpp
// actually use to bound their per-frame work to bins that get displayed.
int visible_bin_count(uint16_t N);

// -------------------- Fs/N fidelity recommendation ------------------
struct FsNRecommendation
{
  uint32_t fs;
  uint16_t N;
  int kvis; // resulting visible bin count at (fs, N, fmaxHz)
};

// Finds the (Fs, N) combination whose visible bin count (0..fmaxHz) best
// matches the plot's PLOT_W horizontal pixels - the point past which more
// bins just get averaged together for display anyway, so bins beyond it
// buy nothing, and fewer bins under-use the available width.
FsNRecommendation recommend_fs_n(float fmaxHz);
