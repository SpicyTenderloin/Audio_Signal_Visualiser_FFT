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
// Formats a voltage as plain digits (unit-free - callers add "V"), with
// decimal precision matched to the tick step `major` rather than a fixed
// count of places - a clean step like 0.5V or 2V doesn't need the trailing
// zero digits a fixed "%.2f" would force, and every extra digit costs space
// in the narrow left-axis margin.
void format_volts_label(char *out, size_t n, float v, float major);

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
// buy nothing, and fewer bins under-use the available width. Among the N that
// fit nearly as well as the best, the smallest (cheapest FFT) is chosen, and Fs
// never goes below the alias-safe floors (see DSPUtils.cpp and Config.h).
FsNRecommendation recommend_fs_n(float fmaxHz);

// The lowest Fs that keeps Nyquist at least FIDELITY_OVERSAMPLE_MARGIN
// times above fmaxHz - the same anti-aliasing margin recommend_fs_n() uses,
// exposed standalone for callers (e.g. the waveform mode's zoom-out floor)
// that just need the floor itself, not a full (Fs, N) recommendation.
float min_alias_safe_fs(float fmaxHz);
