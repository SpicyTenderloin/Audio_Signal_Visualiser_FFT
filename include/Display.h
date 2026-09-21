#pragma once

#include <Arduino.h>

// -------------------- UI / Axes -------------------
// Redraws the title, plot box, and X/Y axis ticks and labels. Branches
// internally on gDisplayMode - N is only meaningful for MODE_FFT.
void draw_axes(uint16_t N);

// -------------------- HUD (bottom status band) ---------------
// Redraws the static left part of the HUD - its field set depends on
// gDisplayMode (Fs/N/Df/Hann for FFT, Fs/Span for waveform) - plus the FPS
// box (see draw_hud_fps()) so it stays in sync.
void draw_hud();
// Redraws just the FPS digits on the right of the HUD, without touching
// (or flickering) the rest of it. Safe to call often - e.g. once/sec - since
// it skips the actual redraw when the displayed value hasn't changed.
// Pass force=true after anything that may have wiped those pixels some
// other way (e.g. a full-screen draw_axes()), to repaint unconditionally.
void draw_hud_fps(bool force = false);

// -------------------- Line rendering (O(width)) ---
// Draws one frame of the spectrum line using the precomputed power/prefix-sum buffers.
void draw_line_spectrum(uint16_t N);
// Draws one frame of the raw waveform: samples[0..PLOT_W) are one centered
// ADC sample per pixel column.
void draw_waveform(const int16_t *samples);

// -------------------- Interactive calibration screen ---------------
// Shown instead of the normal FFT/waveform display while gCalibrating is
// true (see Controls.cpp/SpectrumTask.cpp/Calibration.cpp). full=true does
// a complete redraw (title, points list, instructions); full=false just
// updates the live target-voltage/raw-ADC readout, flicker-free.
void draw_calibration_screen(bool full);
