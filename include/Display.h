#pragma once

#include <Arduino.h>

// -------------------- UI / Axes -------------------
// Redraws the title, plot box, and X/Y axis ticks and labels.
void draw_axes(uint16_t N);

// -------------------- HUD (bottom status band) ---------------
// Redraws the static left part of the HUD (Fs, N, Df, Hann on/off), plus
// the FPS box (see draw_hud_fps()) so it stays in sync.
void draw_hud(uint16_t N, float df_eff);
// Redraws just the FPS digits on the right of the HUD, without touching
// (or flickering) the rest of it. Safe to call often - e.g. once/sec - since
// it skips the actual redraw when the displayed value hasn't changed.
// Pass force=true after anything that may have wiped those pixels some
// other way (e.g. a full-screen draw_axes()), to repaint unconditionally.
void draw_hud_fps(bool force = false);

// -------------------- Line rendering (O(width)) ---
// Draws one frame of the spectrum line using the precomputed power/prefix-sum buffers.
void draw_line_spectrum(uint16_t N);
