#pragma once

#include <Arduino.h>

// -------------------- UI / Axes -------------------
// Redraws the title, plot box, and X/Y axis ticks and labels.
void draw_axes(uint16_t N);

// -------------------- HUD (bottom banner, centered) ---------------
// Redraws the bottom status band (Fs, N, Df, Hann on/off).
void draw_hud(uint16_t N, float df_eff);

// -------------------- Line rendering (O(width)) ---
// Draws one frame of the spectrum line using the precomputed power/prefix-sum buffers.
void draw_line_spectrum(uint16_t N);
