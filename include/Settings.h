#pragma once

#include <Arduino.h>

// -------------------- Horizontal zoom helpers ------
// Makes Fmax track Fs/2 (Nyquist) again, instead of a fixed zoom level.
void setFmax_followNyq();
// Sets Fmax (horizontal zoom) directly, clamped to [50Hz, Nyquist].
void setFmax(float hz);
// Zooms Fmax in/out by `pct` percent (true = zoom in, false = zoom out).
void scaleFmax(bool up, float pct = 5.0f);

// -------------------- Sample rate setter -----------
// Sets the ADC sample rate, reprograms the capture timer, and re-clamps Fmax.
void setFs(uint32_t fs);

// -------------------- Y range setters ---------------
// Sets the top of the Y (dBFS) axis, keeping at least a 10dB span.
void setYMax(float dB);
// Sets the bottom of the Y (dBFS) axis, keeping at least a 10dB span.
void setYMin(float dB);

// -------------------- Other setters -----------------
// Sets the display refresh rate cap, clamped to [10, 120] FPS.
void setFPS(uint16_t v);
// Selects an FFT length by index into N_CHOICES and rebuilds the window/ISR state.
void setNidx(uint8_t i);
// Sets the bin-aggregation factor (coarser/finer plot), clamped to [1, 64].
void setAgg(uint8_t v);
// Selects linear (0) or logarithmic (1) X axis scaling.
void setXScale(uint8_t m);
// Selects dBFS (0) or linear-amplitude-% (1) Y axis scaling.
void setYScale(uint8_t m);
// Turns the Hann window on/off.
void setHann(bool on);
