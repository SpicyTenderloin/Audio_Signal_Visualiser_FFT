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
// Requests a new ADC sample rate (see set_sample_rate()) and re-clamps Fmax.
void setFs(uint32_t fs);
// Sets how many ADC conversions are averaged into each sample in the current
// mode (0 = automatic: as many as the ADC's top rate allows).
void setAvg(uint32_t k);
// Zooms Fs in/out by `pct` percent - the waveform mode's time-axis zoom.
// The plot always shows PLOT_W raw samples, so a higher Fs means each of
// those samples spans less real time: "zoom in" (up=true) means less time
// shown, more detail; "zoom out" means more time shown, less detail.
void scaleFs(bool up, float pct = 5.0f);

// -------------------- Y range setters ---------------
// Sets the top of the Y (dBFS) axis, keeping at least a 10dB span.
void setYMax(float dB);
// Sets the bottom of the Y (dBFS) axis, keeping at least a 10dB span.
void setYMin(float dB);

// -------------------- Other setters -----------------
// Selects an FFT length by index into N_CHOICES and rebuilds the window.
void setNidx(uint8_t i);
// Sets the bin-aggregation factor (coarser/finer plot), clamped to [1, 64].
void setAgg(uint8_t v);
// Selects linear (0) or logarithmic (1) X axis scaling.
void setXScale(uint8_t m);
// Selects dBFS (0) or linear-amplitude-% (1) Y axis scaling.
void setYScale(uint8_t m);
// Turns the Hann window on/off.
void setHann(bool on);

// -------------------- Display mode -----------------
// Switches between the FFT spectrum and raw-waveform display modes.
void toggleDisplayMode();
// Zooms the waveform mode's Y (amplitude) axis in/out by `pct` percent.
void scaleWaveYRange(bool up, float pct = 5.0f);
