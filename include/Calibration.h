#pragma once

#include <Arduino.h>

// -------------------- ADC calibration --------------------------------
// Three tiers, in priority order: a user-run multi-point calibration
// (saved to NVS flash, survives power cycles) beats the chip's factory
// eFuse curve (free accuracy, no extra hardware, but only
// corrects the ADC itself, not the external mic circuit) beats the naive
// ADC_VREF/ADC_FS ratio Config.h's constants imply. All three ultimately
// resolve to a single (gain, offset) pair at boot - see calibration_init()
// in Calibration.cpp - so every other caller just uses the two conversion
// functions below without caring which tier is active.

// Sets up ADC calibration for this boot: tries the saved user calibration
// first, then the chip's eFuse curve, then falls back to the naive ratio.
// Logs which tier ended up active. Call once from setup(), any time after
// init_audio_capture() (needs ADC1's channel/attenuation configured).
void calibration_init();

// Raw ADC code (0..4095, NOT DC-centered - i.e. what capture_sample_at()
// plus gDC gives back) -> volts, via whichever tier calibration_init()
// (or a just-finished interactive calibration) selected.
float adc_counts_to_volts(float rawCounts);
// Inverse of the above - volts -> raw ADC code. Used by the waveform
// Y-axis to find which raw code a "nice" round volt value falls at.
float adc_volts_to_counts(float volts);

// -------------------- Interactive multi-point calibration -------------
// Recommended calibration points, also the preset list the ZOOM buttons
// cycle through in calibration mode: 5 evenly spaced across the full
// 0..ADC_VREF range. Two (the endpoints) is the mathematical minimum for
// a line fit; the middle three let the fit average out measurement noise
// and expose real nonlinearity, if the mic circuit has any.
#define CAL_PRESET_COUNT 5
extern const float CAL_PRESET_VOLTS[CAL_PRESET_COUNT];

// Enters calibration mode: clears any in-progress points and resets the
// target voltage to the first preset. See the gCalibrating branches in
// Controls.cpp (button handling) and SpectrumTask.cpp/Display.cpp (the
// calibration screen replaces the normal FFT/waveform display while this
// is active).
void calibration_enter();
// Cycles the target voltage through CAL_PRESET_VOLTS (wrapping).
void calibration_cycle_preset(bool up);
// Fine-adjusts the target voltage by +/-0.01V, for a value off the preset
// list - this is what lets the user calibrate at their own chosen points.
void calibration_nudge_target(bool up);
// Averages ~256 recent raw samples out of the live capture buffer and
// records them against the current target voltage as a new point. No-op
// if the point table is already full.
void calibration_capture_point();
// Removes the most recently captured point, if any.
void calibration_undo_point();
// Fits a line to the captured points (exact two-point solution for
// exactly 2 points, least-squares regression for more), saves it as the
// new active user calibration (NVS + in-session state), and exits
// calibration mode. No-ops, staying in calibration mode, if fewer than 2
// points have been captured yet.
void calibration_finish();
// Discards all captured points and exits without saving anything.
void calibration_cancel();
// Erases any saved user calibration from NVS and re-resolves the active
// tier immediately (falling back to eFuse/naive for the rest of this
// session too, not just the next boot).
void calibration_clear_saved();
