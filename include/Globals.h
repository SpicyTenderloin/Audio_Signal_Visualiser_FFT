#pragma once

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include "Config.h"

// -------------------- Display -----------------------
extern Adafruit_ILI9341 tft;

// -------------------- Settings -----------------------
// Sample rate is per display mode, so tuning one mode never moves the
// other's. gFs always holds the ACTIVE mode's value - capture and the axes
// read it directly - while gInactiveFs parks the value of the mode that isn't
// showing. toggleDisplayMode() swaps them on every switch.
extern volatile uint32_t gFs;  // Hz, sample rate (serial-adjustable)
extern uint32_t gInactiveFs;

extern const uint16_t N_CHOICES[9];
extern volatile uint8_t gNidx;         // index into N_CHOICES
extern volatile uint8_t gAgg;          // aggregation (>=1) => bpp defaults to 1
extern volatile float gFmaxHz;         // horizontal max frequency
extern volatile bool gFmaxFollowNyq;   // Fmax acts as zoom (independent of Nyquist)

// Y-axis range (in dBFS). Top is ymax, bottom is ymin.
extern volatile float gYMax_dB;
extern volatile float gYMin_dB;

enum XScaleMode
{
  XS_LIN = 0,
  XS_LOG = 1
};
extern volatile XScaleMode gXScale;

enum YScaleMode
{
  YS_DB = 0,  // dBFS (20*log10 of amplitude ratio)
  YS_LIN = 1  // linear amplitude, as a % of full scale
};
extern volatile YScaleMode gYScale;

extern bool gUseHann;
extern bool gPaused;
extern volatile float gWindowGain; // coherent gain (mean) of window_buf for the current N; 1.0 when gUseHann is off

// -------------------- Display mode -----------------------
enum DisplayMode
{
  MODE_FFT = 0,     // frequency-domain spectrum (the default)
  MODE_WAVEFORM = 1 // time-domain raw waveform ("scope" view)
};
extern volatile DisplayMode gDisplayMode;

// The spectrum mode's Fs whichever mode is showing (gFs while in spectrum
// mode, the parked value otherwise). Spectrum-only settings like Fmax are
// limited by this, not by whatever rate the waveform mode is using.
uint32_t fft_mode_fs();

// Waveform mode's Y-axis half-range, in centered ADC counts (i.e. the axis
// spans -gWaveYRange..+gWaveYRange) - the time-domain analog of gYMax_dB/
// gYMin_dB, adjustable the same way Fmax is.
extern volatile float gWaveYRange;

// -------------------- FFT / sample buffers -----------
// Heap-allocated (not fixed-size static arrays) so FFT_MAX can be sized
// well beyond what the linker's static DRAM segment alone can fit -
// allocated once by alloc_fft_buffers(), called first thing in setup().
extern float *fft_buf;    // size FFT_MAX: N real samples, read as N/2 complex (see RealFFT.h)
extern float *window_buf; // size FFT_MAX
void alloc_fft_buffers();

// Halts with a clear serial message if p is null (an allocation failed),
// instead of continuing into an eventual null-pointer crash somewhere
// downstream that would be much harder to diagnose. what/bytes are just
// for the message.
void check_alloc(const void *p, const char *what, size_t bytes);

// -------------------- ADC/DC ----------------------
extern volatile uint16_t gDC;

// -------------------- ADC calibration ----------------------
// Which tier calibration_init() (Calibration.cpp) resolved gCalGain/
// gCalOffset from - for boot-log/diagnostic purposes only; every other
// caller just uses adc_counts_to_volts()/adc_volts_to_counts() and doesn't
// need to know which tier is behind them.
enum CalSource
{
  CAL_NONE = 0,  // naive ADC_VREF/ADC_FS ratio - no calibration data available
  CAL_EFUSE = 1, // the chip's factory eFuse curve (esp_adc_cal)
  CAL_USER = 2   // a saved interactive multi-point calibration (NVS)
};
extern volatile CalSource gCalSource;
// Active calibration coefficients: volts = rawCounts * gCalGain + gCalOffset.
// Resolved once at boot (or immediately after finishing/clearing a user
// calibration) regardless of which tier they came from, so hot paths (axis
// tick generation) never need to branch on gCalSource themselves.
extern volatile float gCalGain;
extern volatile float gCalOffset;

// True while the interactive calibration screen is active - SpectrumTask.cpp
// shows it instead of the normal FFT/waveform display, and Controls.cpp
// routes buttons to calibration actions instead of their usual ones.
extern volatile bool gCalibrating;
// Set whenever the calibration screen's point list changes (a new point
// captured, one undone) or the screen is freshly entered - tells
// draw_calibration_screen() to do a full redraw instead of just updating
// the live target-voltage/raw-ADC readout.
extern bool gCalScreenDirty;

#define CAL_MAX_POINTS 12
struct CalPoint
{
  float raw;   // averaged raw ADC code (0..4095) captured at this point
  float volts; // the true voltage the user applied when it was captured
};
extern CalPoint gCalPoints[CAL_MAX_POINTS];
extern volatile uint8_t gCalPointCount;
extern volatile float gCalTargetV; // currently selected target voltage (calibration mode)

// -------------------- Dirty flags -----------------
extern bool gAxesDirty;
extern bool gHUDDirty;

// --- Fast draw (prefix sums over per-bin power) ---
extern float *gPrefixPow; // size FFT_MAX/2; prefix sums for O(1) bin-range averages, heap-allocated (see alloc_fft_buffers())

// -------------------- Performance stats -----------
// Measured frame rate (frames/sec spectrum_task delivers), updated once/sec.
// There is no target - the loop runs as fast as compute and drawing allow.
extern volatile float gMeasuredFPS;
// Time spent in the FFT, in drawing the plot, and in the whole frame
// (compute + draw), for the most recently processed frame (microseconds).
// Excludes the idle wait for fresh samples, so this is what actually
// competes for the frame budget.
extern volatile uint32_t gLastFFTus;
extern volatile uint32_t gLastDrawUs;
extern volatile uint32_t gLastFrameUs;
extern float gRefPow;                 // full-scale power for dBFS reference
