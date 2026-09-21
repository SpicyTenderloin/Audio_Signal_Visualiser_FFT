#pragma once

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include "Config.h"

// -------------------- Display -----------------------
extern Adafruit_ILI9341 tft;

// -------------------- Settings -----------------------
extern volatile uint32_t gFs;  // Hz, sample rate (serial-adjustable)
extern volatile uint16_t gFPS;

extern const uint16_t N_CHOICES[8];
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

// Waveform mode's Y-axis half-range, in centered ADC counts (i.e. the axis
// spans -gWaveYRange..+gWaveYRange) - the time-domain analog of gYMax_dB/
// gYMin_dB, adjustable the same way Fmax is.
extern volatile float gWaveYRange;

// -------------------- FFT / sample buffers -----------
// Heap-allocated (not fixed-size static arrays) so FFT_MAX can be sized
// well beyond what the linker's static DRAM segment alone can fit -
// allocated once by alloc_fft_buffers(), called first thing in setup().
extern float *fft_buf;    // size 2*FFT_MAX
extern float *window_buf; // size FFT_MAX
void alloc_fft_buffers();

// Halts with a clear serial message if p is null (an allocation failed),
// instead of continuing into an eventual null-pointer crash somewhere
// downstream that would be much harder to diagnose. what/bytes are just
// for the message.
void check_alloc(const void *p, const char *what, size_t bytes);

// -------------------- ADC/DC ----------------------
extern volatile uint16_t gDC;

// -------------------- Dirty flags -----------------
extern bool gAxesDirty;
extern bool gHUDDirty;

// --- Fast draw (prefix sums over per-bin power) ---
extern float *gPrefixPow; // size FFT_MAX/2; prefix sums for O(1) bin-range averages, heap-allocated (see alloc_fft_buffers())

// -------------------- Performance stats -----------
// Actual measured frame rate (frames/sec spectrum_task delivers), updated
// once/sec - distinct from gFPS, which is only the *target* cap.
extern volatile float gMeasuredFPS;
// Time spent in the two FFT calls, and total compute+draw time, for the
// most recently processed frame (microseconds). Excludes the idle wait for
// fresh samples, so this is what actually competes for the frame budget.
extern volatile uint32_t gLastFFTus;
extern volatile uint32_t gLastFrameUs;
extern float gRefPow;                 // full-scale power for dBFS reference
