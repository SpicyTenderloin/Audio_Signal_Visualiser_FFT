#pragma once

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include "Config.h"

// -------------------- Display -----------------------
extern Adafruit_ILI9341 tft;

// -------------------- Settings -----------------------
extern volatile uint32_t gFs;  // Hz, sample rate (serial-adjustable)
extern volatile uint16_t gFPS;

extern const uint16_t N_CHOICES[7];
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

extern bool gUseHann;
extern bool gPaused;

// -------------------- FFT / sample buffers -----------
extern float fft_buf[2 * FFT_MAX];
extern float window_buf[FFT_MAX];

// Ping-pong raw sample buffers (int16 centered)
extern int16_t bufA[FFT_MAX];
extern int16_t bufB[FFT_MAX];

// -------------------- ADC/DC ----------------------
extern volatile uint16_t gDC;

// -------------------- Dirty flags -----------------
extern bool gAxesDirty;
extern bool gHUDDirty;

// --- Fast draw (power + prefix sums) ---
extern float gPow[FFT_MAX / 2];       // power per bin (k=0..Kny-1), skip 0 for display
extern float gPrefixPow[FFT_MAX / 2]; // prefix sums for O(1) bin-range averages
extern float gRefPow;                 // full-scale power for dBFS reference
