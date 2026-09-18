#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Hardware Timer Capture -------
// Circular capture buffer length in samples: FFT_MAX plus 50% headroom, so
// the consumer always has margin behind the ISR write pointer before a
// window it is reading could get overwritten. That headroom only has to
// outlast a few microseconds of per-sample reads, even at the fastest
// sample rate (2048 samples of headroom at FFT_MAX=4096 is >50ms at 40kHz),
// so 50% is already generous - it doesn't need FFT_MAX's old 100% margin.
// Not a power of two, so indexing uses modulo instead of a bitmask.
#define CAP_BUF_LEN (FFT_MAX + FFT_MAX / 2)

extern hw_timer_t *gTimer;

// Timer ISR: samples the mic ADC and appends it to the circular capture buffer.
void IRAM_ATTR onTimer();
// Reprograms the hardware timer alarm period for a new sample rate (Hz).
void reprogram_timer(uint32_t fs);

// Sets up ADC1 + the hardware timer capture; call once from setup().
void init_audio_capture();

// Averages a batch of raw ADC samples to estimate the DC offset.
uint16_t quick_dc_estimate();

// Total number of samples written to the capture buffer so far. Monotonic;
// wraps at 2^32, but unsigned differences against it stay valid.
uint32_t capture_write_pos();

// Returns the single captured sample at absolute position absPos (as
// returned by/derived from capture_write_pos()).
int16_t capture_sample_at(uint32_t absPos);
