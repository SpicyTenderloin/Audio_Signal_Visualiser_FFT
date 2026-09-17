#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Hardware Timer Capture -------
// Circular capture buffer length in samples (power of two, for fast masking
// instead of modulo). Sized to 2x the largest supported FFT length so the
// consumer always has a full buffer's worth of headroom behind the ISR
// write pointer before a window it is copying could be overwritten.
#define CAP_BUF_LEN (2 * FFT_MAX)

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

// Copies the N most recently captured samples (the window ending at
// `endPos`, as returned by capture_write_pos()) into `out`.
void capture_read_window(int16_t *out, uint16_t N, uint32_t endPos);
