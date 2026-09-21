#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- I2S/DMA ADC Capture -----------
// Sampling is driven by the ESP32's I2S peripheral in built-in-ADC mode:
// DMA pulls conversions from ADC1 continuously in the background at the
// configured clock, with no per-sample CPU/ISR cost, unlike the old
// hardware-timer + blocking adc1_get_raw() approach (which had enough
// per-call driver overhead to make it the practical ceiling on sample
// rate). A dedicated task drains the DMA'd samples into the circular
// capture buffer below (capture_drain_task, in AudioCapture.cpp) - it has
// to be a task rather than an ISR, since i2s_read() blocks.
//
// Circular capture buffer length in samples: FFT_MAX plus 50% headroom, so
// the consumer always has margin behind the write position before a window
// it is reading could get overwritten. The drain task writes in bursts (one
// DMA chunk at a time, a few hundred samples) rather than one sample at a
// time now, but that headroom is still enormously larger than one burst,
// so 50% margin remains very generous. Not a power of two, so indexing
// uses modulo instead of a bitmask.
#define CAP_BUF_LEN (FFT_MAX + FFT_MAX / 2)

// Reconfigures the I2S sample rate (Hz) for a running capture.
void set_sample_rate(uint32_t fs);

// Diagnostic for the serial "rawdump" command: prints a snapshot of raw I2S
// words (before the drain task keeps one of each pair) with a short summary.
void capture_print_raw_dump();

// Sets up ADC1 + I2S/DMA capture and starts the drain task; call once from setup().
void init_audio_capture();

// Averages a batch of raw ADC samples to estimate the DC offset.
uint16_t quick_dc_estimate();

// Total number of samples written to the capture buffer so far. Monotonic;
// wraps at 2^32, but unsigned differences against it stay valid.
uint32_t capture_write_pos();

// Returns the single captured sample at absolute position absPos (as
// returned by/derived from capture_write_pos()).
int16_t capture_sample_at(uint32_t absPos);
