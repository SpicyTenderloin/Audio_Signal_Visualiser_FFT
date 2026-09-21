#pragma once

#include <Arduino.h>
#include "Config.h"

// -------------------- Continuous-ADC/DMA Capture -----------
// Sampling is driven by the ESP32's continuous ADC driver: DMA pulls
// conversions from ADC1 in the background at the configured rate, with no
// per-sample CPU/ISR cost - unlike a timer + blocking one-shot read
// approach, whose per-call driver overhead would be the practical ceiling on
// sample rate. A dedicated task drains the DMA'd samples into the circular
// capture buffer below (capture_drain_task, in AudioCapture.cpp) - it has
// to be a task rather than an ISR, since adc_continuous_read() blocks.
//
// Circular capture buffer length in samples: FFT_MAX plus 50% headroom, so
// the consumer always has margin behind the write position before a window
// it is reading could get overwritten. The drain task writes in small bursts
// (one ADC frame at a time, a few dozen samples), far smaller than that
// headroom, so the margin stays very generous. Not a power of two, so
// indexing uses modulo instead of a bitmask.
#define CAP_BUF_LEN (FFT_MAX + FFT_MAX / 2)

// Requests how many ADC conversions are averaged into each sample: 0 for
// automatic (as many as the ADC's top rate allows), otherwise k, clamped to the
// limits for the current sample rate. The drain task applies it like a rate change.
void set_averaging(uint32_t k);
// The smallest and largest averaging possible at sample rate fs: the ADC has to
// stay between its minimum rate and ADC_HW_MAX_HZ.
void capture_avg_limits(uint32_t fs, uint32_t *minK, uint32_t *maxK);

// Registers a task to be woken (by a task notification) each time the capture
// task appends new samples, so a consumer can sleep until there is something new
// instead of polling on the scheduler tick. Pass the consumer's own task handle.
void capture_set_notify_task(TaskHandle_t task);

// Requests a new sample rate (Hz) for the running capture; the drain task
// applies it within a few milliseconds.
void set_sample_rate(uint32_t fs);

// Diagnostic for the serial "rawdump" command: prints a snapshot of raw ADC
// words as consecutive pairs, with a short summary of how alike each pair is.
void capture_print_raw_dump();

// Sets up ADC1 continuous/DMA capture and starts the drain task; call once from setup().
void init_audio_capture();

// Averages a batch of raw ADC samples to estimate the DC offset.
uint16_t quick_dc_estimate();

// Total number of samples written to the capture buffer so far. Monotonic;
// wraps at 2^32, but unsigned differences against it stay valid.
uint32_t capture_write_pos();

// Returns the single captured sample at absolute position absPos (as
// returned by/derived from capture_write_pos()).
int16_t capture_sample_at(uint32_t absPos);

// A contiguous run of samples inside the ring buffer.
struct CaptureSpan
{
  const int16_t *p;
  uint16_t n;
};
// Splits the n samples starting at absolute position absPos (n < CAP_BUF_LEN)
// into at most two contiguous runs - the second is empty unless the window
// wraps past the end of the buffer - so a caller can walk them with plain
// pointer loops instead of paying a modulo per sample via capture_sample_at().
void capture_spans(uint32_t absPos, uint16_t n, CaptureSpan out[2]);
