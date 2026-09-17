#pragma once

#include <Arduino.h>

// -------------------- Hardware Timer Capture -------
extern volatile int16_t *volatile isrBuf;
extern volatile uint16_t isrIdx;
extern volatile uint16_t isrN;
extern volatile bool bufReady;
extern volatile int16_t *volatile readyBuf;
extern volatile int16_t isrPeakAbs;

extern hw_timer_t *gTimer;
extern portMUX_TYPE timerMux;

// Timer ISR: samples the mic ADC and fills the active ping-pong buffer.
void IRAM_ATTR onTimer();
// Reprograms the hardware timer alarm period for a new sample rate (Hz).
void reprogram_timer(uint32_t fs);

// Sets up ADC1 + hardware timer capture; call once from setup().
void init_audio_capture();

// Averages a batch of raw ADC samples to estimate the DC offset.
uint16_t quick_dc_estimate();
