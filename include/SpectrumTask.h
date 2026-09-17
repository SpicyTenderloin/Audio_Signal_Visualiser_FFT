#pragma once

// Starts the FFT/draw task, pinned to core 0. All tft/VU access happens on
// this task, leaving core 1's Arduino loop() free to poll buttons/serial
// without waiting on FFT compute or SPI draw time. Call once from setup(),
// after init_audio_capture().
void start_spectrum_task();
