#pragma once

// Starts the frame pipeline: a compute task on core 0 (windowing, FFT, power
// spectrum) and a draw task on core 1 (all tft access: axes, HUD, plot and
// calibration screen), so the next frame is computed while the last is drawn.
// Also raises the calling task's priority - the Arduino loop task, which polls
// buttons and serial - above the draw task's, so input handling always preempts
// a draw. Call once from setup(), after init_audio_capture().
void start_spectrum_task();
