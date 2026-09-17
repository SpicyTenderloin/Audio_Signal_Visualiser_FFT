#pragma once

#include <Arduino.h>

// Lights VU1..VU6 up to `level` (0-6) and turns the rest off.
void setVU(uint8_t level);
// Maps a peak absolute sample value to a 0-6 VU level.
uint8_t vu_from_peakAbs(int16_t peakAbs);
