#include "VUMeter.h"
#include "Config.h"

void setVU(uint8_t level)
{
  static uint8_t s_lastLevel = 0xFF; // sentinel: force the first call through
  if (level == s_lastLevel)
    return;
  s_lastLevel = level;

  const uint8_t pins[6] = {VU1, VU2, VU3, VU4, VU5, VU6};
  for (uint8_t i = 0; i < 6; i++)
    digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}

uint8_t vu_from_peakAbs(int16_t peakAbs)
{
  float norm = (float)peakAbs / 2048.0f;
  if (norm > 1.5f)
    norm = 1.5f;
  if (norm > 1.00f)
    return 6;
  if (norm > 0.50f)
    return 5;
  if (norm > 0.25f)
    return 4;
  if (norm > 0.12f)
    return 3;
  if (norm > 0.06f)
    return 2;
  if (norm > 0.03f)
    return 1;
  return 0;
}
