#include "AudioCapture.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "esp_adc_cal.h"

// Single-producer (ISR) circular buffer of centered raw samples. The consumer
// (spectrum task) only ever reads samples well behind the write pointer, so
// no locking is needed between the two cores for the buffer itself.
static int16_t capBuf[CAP_BUF_LEN];
static volatile uint32_t gCapWritePos = 0;

hw_timer_t *gTimer = nullptr;

void IRAM_ATTR onTimer()
{
  int raw = adc1_get_raw(MIC_CH); // 0..4095
  int16_t centered = (int16_t)raw - (int16_t)gDC;

  uint32_t pos = gCapWritePos;
  capBuf[pos % CAP_BUF_LEN] = centered;
  gCapWritePos = pos + 1; // single 32-bit store: atomic w.r.t. the reading core
}

void reprogram_timer(uint32_t fs)
{
  if (!gTimer)
    return;
  fs = clampi((int)fs, 2000, 40000);
  uint32_t ticks = (1000000UL + fs / 2) / fs; // 1 MHz base
  if (ticks < 1)
    ticks = 1;
  timerAlarmWrite(gTimer, ticks, true);
}

uint16_t quick_dc_estimate()
{
  uint32_t s = 0;
  for (int i = 0; i < 256; i++)
  {
    s += adc1_get_raw(MIC_CH);
    delayMicroseconds(100);
  }
  return (uint16_t)(s / 256);
}

uint32_t capture_write_pos()
{
  return gCapWritePos;
}

int16_t capture_sample_at(uint32_t absPos)
{
  return capBuf[absPos % CAP_BUF_LEN];
}

void init_audio_capture()
{
  // ADC1 config (fast path)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(MIC_CH, ADC_ATTEN_DB_12);
  gDC = quick_dc_estimate();

  // Timer @ 1 MHz tick
  gTimer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(gTimer, &onTimer, true);
  reprogram_timer(gFs);
  timerAlarmEnable(gTimer);

  gCapWritePos = 0;
}
