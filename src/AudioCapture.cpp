#include "AudioCapture.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "esp_adc_cal.h"

volatile int16_t *volatile isrBuf = bufA;
volatile uint16_t isrIdx = 0;
volatile uint16_t isrN = 512; // match default N at startup
volatile bool bufReady = false;
volatile int16_t *volatile readyBuf = nullptr;

volatile int16_t isrPeakAbs = 0;

hw_timer_t *gTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer()
{
  int raw = adc1_get_raw(MIC_CH); // 0..4095
  int16_t centered = (int16_t)raw - (int16_t)gDC;
  int16_t a = centered >= 0 ? centered : -centered;

  portENTER_CRITICAL_ISR(&timerMux);
  if (a > isrPeakAbs)
    isrPeakAbs = a;
  isrBuf[isrIdx++] = centered;

  if (isrIdx >= isrN)
  {
    readyBuf = isrBuf;
    bufReady = true;
    isrBuf = (isrBuf == bufA) ? bufB : bufA;
    isrIdx = 0;
    isrPeakAbs = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
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

  // Set ISR N
  portENTER_CRITICAL(&timerMux);
  isrN = N_CHOICES[gNidx];
  isrIdx = 0;
  isrBuf = bufA;
  readyBuf = nullptr;
  bufReady = false;
  isrPeakAbs = 0;
  portEXIT_CRITICAL(&timerMux);
}
