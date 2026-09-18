#include "AudioCapture.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "esp_adc_cal.h"
#include "driver/i2s.h"

// Single-producer circular buffer of centered samples. The consumer
// (spectrum task) only ever reads samples well behind the write position,
// so no locking is needed between cores for the buffer itself.
static int16_t capBuf[CAP_BUF_LEN];
static volatile uint32_t gCapWritePos = 0;

static const i2s_port_t I2S_PORT = I2S_NUM_0;
// Frames per internal DMA buffer, and how many such buffers the driver
// cycles through. Kept modest (~a few ms of audio each) so draining doesn't
// add much capture latency on top of the analysis window itself.
static const int I2S_DMA_BUF_LEN = 256;
static const int I2S_DMA_BUF_COUNT = 4;

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

void set_sample_rate(uint32_t fs)
{
  fs = clampi((int)fs, 2000, 200000);
  i2s_set_sample_rates(I2S_PORT, fs);
}

uint32_t capture_write_pos()
{
  return gCapWritePos;
}

int16_t capture_sample_at(uint32_t absPos)
{
  return capBuf[absPos % CAP_BUF_LEN];
}

// Drains samples the I2S/DMA hardware has already pulled from the ADC into
// the circular capture buffer. Has to be a task, not an ISR, since
// i2s_read() blocks until DMA data is available. Each 16-bit word returned
// carries a 12-bit ADC reading in its low bits (the built-in-ADC-mode
// format) - mask off the rest before treating it as a sample.
static void capture_drain_task(void *pvParameters)
{
  static uint16_t raw[I2S_DMA_BUF_LEN * 2];
  for (;;)
  {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, portMAX_DELAY);
    size_t n = bytesRead / sizeof(uint16_t);
    for (size_t i = 0; i < n; i++)
    {
      int16_t centered = (int16_t)(raw[i] & 0x0FFF) - (int16_t)gDC;
      uint32_t pos = gCapWritePos;
      capBuf[pos % CAP_BUF_LEN] = centered;
      gCapWritePos = pos + 1; // single 32-bit store: atomic w.r.t. the reading core
    }
  }
}

void init_audio_capture()
{
  // ADC1 config (channel width/attenuation). I2S built-in-ADC mode still
  // uses this configuration - it just triggers conversions via DMA instead
  // of software polling.
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(MIC_CH, ADC_ATTEN_DB_12);
  gDC = quick_dc_estimate(); // direct blocking reads, before I2S claims ADC1

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = gFs,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = I2S_DMA_BUF_COUNT,
      .dma_buf_len = I2S_DMA_BUF_LEN,
      .use_apll = false,
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, nullptr);
  i2s_set_adc_mode(ADC_UNIT_1, MIC_CH);
  i2s_adc_enable(I2S_PORT);

  gCapWritePos = 0;

  xTaskCreatePinnedToCore(capture_drain_task, "capture", 4096, nullptr, 2, nullptr, 1);
}
