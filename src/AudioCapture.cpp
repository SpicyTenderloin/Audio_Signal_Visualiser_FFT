#include "AudioCapture.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "driver/i2s.h"
#include <new>

// Single-producer circular buffer of centered samples. The consumer
// (spectrum task) only ever reads samples well behind the write position,
// so no locking is needed between cores for the buffer itself.
// Heap-allocated (not a fixed-size static array), same reasoning as
// fft_buf/window_buf/gPrefixPow in Globals.cpp - see alloc_fft_buffers().
static int16_t *capBuf = nullptr;
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
  fs = clampi((int)fs, FS_MIN_HZ, FS_MAX_HZ);
  i2s_set_sample_rates(I2S_PORT, fs);
}

// Raw-word diagnostic (serial "rawdump"): a snapshot of the start of one
// i2s_read(), taken by the drain task BEFORE its keep-one-of-two decimation
// so the raw word pairs can be inspected. Handshake is two flags - the
// console sets Wanted, the drain task fills the buffer and sets Ready.
static const size_t RAW_DUMP_WORDS = 64; // even: it's printed as pairs
static uint16_t s_rawDump[RAW_DUMP_WORDS];
static volatile bool s_rawDumpWanted = false;
static volatile bool s_rawDumpReady = false;

// Prints one snapshot of raw I2S words as pairs, with a summary that shows
// whether the second word of each pair repeats the first (identical values,
// so the ADC really runs at the configured rate) or is a separate
// conversion (values differ by about as much as neighbouring pairs do).
// Best read with a steady tone playing, so real conversions differ visibly.
void capture_print_raw_dump()
{
  s_rawDumpReady = false;
  s_rawDumpWanted = true;
  const uint32_t t0 = millis();
  while (!s_rawDumpReady && millis() - t0 < 500)
    delay(1);
  if (!s_rawDumpReady)
  {
    s_rawDumpWanted = false;
    Serial.println(F("rawdump: timed out waiting for capture data."));
    return;
  }
  __sync_synchronize();

  Serial.println(F("Raw I2S words, one pair per line: full word in hex, 12-bit ADC value in decimal."));
  const int pairs = (int)(RAW_DUMP_WORDS / 2);
  int identical = 0;
  long pairDiff = 0, stepDiff = 0;
  for (int p = 0; p < pairs; p++)
  {
    const uint16_t a = s_rawDump[2 * p], b = s_rawDump[2 * p + 1];
    const int va = a & 0x0FFF, vb = b & 0x0FFF;
    Serial.printf("  %2d: 0x%04X (%4d)   0x%04X (%4d)   diff=%+d\r\n", p, a, va, b, vb, vb - va);
    if (va == vb)
      identical++;
    pairDiff += abs(vb - va);
    if (p + 1 < pairs)
      stepDiff += abs((int)(s_rawDump[2 * p + 2] & 0x0FFF) - vb);
  }
  Serial.printf("Pairs with identical ADC value: %d of %d\r\n", identical, pairs);
  Serial.printf("Mean |difference| within a pair: %.2f   from a pair to the next: %.2f\r\n",
                (double)pairDiff / pairs, (double)stepDiff / (pairs - 1));
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
//
// The stream carries TWO words per sample period at the configured rate
// (stereo I2S framing): a 1kHz test tone read as 500Hz when every word was
// treated as its own sample, and the frame loop ran ~19% faster than its
// sample-availability gate allows. The second word of each pair is an exact
// repeat of the first (serial "rawdump": 32 of 32 pairs identical, with and
// without a tone), so the ADC really converts at gFs and no conversions are
// wasted - keeping one word per pair makes gCapWritePos count samples at
// gFs, which is what the hop gate, the FFT bin frequencies and the waveform
// time axis all assume.
static void capture_drain_task(void *pvParameters)
{
  static uint16_t raw[I2S_DMA_BUF_LEN * 2];
  uint32_t wordIdx = 0; // running across reads so the keep-one-of-two pattern never slips
  for (;;)
  {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0)
    {
      // Guard against ever busy-spinning this task if a read fails or
      // returns instantly instead of blocking - at this task's priority,
      // a tight spin here would starve everything else on its core.
      vTaskDelay(1);
      continue;
    }

    size_t n = bytesRead / sizeof(uint16_t);

    if (s_rawDumpWanted && n >= RAW_DUMP_WORDS)
    {
      memcpy(s_rawDump, raw, sizeof(s_rawDump));
      __sync_synchronize(); // buffer contents visible before the flag is
      s_rawDumpWanted = false;
      s_rawDumpReady = true;
    }

    for (size_t i = 0; i < n; i++)
    {
      if (wordIdx++ & 1)
        continue; // second word of the pair - see the note above
      int16_t centered = (int16_t)(raw[i] & 0x0FFF) - (int16_t)gDC;
      uint32_t pos = gCapWritePos;
      capBuf[pos % CAP_BUF_LEN] = centered;
      gCapWritePos = pos + 1; // single 32-bit store: atomic w.r.t. the reading core
    }
  }
}

void init_audio_capture()
{
  capBuf = new (std::nothrow) int16_t[CAP_BUF_LEN];
  check_alloc(capBuf, "capBuf", CAP_BUF_LEN * sizeof(int16_t));

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

  // Pinned to core 0 (with spectrum_task), not core 1 - loop() there handles
  // buttons/serial, and this task must never be able to contend with it: if
  // i2s_read() ever misbehaves (fails, or returns without truly blocking),
  // a task at this priority spinning on core 1 would completely lock out
  // button/serial polling. Core 0 has the same risk in principle, but
  // starves the display instead of input handling, and the guard above
  // caps how bad that can get either way.
  xTaskCreatePinnedToCore(capture_drain_task, "capture", 4096, nullptr, 2, nullptr, 0);
}
