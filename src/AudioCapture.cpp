#include "AudioCapture.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include <new>
#include <string.h>

// Single-producer circular buffer of centered samples. The consumer
// (compute task) only ever reads samples well behind the write position,
// so no locking is needed between cores for the buffer itself.
// Heap-allocated (not a fixed-size static array), same reasoning as
// fft_buf/window_buf in Globals.cpp - see alloc_fft_buffers().
static int16_t *capBuf = nullptr;
static volatile uint32_t gCapWritePos = 0;

static adc_continuous_handle_t s_adc = nullptr;

// Bytes per adc_continuous_read() frame. This is the unit in which new samples
// become visible to the rest of the firmware, so it puts a ceiling on the
// display frame rate: frames per second can't exceed Fs divided by the samples
// per frame. It must be a multiple of 4 (SOC_ADC_DIGI_DATA_BYTES_PER_CONV).
// At 512 bytes (256 samples) the display was capped near 160 FPS at Fs=41kHz
// even though a frame takes under 3ms to compute; 128 bytes (64 samples) raises
// that ceiling four-fold, to Fs/64 (750 FPS at 48kHz). Smaller frames mean more
// interrupts and reads per second, which stays small even at the top sample rate.
static const uint32_t ADC_FRAME_BYTES = 128;
// Driver-side store for frames not yet read: generous, so a drain task that
// gets preempted for a while can't overflow it and drop samples.
static const uint32_t ADC_STORE_BYTES = 4096;

// By default (serial avg=auto) the ADC runs at the highest multiple of the
// requested sample rate that fits under ADC_HW_MAX_HZ (Config.h); avg=<k> picks
// k explicitly, within the limits capture_avg_limits() reports. Each group of k
// conversions is averaged into one sample. Averaging k conversions cuts random
// ADC noise by about 10*log10(k) dB, and acts as a boxcar low-pass ahead of the
// decimation - though only a weak one, so it does not replace a proper
// anti-alias filter. It costs the FFT nothing, since the FFT only ever sees the
// averaged samples at the requested rate. Following the requested rate (rather
// than locking the ADC at one speed) keeps every Fs available exactly. The
// ESP32's continuous ADC won't run below ADC_HW_MIN_HZ
// (SOC_ADC_SAMPLE_FREQ_THRES_LOW), which sets the least averaging possible at
// low sample rates.
static const uint32_t ADC_HW_MIN_HZ = 20000;

void capture_avg_limits(uint32_t fs, uint32_t *minK, uint32_t *maxK)
{
  uint32_t hi = ADC_HW_MAX_HZ / fs;
  if (hi < 1)
    hi = 1;
  uint32_t lo = (fs >= ADC_HW_MIN_HZ) ? 1 : (ADC_HW_MIN_HZ + fs - 1) / fs;
  if (lo > hi)
    lo = hi;
  *minK = lo;
  *maxK = hi;
}

// The averaging actually used for sample rate fs, given the requested value
// (0 = as much as the ADC's top rate allows), clamped to what is possible.
static uint32_t oversample_factor(uint32_t fs, uint32_t requested)
{
  uint32_t lo, hi;
  capture_avg_limits(fs, &lo, &hi);
  if (requested == 0)
    return hi;
  return requested < lo ? lo : (requested > hi ? hi : requested);
}

// The requested sample rate. set_sample_rate() only records it: the drain
// task owns the ADC handle and applies it (stop, config, start) between
// reads, so a reconfigure can never race a read in progress.
static volatile uint32_t s_requestedFs = 0;
// The requested averaging (0 = automatic). Applied the same way.
static volatile uint32_t s_requestedAvg = 0;

// The task (if any) to notify whenever new samples land in the buffer.
static TaskHandle_t volatile s_notifyTask = nullptr;

void capture_set_notify_task(TaskHandle_t task)
{
  s_notifyTask = task;
}

// Averages a batch of raw ADC samples to estimate the DC offset. Uses the
// one-shot driver, and must run before the continuous driver claims ADC1.
uint16_t quick_dc_estimate()
{
  adc_oneshot_unit_handle_t unit = nullptr;
  adc_oneshot_unit_init_cfg_t unitCfg = {};
  unitCfg.unit_id = MIC_UNIT;
  unitCfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&unitCfg, &unit) != ESP_OK)
    return 2048; // mid-scale: a sane fallback if the estimate can't be taken

  adc_oneshot_chan_cfg_t chanCfg = {};
  chanCfg.atten = ADC_ATTEN_DB_12;
  chanCfg.bitwidth = ADC_BITWIDTH_12;
  adc_oneshot_config_channel(unit, MIC_CH, &chanCfg);

  uint32_t s = 0;
  for (int i = 0; i < 256; i++)
  {
    int raw = 0;
    adc_oneshot_read(unit, MIC_CH, &raw);
    s += (uint32_t)raw;
    delayMicroseconds(100);
  }
  adc_oneshot_del_unit(unit); // release ADC1 for the continuous driver
  return (uint16_t)(s / 256);
}

void set_sample_rate(uint32_t fs)
{
  s_requestedFs = (uint32_t)clampi((int)fs, (int)FS_MIN_HZ, (int)FS_MAX_HZ);
}

void set_averaging(uint32_t k)
{
  s_requestedAvg = k;
}

// Raw-word diagnostic (serial "rawdump"): a snapshot of the start of one
// adc_continuous_read(), taken by the drain task before any averaging, so the
// raw words can be inspected as consecutive pairs. Handshake is two flags -
// the console sets Wanted, the drain task fills the buffer and sets Ready.
static const size_t RAW_DUMP_WORDS = 64; // even: it's printed as pairs
static uint16_t s_rawDump[RAW_DUMP_WORDS];
static volatile bool s_rawDumpWanted = false;
static volatile bool s_rawDumpReady = false;

// Prints one snapshot of raw ADC words as pairs, with a summary that shows
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

  Serial.println(F("Raw ADC words, one pair per line: full word in hex, 12-bit ADC value in decimal."));
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

void capture_spans(uint32_t absPos, uint16_t n, CaptureSpan out[2])
{
  const uint32_t idx = absPos % CAP_BUF_LEN;
  const uint16_t first = (idx + n <= (uint32_t)CAP_BUF_LEN) ? n : (uint16_t)(CAP_BUF_LEN - idx);
  out[0] = {capBuf + idx, first};
  out[1] = {capBuf, (uint16_t)(n - first)};
}

// Points the continuous ADC at sample rate fs (Hz) and (re)starts it.
// `running` says whether it is currently started and so must be stopped
// before it can be reconfigured. Returns k, the number of hardware
// conversions averaged into each output sample (see oversample_factor()), or 0
// if the ADC could not be started.
static uint32_t start_adc_at(uint32_t fs, uint32_t avgRequest, bool running)
{
  const uint32_t k = oversample_factor(fs, avgRequest);

  if (running)
    adc_continuous_stop(s_adc);
  // Drop frames still queued from the previous rate - they would otherwise be
  // read as if they had been taken at the new one.
  adc_continuous_flush_pool(s_adc);

  adc_digi_pattern_config_t pattern = {};
  pattern.atten = ADC_ATTEN_DB_12;
  pattern.channel = MIC_CH;
  pattern.unit = MIC_UNIT;
  pattern.bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t cfg = {};
  cfg.pattern_num = 1;
  cfg.adc_pattern = &pattern;
  cfg.sample_freq_hz = fs * k;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

  if (adc_continuous_config(s_adc, &cfg) != ESP_OK)
    return 0;
  if (adc_continuous_start(s_adc) != ESP_OK)
    return 0;
  gAdcHwHz = fs * k;
  gAdcAvg = k;
  return k;
}

// Drains samples the ADC/DMA hardware has already converted into the
// circular capture buffer. Has to be a task, not an ISR, since
// adc_continuous_read() blocks until data is available. Each 16-bit word
// returned carries a 12-bit ADC reading in its low bits (type1 format, with
// the channel number in the top nibble) - mask off the rest before treating
// it as a sample.
//
// Every word is one conversion. (This is NOT the old I2S-based capture, whose
// stream carried each conversion twice: keeping one word of every two here made
// a 1kHz test tone read as 2kHz, at two different sample rates - measured on
// the board.) So gCapWritePos counts samples at the configured rate, which is
// what the new-sample check, the FFT bin frequencies and the waveform time axis
// all assume. The serial "rawdump" command shows the raw words if that ever
// needs re-checking.
static void capture_drain_task(void *pvParameters)
{
  static uint16_t raw[ADC_FRAME_BYTES / 2];
  uint32_t appliedFs = 0;  // rate the ADC is currently set to
  uint32_t appliedAvg = 0; // and the averaging request it was set up with
  uint32_t k = 1;          // hardware conversions averaged into each sample
  bool running = false;
  uint32_t accSum = 0;     // partial average when k > 1
  uint32_t accCnt = 0;

  for (;;)
  {
    const uint32_t wantFs = s_requestedFs;
    const uint32_t wantAvg = s_requestedAvg;
    if (!running || wantFs != appliedFs || wantAvg != appliedAvg)
    {
      const uint32_t newK = start_adc_at(wantFs, wantAvg, running);
      running = (newK != 0);
      if (!running)
      {
        vTaskDelay(pdMS_TO_TICKS(100)); // couldn't start: try again shortly
        continue;
      }
      k = newK;
      appliedFs = wantFs;
      appliedAvg = wantAvg;
      accSum = 0;
      accCnt = 0;
    }

    uint32_t bytesRead = 0;
    const esp_err_t err = adc_continuous_read(s_adc, (uint8_t *)raw, sizeof(raw), &bytesRead, 50);
    if (err == ESP_ERR_TIMEOUT)
      continue; // no data yet; loop back so a rate change is still noticed
    if (err != ESP_OK || bytesRead == 0)
    {
      // Guard against ever busy-spinning this task if a read fails or
      // returns instantly instead of blocking - at this task's priority,
      // a tight spin here would starve everything else on its core.
      vTaskDelay(1);
      continue;
    }

    const size_t n = bytesRead / sizeof(uint16_t);

    if (s_rawDumpWanted && n >= RAW_DUMP_WORDS)
    {
      memcpy(s_rawDump, raw, sizeof(s_rawDump));
      __sync_synchronize(); // buffer contents visible before the flag is
      s_rawDumpWanted = false;
      s_rawDumpReady = true;
    }

    for (size_t i = 0; i < n; i++)
    {
      uint32_t v = raw[i] & 0x0FFF;
      if (k > 1)
      {
        accSum += v;
        if (++accCnt < k)
          continue;
        v = (accSum + k / 2) / k;
        accSum = 0;
        accCnt = 0;
      }
      const int16_t centered = (int16_t)v - (int16_t)gDC;
      const uint32_t pos = gCapWritePos;
      capBuf[pos % CAP_BUF_LEN] = centered;
      gCapWritePos = pos + 1; // single 32-bit store: atomic w.r.t. the reading core
    }

    // Wake the consumer now rather than letting it discover the new samples on
    // its next poll - that poll was quantized to the 1ms scheduler tick, which
    // held small-N frame rates well below what the compute allowed.
    TaskHandle_t waiter = s_notifyTask;
    if (waiter)
      xTaskNotifyGive(waiter);
  }
}

void init_audio_capture()
{
  capBuf = new (std::nothrow) int16_t[CAP_BUF_LEN];
  check_alloc(capBuf, "capBuf", CAP_BUF_LEN * sizeof(int16_t));

  gDC = quick_dc_estimate(); // one-shot reads, before the continuous driver claims ADC1

  adc_continuous_handle_cfg_t handleCfg = {};
  handleCfg.max_store_buf_size = ADC_STORE_BYTES;
  handleCfg.conv_frame_size = ADC_FRAME_BYTES;
  const esp_err_t err = adc_continuous_new_handle(&handleCfg, &s_adc);
  if (err != ESP_OK)
  {
    Serial.printf("FATAL: adc_continuous_new_handle failed (err=%d). Halting.\r\n", (int)err);
    while (true)
      delay(1000);
  }

  set_sample_rate(gFs); // the drain task applies these and starts the ADC
  set_averaging(gAvgReq);

  gCapWritePos = 0;

  // Pinned to core 0 (with the compute task), not core 1 - loop() there handles
  // buttons/serial, and this task must never be able to contend with it: if
  // adc_continuous_read() ever misbehaves (fails, or returns without truly
  // blocking), a task at this priority spinning on core 1 would completely
  // lock out button/serial polling. Core 0 has the same risk in principle,
  // but starves the display instead of input handling, and the guard above
  // caps how bad that can get either way.
  xTaskCreatePinnedToCore(capture_drain_task, "capture", 4096, nullptr, 2, nullptr, 0);
}
