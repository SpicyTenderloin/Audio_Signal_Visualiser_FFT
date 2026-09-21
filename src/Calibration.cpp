#include "Calibration.h"
#include "Config.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "AudioCapture.h"
#include <Preferences.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const float CAL_PRESET_VOLTS[CAL_PRESET_COUNT] = {0.00f, 0.825f, 1.65f, 2.475f, 3.30f};

static const char *NVS_NAMESPACE = "adccal";
static Preferences prefs;

// ---------------- raw <-> volts, tier-agnostic at call time -------------
// Every tier reduces to a single (gain, offset) pair in calibration_init()
// (the driver's line-fitting curve is itself just gain+offset on the plain
// ESP32 this project targets, so probing it at two raw codes reconstructs it
// exactly - see calibration_init() below), so these never need to branch
// on gCalSource or touch the ADC calibration driver per call.
float adc_counts_to_volts(float rawCounts)
{
  return rawCounts * gCalGain + gCalOffset;
}

float adc_volts_to_counts(float volts)
{
  return (volts - gCalOffset) / gCalGain;
}

// ---------------- NVS persistence for the user calibration --------------
static bool load_user_cal(float &gain, float &offset)
{
  if (!prefs.begin(NVS_NAMESPACE, true))
    return false;
  bool valid = prefs.getBool("valid", false);
  if (valid)
  {
    gain = prefs.getFloat("gain", 0.0f);
    offset = prefs.getFloat("offset", 0.0f);
  }
  prefs.end();
  return valid;
}

static void save_user_cal(float gain, float offset)
{
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat("gain", gain);
  prefs.putFloat("offset", offset);
  prefs.putBool("valid", true);
  prefs.end();
}

void calibration_clear_saved()
{
  prefs.begin(NVS_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  Serial.println(F("ADC calibration: saved user calibration erased."));
  calibration_init(); // re-resolve immediately, not just on next boot
}

// ---------------- Boot-time tier resolution -----------------------------
void calibration_init()
{
  float gain, offset;
  if (load_user_cal(gain, offset))
  {
    gCalGain = gain;
    gCalOffset = offset;
    gCalSource = CAL_USER;
    Serial.printf("ADC calibration: using saved user calibration (volts = raw*%.6f + %.6f).\r\n",
                  (double)gain, (double)offset);
    return;
  }

  // The chip's factory curve, if its eFuses hold calibration data (a
  // two-point or a Vref calibration).
  adc_cali_line_fitting_efuse_val_t efuseCal = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
  const bool haveEfuse = adc_cali_scheme_line_fitting_check_efuse(&efuseCal) == ESP_OK &&
                         (efuseCal == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF ||
                          efuseCal == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP);
  adc_cali_handle_t caliHandle = nullptr;
  if (haveEfuse)
  {
    adc_cali_line_fitting_config_t caliCfg = {};
    caliCfg.unit_id = MIC_UNIT;
    caliCfg.atten = ADC_ATTEN_DB_12;
    caliCfg.bitwidth = ADC_BITWIDTH_12;
    caliCfg.default_vref = 1100;
    if (adc_cali_create_scheme_line_fitting(&caliCfg, &caliHandle) != ESP_OK)
      caliHandle = nullptr;
  }
  if (caliHandle)
  {
    // Reduce the driver's curve to a plain (gain, offset) pair by probing it
    // at the two extremes, rather than depending on its internal
    // coefficients (not stable public API across IDF versions).
    int mv0 = 0, mv4095 = 0;
    adc_cali_raw_to_voltage(caliHandle, 0, &mv0);
    adc_cali_raw_to_voltage(caliHandle, 4095, &mv4095);
    adc_cali_delete_scheme_line_fitting(caliHandle);
    const float v0 = (float)mv0 / 1000.0f;
    const float v4095 = (float)mv4095 / 1000.0f;
    gCalGain = (v4095 - v0) / 4095.0f;
    gCalOffset = v0;
    gCalSource = CAL_EFUSE;
    Serial.printf("ADC calibration: using factory eFuse curve (%s).\r\n",
                  efuseCal == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP ? "two-point" : "vref");
    return;
  }

  gCalGain = ADC_VREF / ADC_FS;
  gCalOffset = 0.0f;
  gCalSource = CAL_NONE;
  Serial.println(F("ADC calibration: no eFuse data on this chip - using nominal 3.3V/4095 math."));
}

// ---------------- Interactive multi-point calibration --------------------
static uint8_t s_presetIdx = 0;

void calibration_enter()
{
  gCalibrating = true;
  gCalPointCount = 0;
  s_presetIdx = 0;
  gCalTargetV = CAL_PRESET_VOLTS[0];
  gCalScreenDirty = true;

  Serial.println();
  Serial.println(F("=== Calibration mode ==="));
  Serial.println(F("Apply a known voltage to the mic input pin, select the matching target with"));
  Serial.println(F("ZOOM (cycles presets) or AGG (fine +/-0.01V), then press PAUSE to capture the"));
  Serial.println(F("point. Hold PAUSE ~1.2s to finish (needs >=2 points). N- undoes the last point."));
  Serial.printf("Recommended points (%d): ", CAL_PRESET_COUNT);
  for (int i = 0; i < CAL_PRESET_COUNT; i++)
    Serial.printf("%.3fV%s", (double)CAL_PRESET_VOLTS[i], i + 1 < CAL_PRESET_COUNT ? ", " : "\r\n");
}

void calibration_cycle_preset(bool up)
{
  if (up)
    s_presetIdx = (s_presetIdx + 1) % CAL_PRESET_COUNT;
  else
    s_presetIdx = (s_presetIdx + CAL_PRESET_COUNT - 1) % CAL_PRESET_COUNT;
  gCalTargetV = CAL_PRESET_VOLTS[s_presetIdx];
}

void calibration_nudge_target(bool up)
{
  float v = gCalTargetV + (up ? 0.01f : -0.01f);
  // A little slack past the rails, for a bench supply that overshoots
  // slightly at its extremes rather than hard-clamping exactly at 0/VREF.
  gCalTargetV = clampf(v, -0.5f, ADC_VREF + 0.5f);
}

void calibration_capture_point()
{
  if (gCalPointCount >= CAL_MAX_POINTS)
  {
    Serial.println(F("Calibration point table full."));
    return;
  }

  // Average recent raw samples straight out of the live capture ring
  // buffer instead of a fresh one-shot read - the continuous ADC driver has
  // claimed ADC1 for DMA since init_audio_capture(), so direct reads
  // aren't safe here (see quick_dc_estimate()'s comment in AudioCapture.cpp).
  // capture_sample_at() returns DC-centered samples, so add gDC back to
  // recover true raw codes.
  const int NSAMP = 256;
  uint32_t pos = capture_write_pos();
  int64_t sum = 0;
  for (int i = 0; i < NSAMP; i++)
    sum += (int32_t)capture_sample_at(pos - NSAMP + i) + (int32_t)gDC;
  float avgRaw = (float)sum / (float)NSAMP;

  gCalPoints[gCalPointCount].raw = avgRaw;
  gCalPoints[gCalPointCount].volts = gCalTargetV;
  gCalPointCount = gCalPointCount + 1; // not ++: incrementing a volatile is deprecated in C++20
  gCalScreenDirty = true;

  Serial.printf("Calibration point %d captured: raw=%.1f at %.3fV\r\n",
                gCalPointCount, (double)avgRaw, (double)gCalTargetV);
}

void calibration_undo_point()
{
  if (gCalPointCount == 0)
    return;
  gCalPointCount = gCalPointCount - 1;
  gCalScreenDirty = true;
}

// n==2: exact two-point solution. n>=3: ordinary least-squares linear
// regression, so noisy individual readings average out instead of the fit
// being pinned to whichever two points happen to be exact.
static void fit_line(const CalPoint *pts, int n, float &gain, float &offset)
{
  if (n == 2)
  {
    gain = (pts[1].volts - pts[0].volts) / (pts[1].raw - pts[0].raw);
    offset = pts[0].volts - gain * pts[0].raw;
    return;
  }

  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < n; i++)
  {
    sumX += pts[i].raw;
    sumY += pts[i].volts;
    sumXY += (double)pts[i].raw * pts[i].volts;
    sumXX += (double)pts[i].raw * pts[i].raw;
  }
  double denom = (double)n * sumXX - sumX * sumX;
  gain = (float)(((double)n * sumXY - sumX * sumY) / denom);
  offset = (float)((sumY - (double)gain * sumX) / n);
}

void calibration_finish()
{
  if (gCalPointCount < 2)
  {
    Serial.println(F("Need at least 2 points to calibrate - keep capturing."));
    return;
  }

  float gain, offset;
  fit_line(gCalPoints, gCalPointCount, gain, offset);
  gCalGain = gain;
  gCalOffset = offset;
  gCalSource = CAL_USER;
  save_user_cal(gain, offset);

  Serial.printf("Calibration saved from %d points: volts = raw*%.6f + %.6f\r\n",
                gCalPointCount, (double)gain, (double)offset);

  gCalibrating = false;
  gAxesDirty = true; // repaint the normal screen on the way out
}

void calibration_cancel()
{
  gCalibrating = false;
  gAxesDirty = true;
  Serial.println(F("Calibration cancelled - no changes saved."));
}
