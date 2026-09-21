#include "Controls.h"
#include "Config.h"
#include "Globals.h"
#include "Settings.h"
#include "Calibration.h"

static Btn bAggDn{BTN_AGG_DOWN, true, 0}, bAggUp{BTN_AGG_UP, true, 0},
    bNDn{BTN_N_DOWN, true, 0}, bNUp{BTN_N_UP, true, 0},
    bPause{BTN_PAUSE, true, 0}, bZoomUp{BTN_ZOOM_UP, true, 0}, bZoomDn{BTN_ZOOM_DN, true, 0};

// PAUSE's dual behavior in calibration mode (short press = capture point,
// held = finish) needs how-long-has-it-been-down tracking, which
// pressedEdge()'s debounce alone doesn't give - it only reports the press
// edge. Independent of Btn/pressedEdge() since this only matters here.
static uint32_t s_pauseDownAt = 0;
static bool s_pauseLongFired = false;
static const uint32_t CAL_LONG_PRESS_MS = 1200;

bool pressedEdge(Btn &b)
{
  bool nowPressed = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (nowPressed != b.last && (now - b.t) >= DEBOUNCE_MS)
  {
    b.last = nowPressed;
    b.t = now;
    return nowPressed; // true only on press edge
  }
  return false;
}

void init_controls()
{
  pinMode(BTN_AGG_DOWN, INPUT_PULLUP);
  pinMode(BTN_AGG_UP, INPUT_PULLUP);
  pinMode(BTN_N_DOWN, INPUT_PULLUP);
  pinMode(BTN_N_UP, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_ZOOM_UP, INPUT_PULLUP);
  pinMode(BTN_ZOOM_DN, INPUT_PULLUP);
}

void pollButtons()
{
  // Every button gets polled every cycle regardless of mode, so debounce
  // state (and edge detection) stays correct even for buttons that are
  // inert in the current mode - only which *action* runs off each edge
  // depends on gDisplayMode.
  bool pause = pressedEdge(bPause);
  bool aggDn = pressedEdge(bAggDn);
  bool aggUp = pressedEdge(bAggUp);
  bool nDn = pressedEdge(bNDn);
  bool nUp = pressedEdge(bNUp);
  bool zoomDn = pressedEdge(bZoomDn);
  bool zoomUp = pressedEdge(bZoomUp);

  if (gCalibrating)
  {
    // ZOOM cycles the recommended presets; AGG fine-adjusts +/-0.01V for a
    // custom target not on that list; N- undoes the last captured point.
    if (zoomUp)
      calibration_cycle_preset(true);
    if (zoomDn)
      calibration_cycle_preset(false);
    if (aggUp)
      calibration_nudge_target(true);
    if (aggDn)
      calibration_nudge_target(false);
    if (nDn)
      calibration_undo_point();

    // PAUSE: short press captures a point at the current target voltage;
    // held past CAL_LONG_PRESS_MS finishes calibration - fires once on
    // crossing the threshold, not every poll while still held down.
    bool pauseHeld = (digitalRead(BTN_PAUSE) == LOW);
    if (pause)
    {
      s_pauseDownAt = millis();
      s_pauseLongFired = false;
    }
    if (pauseHeld && s_pauseDownAt != 0 && !s_pauseLongFired &&
        (millis() - s_pauseDownAt) >= CAL_LONG_PRESS_MS)
    {
      s_pauseLongFired = true;
      calibration_finish();
    }
    if (!pauseHeld && s_pauseDownAt != 0)
    {
      if (!s_pauseLongFired)
        calibration_capture_point();
      s_pauseDownAt = 0;
    }
    return;
  }

  // Mode switch - works the same regardless of which mode is active.
  if (pause)
    toggleDisplayMode();

  if (gDisplayMode == MODE_FFT)
  {
    // Aggregation
    if (aggDn)
      setAgg(gAgg > 1 ? gAgg - 1 : 1);
    if (aggUp)
      setAgg(gAgg + 1);

    // N
    if (nDn)
      setNidx(gNidx > 0 ? gNidx - 1 : 0);
    if (nUp)
      setNidx(gNidx + 1);

    // Horizontal zoom via Fmax (±5%)
    if (zoomDn)
      scaleFmax(false, 5.0f); // zoom out
    if (zoomUp)
      scaleFmax(true, 5.0f); // zoom in
  }
  else // MODE_WAVEFORM
  {
    // Y (amplitude) zoom. N has no meaning here - the plot always shows
    // exactly one raw sample per pixel column - so nDn/nUp are read
    // above (to keep their debounce state correct) but never acted on.
    if (aggDn)
      scaleWaveYRange(false, 5.0f); // zoom out
    if (aggUp)
      scaleWaveYRange(true, 5.0f); // zoom in

    // X (time) zoom, via Fs (±5%)
    if (zoomDn)
      scaleFs(false, 5.0f); // zoom out: more time shown, less detail
    if (zoomUp)
      scaleFs(true, 5.0f); // zoom in: less time shown, more detail
  }
}
