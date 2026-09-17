#include "Controls.h"
#include "Config.h"
#include "Globals.h"
#include "Settings.h"

static Btn bAggDn{BTN_AGG_DOWN, true, 0}, bAggUp{BTN_AGG_UP, true, 0},
    bNDn{BTN_N_DOWN, true, 0}, bNUp{BTN_N_UP, true, 0},
    bPause{BTN_PAUSE, true, 0}, bZoomUp{BTN_ZOOM_UP, true, 0}, bZoomDn{BTN_ZOOM_DN, true, 0};

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
  if (pressedEdge(bPause))
    gPaused = !gPaused;

  // Aggregation
  if (pressedEdge(bAggDn))
    setAgg(gAgg > 1 ? gAgg - 1 : 1);
  if (pressedEdge(bAggUp))
    setAgg(gAgg + 1);

  // N
  if (pressedEdge(bNDn))
    setNidx(gNidx > 0 ? gNidx - 1 : 0);
  if (pressedEdge(bNUp))
    setNidx(gNidx + 1);

  // Horizontal zoom via Fmax (±5%)
  if (pressedEdge(bZoomDn))
    scaleFmax(false, 5.0f); // zoom out
  if (pressedEdge(bZoomUp))
    scaleFmax(true, 5.0f); // zoom in
}
