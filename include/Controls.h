#pragma once

#include <Arduino.h>

// Debounced button state: pin, last stable reading, and time of last change.
struct Btn
{
  uint8_t pin;
  bool last;
  uint32_t t;
};

// Returns true on the debounced press edge (transition to LOW) of button b.
bool pressedEdge(Btn &b);

// Sets pinModes for all control buttons; call once from setup().
void init_controls();

// Polls all control buttons and applies the corresponding setting changes.
void pollButtons();
