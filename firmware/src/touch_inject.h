/*
 * Touch sources: the sensed touch (PTC) and the injected one (REG_DEBUG_TOUCH).
 *
 * Pure logic with no hardware access, so it runs in the native_test host tests.
 * main.cpp owns one TouchSources, feeds it the PTC detect/release events, host
 * writes and a tick from every loop pass, and applies the edges these return
 * through the one function that used to be the PTC callback's detect/release
 * branches. The touch the fader acts on is the OR of the two sources.
 *
 * Time is millis(); every comparison is a subtraction, so a millis() wrap
 * inside a hold does not stretch or cut it.
 */

#pragma once

#include <stdint.h>

#include "shared/i2c_data.h"

enum TouchEdge : uint8_t {
  TOUCH_EDGE_NONE = 0,     // the effective touch did not change
  TOUCH_EDGE_DETECT = 1,   // the effective touch went from released to touched
  TOUCH_EDGE_RELEASE = 2,  // the effective touch went from touched to released
};

struct TouchSources {
  bool sensed;            // last PTC event: true after DETECT, false after RELEASE
  bool injected;          // a REG_DEBUG_TOUCH hold is active
  uint32_t inject_start;  // millis() of the write that started (or restarted) the hold
  uint16_t inject_ms;     // length of the hold, 1..DEBUG_TOUCH_MAX_MS
};

static const TouchSources TOUCH_SOURCES_INIT = {false, false, 0, 0};

// The touch the fader acts on.
static inline bool touch_effective(const TouchSources &t) {
  (void)t;
  return false;
}

// The PTC reported a touch (true) or a release (false).
static inline TouchEdge touch_sensor_event(TouchSources &t, bool touched) {
  (void)t;
  (void)touched;
  return TOUCH_EDGE_NONE;
}

// A REG_DEBUG_TOUCH write, already decoded by debug_touch_ms_from_wire():
// 0 releases the hold, anything else (re)starts it at now.
static inline TouchEdge touch_inject_write(TouchSources &t, uint32_t now, uint16_t ms) {
  (void)t;
  (void)now;
  (void)ms;
  return TOUCH_EDGE_NONE;
}

// Called on every main-loop pass: ends a hold whose time is up.
static inline TouchEdge touch_inject_tick(TouchSources &t, uint32_t now) {
  (void)t;
  (void)now;
  return TOUCH_EDGE_NONE;
}

// What a REG_DEBUG_TOUCH read reports: the remaining hold in ms, 0 when none.
static inline uint16_t touch_inject_remaining(const TouchSources &t, uint32_t now) {
  (void)t;
  (void)now;
  return 0;
}

// Haptics may drive the motor only under a touch that is not injection alone:
// an injection must never start a motor.
static inline bool touch_haptics_allowed(const TouchSources &t) {
  (void)t;
  return true;
}

// STATE with its touch bits taken from the sources: STATE_TOUCH = effective
// touch, STATE_TOUCH_INJECTED = hold active. Every other bit is kept.
static inline uint32_t touch_state_bits(uint32_t state, const TouchSources &t) {
  (void)t;
  return state;
}
