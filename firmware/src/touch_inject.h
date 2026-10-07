/*
 * Touch sources: the sensed touch (PTC) and the injected one (REG_DEBUG_TOUCH).
 *
 * Pure logic with no hardware access, so it runs in the native_test host tests.
 * main.cpp owns one TouchSources and feeds it the PTC detect/release events
 * and the host's writes. After each, and on every loop pass, touch_update()
 * expires the hold and returns the edge, which main.cpp applies through the
 * one function that used to be the PTC callback's detect/release branches.
 * The touch the fader acts on is the OR of the two sources.
 *
 * Time is millis(), kept in 16 bits: the ATtiny's flash is nearly full and
 * 32-bit arithmetic costs several times the code. A hold is at most
 * DEBUG_TOUCH_MAX_MS, far inside the 16-bit range, and every comparison is a
 * subtraction, so a wrap inside a hold does not stretch or cut it. A remaining
 * time outside 1..DEBUG_TOUCH_MAX_MS means the hold is over; the only way to
 * misread it is a main-loop pass longer than about 63 s, which only a hung
 * board makes.
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
  uint16_t inject_until;  // low 16 bits of millis() when the hold ends
  bool haptics_blocked;   // an injection touched the fader with no hand on it
};

static const int8_t TOUCH_SENSED_NO_EVENT = -1;  // touch_refresh(): no PTC event

static const TouchSources TOUCH_SOURCES_INIT = {false, false, 0, false};

// The touch the fader acts on.
static inline bool touch_effective(const TouchSources &t) { return t.sensed || t.injected; }

// The PTC reported a touch (true) or a release (false).
// A hand arriving lifts the haptics block; a hand leaving an injected hold sets
// it. Call with interrupts off: the receive ISR writes the injection.
static inline void touch_sensor_event(TouchSources &t, bool touched) {
  t.sensed = touched;
  if (touched) t.haptics_blocked = false;
  else if (t.injected) t.haptics_blocked = true;
}

// A REG_DEBUG_TOUCH write, already decoded by debug_touch_ms_from_wire():
// 0 releases the hold, anything else (re)starts it at now. Longer than
// DEBUG_TOUCH_MAX_MS is cut to it here too, whoever the caller.
static inline void touch_inject_write(TouchSources &t, uint32_t now, uint16_t ms) {
  if (ms > DEBUG_TOUCH_MAX_MS) ms = DEBUG_TOUCH_MAX_MS;
  t.injected = (ms != 0);
  t.inject_until = (uint16_t)((uint16_t)now + ms);
  if (t.injected && !t.sensed) t.haptics_blocked = true;
}

// What a REG_DEBUG_TOUCH read reports: the remaining hold in ms, 0 when none.
static inline uint16_t touch_inject_remaining(const TouchSources &t, uint32_t now) {
  if (!t.injected) return 0;
  uint16_t left = (uint16_t)(t.inject_until - (uint16_t)now);
  return left > DEBUG_TOUCH_MAX_MS ? 0 : left;
}

// Called after every change of a source and on every main-loop pass: ends a
// hold whose time is up (with no host refresh), then reports how the
// effective touch differs from was_touched, the touch the fader last acted on.
static inline TouchEdge touch_update(TouchSources &t, uint32_t now, bool was_touched) {
  if (touch_inject_remaining(t, now) == 0) t.injected = false;
  bool touched = touch_effective(t);
  if (touched == was_touched) return TOUCH_EDGE_NONE;
  return touched ? TOUCH_EDGE_DETECT : TOUCH_EDGE_RELEASE;
}

// Haptics may drive the motor only when no injection touched the fader without
// a hand since it last went idle: an injection must never start a motor. That
// covers the hold itself and the idle time after it, when the fader is still
// INPUT_ACTIVE with nothing touching it. A real touch lifts the block.
static inline bool touch_haptics_allowed(const TouchSources &t) { return t.sensed || !t.haptics_blocked; }

// The fader went from INPUT_ACTIVE to INPUT_IDLE: the block ends, unless a new
// hold has started meanwhile. Call with interrupts off.
static inline void touch_went_idle(TouchSources &t) {
  if (!t.injected) t.haptics_blocked = false;
}

// STATE with its touch bits taken from the sources: STATE_TOUCH = effective
// touch, STATE_TOUCH_INJECTED = hold active. Every other bit is kept.
static inline uint32_t touch_state_bits(uint32_t state, const TouchSources &t) {
  state &= ~(uint32_t)(STATE_TOUCH_bm | STATE_TOUCH_INJECTED_bm);
  if (touch_effective(t)) state |= STATE_TOUCH_bm;
  if (t.injected) state |= STATE_TOUCH_INJECTED_bm;
  return state;
}
