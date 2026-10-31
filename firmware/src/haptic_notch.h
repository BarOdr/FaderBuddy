/*
 * Centre notch (HAPTIC_CENTRE_NOTCH): a light pull toward the travel's centre
 * that acts only within a window around it; the step and placed notches built
 * on the same window; and the validation of a haptic configuration word.
 *
 * Pure logic with no hardware access, so it runs in the native_test host tests.
 *
 * A single detent (HAPTIC_DETENTS, count 1) pulls to the centre from anywhere
 * on the travel: a spring, not a notch. The notch pulls only within
 * half_width counts of the centre and leaves the fader free (coasting)
 * outside, as magnet ends leave it free between their end windows. A window
 * that switched from coast to the capped drive at its edge would be a relay,
 * so the drive fades out over the window's outer HAPTIC_NOTCH_FADE counts; the
 * inner soft engage (haptic_engage.h) fades it in past the deadband. Both
 * fades share one scaling step (haptic_shape, the smaller fade wins where they
 * overlap), so the drive is zero at both ends and has no step anywhere.
 */

#pragma once

#include <stdint.h>

#include "shared/i2c_data.h"
#include "haptic_engage.h"

// The notch half-width is the config's HAPTIC_DETENT_COUNT field in units of
// 8 ADC counts (1-15: 8-120 counts, about 0.5-7 mm on a 60 mm fader).
#define HAPTIC_NOTCH_UNIT_SHIFT (3)

// ADC counts at the window's outer edge over which the drive fades out to
// zero. It shares haptic_shape()'s single scaling step with the inner soft
// engage, so it is the same band (a shift, not a division: the app has little
// flash left). To be tuned on the Entity mule's MF60T faders.
#define HAPTIC_NOTCH_FADE_SHIFT HAPTIC_SOFT_BAND_SHIFT
#define HAPTIC_NOTCH_FADE (1 << HAPTIC_NOTCH_FADE_SHIFT)

// Notch half-width in ADC counts for a haptic config word.
static inline int16_t haptic_notch_half_width(uint16_t config) {
  return (int16_t)(((config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp) << HAPTIC_NOTCH_UNIT_SHIFT);
}

// The drive within a window of `half_width` counts around the target, for
// `error` (target - position, whole ADC counts) and `drive` (signed, at most
// 254 either way): 0 at and beyond the edge, `drive` unchanged from
// HAPTIC_NOTCH_FADE counts inside the edge inward, and in between `drive`
// scaled by the counts inside the edge. The sign of `drive` is kept.
static inline int16_t haptic_window(int16_t error, int16_t half_width, int16_t drive) {
  // A deadband of minus one band: the engage fade never acts
  return haptic_shape(error, -HAPTIC_SOFT_BAND, half_width, drive);
}

// The config's HAPTIC_NOTCH_WIDTH field in ADC counts (units of 8).
static inline int16_t haptic_notch_width(uint16_t config) {
  return (int16_t)((config & HAPTIC_NOTCH_WIDTH_bm) >> (HAPTIC_NOTCH_WIDTH_bp - HAPTIC_NOTCH_UNIT_SHIFT));
}

// The window HAPTIC_STEP_NOTCHES pulls within, around the nearest step: the
// config's HAPTIC_NOTCH_WIDTH field (units of 8 counts) while the fader is
// touched, so the hand feels a notch at each step and nothing between; the
// whole travel (INT16_MAX, as HAPTIC_DETENTS) when it is not, so a released
// fader settles on the nearest step.
static inline int16_t haptic_step_notch_half_width(uint16_t config, bool touched) {
  if (!touched) return INT16_MAX;
  return haptic_notch_width(config);
}

// How long the touch bit must stay clear before HAPTIC_STEP_NOTCHES treats the
// fader as released and widens its window to the whole travel. A PTC release
// glitch with the hand resting between two steps would otherwise switch the
// drive from 0 to the cap toward the nearest step for that tick and back: a
// kick under the hand. The same 50 ms as the gate on entering INPUT_ACTIVE
// (TOUCH_OVERRIDE_DURATION_THRESHOLD in main.cpp).
#define HAPTIC_STEP_RELEASE_SETTLE_MS (50)

// Whether HAPTIC_STEP_NOTCHES keeps the touched (notch) window: while the
// touch bit is set, and for HAPTIC_STEP_RELEASE_SETTLE_MS after it drops.
// `ms_since_touch_change` is now - the time of the last touch bit edge
// (unsigned, so it survives the millis() wrap).
static inline bool haptic_step_notch_held(bool touched, uint32_t ms_since_touch_change) {
  return touched || ms_since_touch_change <= HAPTIC_STEP_RELEASE_SETTLE_MS;
}

// The window HAPTIC_PLACED_NOTCHES pulls within: the config's
// HAPTIC_NOTCH_WIDTH field while the fader is touched, and no window at all
// (0, motor_update() coasts) when it is not. Unlike the step notches there is
// no settle time and no pull after a release: a fader let go must stay where
// it is, or the value would change with no hand on it. A touch glitch drops
// the pull for that tick, which the hand does not feel as a kick.
static inline int16_t haptic_placed_notch_half_width(uint16_t config, bool touched) {
  if (!touched) return 0;
  return haptic_notch_width(config);
}

// The notch nearest `position`, of the first `count` REG_LAYER_NOTCHES
// positions, all on the 0-255 scale of REG_LAYER_TARGET (bytes, not ADC counts:
// the app has little flash left, and the windows of two notches a host keeps
// apart do not meet, so the coarser distance picks the same one). The window
// decides whether it pulls at all. The bytes are volatile as the I2C ISR
// writes them; a write landing during this loop is at worst a stale notch for
// one tick. A valid config has a count of at least 1.
static inline uint8_t haptic_placed_notch_nearest(const volatile uint8_t *positions, uint8_t count, uint8_t position) {
  uint8_t nearest = 0;
  uint8_t nearest_distance = 255;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t notch = positions[i];
    uint8_t distance = notch > position ? notch - position : position - notch;
    if (distance <= nearest_distance) {
      nearest_distance = distance;
      nearest = notch;
    }
  }
  return nearest;
}

// Whether a layer haptic config word may be stored: detents need a count of
// 1-10, the centre notch a width of 1-15, the step notches 2-10 steps and a
// width of 1-15, and the placed notches 1-4 notches and a width of 1-15; other
// modes are not checked (an unknown mode coasts).
static inline bool haptic_config_valid(uint16_t config) {
  uint8_t mode = (config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp;
  uint8_t count = (config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
  if (mode == HAPTIC_DETENTS) return count >= 1 && count <= 10;
  if (mode == HAPTIC_CENTRE_NOTCH) return count != 0;
  if (mode == HAPTIC_STEP_NOTCHES) return count >= 2 && count <= 10 && (config & HAPTIC_NOTCH_WIDTH_bm) != 0;
  if (mode == HAPTIC_PLACED_NOTCHES) {
    return count >= 1 && count <= HAPTIC_PLACED_NOTCHES_MAX && (config & HAPTIC_NOTCH_WIDTH_bm) != 0;
  }
  return true;
}
