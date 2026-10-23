/*
 * Centre notch (HAPTIC_CENTRE_NOTCH): a light pull toward the travel's centre
 * that acts only within a window around it, and the validation of a haptic
 * configuration word.
 *
 * Pure logic with no hardware access, so it runs in the native_test host tests.
 */

#pragma once

#include <stdint.h>

#include "shared/i2c_data.h"

#define HAPTIC_NOTCH_UNIT_SHIFT (3)
#define HAPTIC_NOTCH_FADE_SHIFT (4)
#define HAPTIC_NOTCH_FADE (1 << HAPTIC_NOTCH_FADE_SHIFT)

// STUB (RED): contract only.
static inline int16_t haptic_notch_half_width(uint16_t config) {
  (void)config;
  return 0;
}

// STUB (RED): contract only.
static inline int16_t haptic_window(int16_t error, int16_t half_width, int16_t drive) {
  (void)error;
  (void)half_width;
  return drive;
}

// STUB (RED): contract only.
static inline bool haptic_config_valid(uint16_t config) {
  (void)config;
  return true;
}
