/*
 * Haptic engage shaping: how much of the control law's drive a haptic pull
 * applies near its target.
 *
 * Pure logic with no hardware access, so it runs in the native_test host tests.
 *
 * The control law never asks for less than breakaway (its feedforward floor),
 * so a pull that coasts inside the deadband and applies the law outside it
 * steps from 0 to about half duty at the deadband edge. A fader left alone
 * sits still inside the deadband, but a finger resting on it keeps nudging it
 * across the edge, and the motor answers each nudge with a full kick: the
 * fader trembles under the hand (Entity HIL-2026-10-11-02 F1). Past the
 * deadband the drive therefore fades in over HAPTIC_SOFT_BAND counts, so the
 * pull grows from nothing like a spring instead of switching like a relay.
 */

#pragma once

#include <stdint.h>

// ADC counts past the deadband over which a haptic pull fades in to the full
// control-law drive, as a power of two: the app has about 100 bytes of flash
// left, so the fade is a shift, not a float division. To be tuned on the
// Entity mule's MF60T faders.
#define HAPTIC_SOFT_BAND_SHIFT (4)
#define HAPTIC_SOFT_BAND (1 << HAPTIC_SOFT_BAND_SHIFT)

// The drive a haptic pull applies for `error` (target - position, whole ADC
// counts), given `drive`, the control law's output already capped at the
// haptic strength (at most 254 either way), in one scaling step for both
// fades (flash: one multiply and shift, not one per fade):
//   - 0 within `deadband`, fading in over HAPTIC_SOFT_BAND counts past it;
//   - 0 at and beyond `half_width` counts from the target, fading out over the
//     HAPTIC_SOFT_BAND counts inside that edge (the centre notch's window,
//     haptic_notch.h; INT16_MAX for a pull from anywhere).
// Where only one fade acts the result is exactly that fade; where both
// overlap (a window narrower than deadband + 2 bands) the smaller one wins,
// still zero at both ends with no step. The sign of `drive` is kept, so
// the law's damping still acts.
static inline int16_t haptic_shape(int16_t error, int16_t deadband, int16_t half_width, int16_t drive) {
  int16_t distance = error < 0 ? -error : error;
  int16_t past = distance - deadband;
  int16_t inside = half_width - distance;
  if (past <= 0 || inside <= 0) return 0;
  int16_t scale = past < inside ? past : inside;
  if (scale >= HAPTIC_SOFT_BAND) return drive;
  return (int16_t)((drive * scale) >> HAPTIC_SOFT_BAND_SHIFT);
}

// The soft engage alone: 0 within `deadband`, the full `drive` from
// `deadband + HAPTIC_SOFT_BAND` on, and in between `drive` scaled by the
// counts past the deadband.
static inline int16_t haptic_engage(int16_t error, int16_t deadband, int16_t drive) {
  return haptic_shape(error, deadband, INT16_MAX, drive);
}
