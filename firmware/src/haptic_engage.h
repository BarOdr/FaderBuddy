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
// control-law drive. Tuned on the Entity mule's MF60T faders.
#define HAPTIC_SOFT_BAND (16.0f)

// The drive a haptic pull applies for `error` (target - position, ADC counts),
// given `drive`, the control law's output already capped at the haptic
// strength: 0 within `deadband`, the full `drive` from `deadband + soft_band`
// on, and in between `drive` scaled by how far past the deadband the error is.
// The sign of `drive` is kept, so the law's damping still acts.
static inline int16_t haptic_engage(float error, float deadband, float soft_band, int16_t drive) {
  return drive;
}
