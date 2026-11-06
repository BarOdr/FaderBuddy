/*
 * Copyright 2026 Scott Bezek
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// The smooth motion profile for remote moves: an acceleration-limited planner
// on the velocity reference, in place of the position loop's proportional
// reference (move_vref in move_law.h, which the fast profile and the haptics
// keep). Pure functions over integers, so a host build can simulate a move
// against a model of the plant (test_move_profile); main.cpp owns the state
// and calls move_profile_aim() when a move starts or is retargeted and
// move_profile_step() once per control tick while one is under way.
//
// What it does ("Smooth motion" in ABOUT_MOTOR_CONTROL.md): the reference
// starts at the Stribeck floor, rises at a constant acceleration, and falls
// at the same rate so that it is back at the floor when the carriage reaches
// the deadband. The acceleration is 4d/T^2 for a move of d counts and a move
// time of T, which makes a velocity triangle of base T: every move long
// enough to leave the floor takes about T, whatever its distance, until the
// speed byte or the plant caps the peak. The braking decision is taken every
// tick from the measured error, not from a clock, so a retarget, a plant
// that lags the reference or a hand that slows the carriage all end at the
// same place: the floor at the deadband's edge, and the brake inside it.
//
// Scales: velocity in counts/s, kept in Q8 so that a small acceleration still
// advances it each 500 us tick; acceleration in counts/s^2, whole; error in
// Q8 counts as everywhere else. Ranges every product relies on: velocity
// within +-MOVE_PROFILE_VEL_CAP, acceleration within MOVE_PROFILE_ACCEL_MAX,
// error within +-1023 counts, vel_min within MOVE_PROFILE_VEL_CAP.

#pragma once

#include <stdint.h>

#include "move_law.h"

// The acceleration is clamped here: 2 * ACCEL_MAX * 1023 counts stays under
// 2^31 in the braking comparison. It is reached at the shortest move time
// over the longest travel, where the plant could not follow anyway.
#define MOVE_PROFILE_ACCEL_MAX (524288UL)  // counts/s^2
// The reference is clamped here: its square stays under 2^31. Twice what the
// plant can do; the speed byte, not this, is how a host limits a move.
#define MOVE_PROFILE_VEL_CAP (16000L)  // counts/s

struct MoveProfile {
  int32_t v_q8;    // planned velocity reference, counts/s in Q8, signed; 0 = none planned
  uint32_t accel;  // this move's acceleration and deceleration, counts/s^2
};

// The acceleration that moves `distance` counts in `time_ms` with a velocity
// triangle: 4 * distance / time^2, clamped to MOVE_PROFILE_ACCEL_MAX.
static inline uint32_t move_profile_accel(uint16_t distance, uint16_t time_ms) {
  if (time_ms == 0) return MOVE_PROFILE_ACCEL_MAX;
  // 4 * d * 1e6 / T^2 counts/s^2: d * 2e6 is at most 2.05e9, in two halves
  // so it never leaves 32 bits; T^2 is at most 4e6.
  uint32_t t2 = (uint32_t)time_ms * time_ms;
  uint32_t a = ((2000000UL * distance) / t2) * 2UL;
  return (a > MOVE_PROFILE_ACCEL_MAX) ? MOVE_PROFILE_ACCEL_MAX : a;
}

// Plan toward a new target `error_q8` (target - position) away. A fresh move,
// or a reversal, starts the reference at the floor in the error's direction
// with the acceleration for this distance. A move `continuing` in the same
// direction keeps its reference (no discontinuity a host could hear) and the
// larger of its acceleration and the new one, so a nearer target is still
// braked for in time.
static inline void move_profile_aim(MoveProfile& p, int32_t error_q8, uint16_t time_ms, uint16_t vel_min, bool continuing) {
  bool rising = error_q8 > 0;
  int32_t mag_q8 = rising ? error_q8 : -error_q8;
  uint32_t accel = move_profile_accel((uint16_t)(mag_q8 >> MOVE_POS_SHIFT), time_ms);
  if (continuing && p.v_q8 != 0) {
    if (accel > p.accel) p.accel = accel;
  } else {
    p.accel = accel;
    p.v_q8 = (int32_t)vel_min << MOVE_POS_SHIFT;
    if (!rising) p.v_q8 = -p.v_q8;
  }
}

// Advance the plan by one control tick of `dt_us` and return the velocity
// reference for move_drive(), signed toward the target: rising at the
// acceleration until braking is due, falling at it from then on, never under
// `vel_min` in magnitude, never over `max_velocity` when that is not 0
// (MOVE_VEL_UNLIMITED). Braking is due when the reference could not come
// down to the floor within the error left outside the `deadband` at this
// deceleration. A reference planned in the other direction (a reversal
// without a new aim, an overshoot past the deadband) restarts at the floor.
static inline int32_t move_profile_step(MoveProfile& p, int32_t error_q8, uint32_t dt_us,
                                        uint16_t max_velocity, uint16_t vel_min, int16_t deadband) {
  if (dt_us > MOVE_DT_MAX_US) dt_us = MOVE_DT_MAX_US;
  bool rising = error_q8 > 0;
  int32_t floor_q8 = (int32_t)vel_min << MOVE_POS_SHIFT;
  // A reference the wrong way, or none: start over at the floor, as
  // move_vref would hand out for this error.
  if (p.v_q8 == 0 || (p.v_q8 > 0) != rising) p.v_q8 = rising ? floor_q8 : -floor_q8;
  int32_t mag_q8 = rising ? p.v_q8 : -p.v_q8;
  int32_t mag = mag_q8 >> MOVE_POS_SHIFT;
  // The error left before the deadband, whole counts.
  int32_t remaining = ((rising ? error_q8 : -error_q8) >> MOVE_POS_SHIFT) - deadband;
  if (remaining < 0) remaining = 0;
  // Brake when the reference could not reach the floor within what is left:
  // v^2 - vmin^2 >= 2 * a * remaining (so the floor at the edge holds). v and vmin are within the cap, so their
  // squares are under 2^28; 2 * a * remaining is under 2^31 (ACCEL_MAX).
  uint32_t vmin = (vel_min > MOVE_PROFILE_VEL_CAP) ? (uint32_t)MOVE_PROFILE_VEL_CAP : vel_min;
  uint32_t stop = (uint32_t)mag * (uint32_t)mag;
  uint32_t room = vmin * vmin + 2UL * p.accel * (uint32_t)remaining;
  // The acceleration over this tick, Q8: a * dt / 1e6 * 256 = a * dt / 3906.25;
  // (a >> 4) * dt is under 2^29 and 244 is 3906.25 / 16 to 0.06%.
  int32_t step_q8 = (int32_t)(((p.accel >> 4) * dt_us) / 244UL);
  if (stop >= room) {
    mag_q8 -= step_q8;
  } else {
    mag_q8 += step_q8;
  }
  int32_t cap_q8 = (int32_t)MOVE_PROFILE_VEL_CAP << MOVE_POS_SHIFT;
  if (max_velocity != 0 && ((int32_t)max_velocity << MOVE_POS_SHIFT) < cap_q8) cap_q8 = (int32_t)max_velocity << MOVE_POS_SHIFT;
  if (mag_q8 > cap_q8) mag_q8 = cap_q8;
  if (mag_q8 < floor_q8) mag_q8 = floor_q8;
  p.v_q8 = rising ? mag_q8 : -mag_q8;
  return rising ? (mag_q8 >> MOVE_POS_SHIFT) : -(mag_q8 >> MOVE_POS_SHIFT);
}

// Forget the plan when a move ends, so the next one starts at the floor.
static inline void move_profile_end(MoveProfile& p) {
  p.v_q8 = 0;
}
