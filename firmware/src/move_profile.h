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
  (void)distance;
  (void)time_ms;
  return 0;
}

// Plan toward a new target `error_q8` (target - position) away. A fresh move,
// or a reversal, starts the reference at the floor in the error's direction
// with the acceleration for this distance. A move `continuing` in the same
// direction keeps its reference (no discontinuity a host could hear) and the
// larger of its acceleration and the new one, so a nearer target is still
// braked for in time.
static inline void move_profile_aim(MoveProfile& p, int32_t error_q8, uint16_t time_ms, uint16_t vel_min, bool continuing) {
  (void)p;
  (void)error_q8;
  (void)time_ms;
  (void)vel_min;
  (void)continuing;
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
  (void)p;
  (void)error_q8;
  (void)dt_us;
  (void)max_velocity;
  (void)vel_min;
  (void)deadband;
  return 0;
}

// Forget the plan when a move ends, so the next one starts at the floor.
static inline void move_profile_end(MoveProfile& p) {
  (void)p;
}
