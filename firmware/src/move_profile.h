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
// Written for the ATtiny: the reference is a 16-bit magnitude in quarter
// counts/s with its direction beside it, the acceleration is 16 bits in
// sixteenths, every product is one 16 x 16 -> 32 multiply, and 32-bit values
// are only added and compared. The step per tick is worked out once per aim
// for the nominal control tick (MOVE_PROFILE_TICK_US): a late tick advances
// the reference a little less than the time that passed, which the
// error-based braking absorbs (the ramps and the velocity estimate, which
// integrate, keep using the measured tick). Ranges every product relies on:
// velocity within MOVE_PROFILE_VEL_CAP, acceleration within
// MOVE_PROFILE_ACCEL_MAX, error within +-1023 counts, vel_min within the cap.

#pragma once

#include <stdint.h>

#include "move_law.h"

// The control tick the acceleration is integrated over.
#define MOVE_PROFILE_TICK_US (500)
// The acceleration is clamped here: in sixteenths it fits 16 bits, and
// 2 * ACCEL_MAX * 1023 counts stays under 2^32 in the braking comparison. It
// is reached at the shortest move times over long travels, where the plant
// could not follow anyway: full travel at this acceleration is 88 ms.
#define MOVE_PROFILE_ACCEL_MAX (524288UL)  // counts/s^2
// The reference is clamped here: in quarters it fits 16 bits, and its square
// stays under 2^28. Twice what the plant can do; the speed byte, not this, is
// how a host limits a move.
#define MOVE_PROFILE_VEL_CAP (16000)  // counts/s

struct MoveProfile {
  uint16_t v_q2;     // |planned velocity reference|, counts/s in Q2; 0 = none planned
  bool rising;       // its direction: toward the motor end
  uint16_t accel16;  // this move's acceleration and deceleration, counts/s^2 / 16
  uint16_t step_q2;  // the acceleration over one control tick, Q2 counts/s
};

// The acceleration, in sixteenths, that moves `distance` counts in `time_ms`
// with a velocity triangle: 4 * distance / time^2 / 16, clamped to
// MOVE_PROFILE_ACCEL_MAX / 16.
static inline uint16_t move_profile_accel16(uint16_t distance, uint16_t time_ms) {
  if (time_ms == 0) return (uint16_t)(MOVE_PROFILE_ACCEL_MAX >> 4);
  // a / 16 = 4e6 * d / T^2 / 16 = 62500 * d / (T^2 / 4): d * 62500 is at most
  // 6.4e7 and T^2 at most 4e6, both one 16 x 16 product.
  uint32_t q = ((uint32_t)time_ms * time_ms) >> 2;
  uint32_t a16 = ((uint32_t)distance * 62500U) / q;
  return (a16 > (MOVE_PROFILE_ACCEL_MAX >> 4)) ? (uint16_t)(MOVE_PROFILE_ACCEL_MAX >> 4) : (uint16_t)a16;
}

// The same in whole counts/s^2.
static inline uint32_t move_profile_accel(uint16_t distance, uint16_t time_ms) {
  return (uint32_t)move_profile_accel16(distance, time_ms) << 4;
}

// The floor of the reference, Q2, within the cap.
static inline uint16_t move_profile_floor_q2(uint16_t vel_min) {
  if (vel_min > MOVE_PROFILE_VEL_CAP) vel_min = MOVE_PROFILE_VEL_CAP;
  return (uint16_t)(vel_min << 2);
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
  uint16_t accel16 = move_profile_accel16((uint16_t)(mag_q8 >> MOVE_POS_SHIFT), time_ms);
  if (continuing && p.v_q2 != 0 && p.rising == rising) {
    if (accel16 > p.accel16) p.accel16 = accel16;
  } else {
    p.accel16 = accel16;
    p.v_q2 = move_profile_floor_q2(vel_min);
    p.rising = rising;
  }
  // a * tick in Q2 = a16 * 16 * 500e-6 * 4 = a16 * 0.032 = a16 * 4194 / 2^17
  // (to 0.01%), rounded; a16 * 4194 is under 2^28.
  p.step_q2 = (uint16_t)(((uint32_t)p.accel16 * 4194U + 65536UL) >> 17);
}

// Advance the plan by one control tick and return the velocity reference for
// move_drive(), signed toward the target: rising at the acceleration until
// braking is due, falling at it from then on, never under `vel_min` in
// magnitude, never over `max_velocity` when that is not 0
// (MOVE_VEL_UNLIMITED). Braking is due when the reference could not come
// down to the floor within the error left outside the `deadband` at this
// deceleration. A reference planned in the other direction (a reversal
// without a new aim, an overshoot past the deadband) restarts at the floor.
static inline int32_t move_profile_step(MoveProfile& p, int32_t error_q8, uint16_t max_velocity, uint16_t vel_min,
                                        int16_t deadband) {
  bool rising = error_q8 > 0;
  uint16_t floor_q2 = move_profile_floor_q2(vel_min);
  uint16_t v = p.v_q2;
  // A reference the wrong way, or none: start over at the floor, as
  // move_vref would hand out for this error.
  if (v == 0 || p.rising != rising) v = floor_q2;
  // The error left before the deadband, whole counts.
  int32_t err_q8 = rising ? error_q8 : -error_q8;
  int16_t remaining = (int16_t)(err_q8 >> MOVE_POS_SHIFT) - deadband;
  if (remaining < 0) remaining = 0;
  // Brake when the reference could not reach the floor within what is left:
  // v^2 - vmin^2 >= 2 * a * remaining (so the floor at the edge holds). v and
  // vmin are within the cap, so their squares are under 2^28; a in
  // sixteenths times the error left, times 32, is under 2^31.
  uint16_t mag = v >> 2;
  uint16_t fl = floor_q2 >> 2;
  uint32_t stop = (uint32_t)mag * mag;
  uint32_t room = (uint32_t)fl * fl + (((uint32_t)p.accel16 * (uint16_t)remaining) << 5);
  if (stop >= room) {
    v = (v > (uint16_t)(floor_q2 + p.step_q2)) ? (uint16_t)(v - p.step_q2) : floor_q2;
  } else {
    v = (uint16_t)(v + p.step_q2);  // at most the cap plus one step: fits
  }
  uint16_t cap = (max_velocity != 0 && max_velocity < MOVE_PROFILE_VEL_CAP) ? max_velocity : (uint16_t)MOVE_PROFILE_VEL_CAP;
  uint16_t cap_q2 = (uint16_t)(cap << 2);
  if (v > cap_q2) v = cap_q2;
  if (v < floor_q2) v = floor_q2;
  p.v_q2 = v;
  p.rising = rising;
  int32_t out = v >> 2;
  return rising ? out : -out;
}

// Re-plan a move under way when REG_MOVE_PROFILE changes while it runs (the
// fast profile or another move time until now): a fresh plan for what is
// left at `time_ms`, seeded with the speed the carriage has toward the target
// (`velocity`, counts/s, + toward the motor end) rather than the floor, so
// the reference carries on from where the move is instead of braking to the
// floor and ramping again.
static inline void move_profile_resume(MoveProfile& p, int32_t error_q8, int32_t velocity, uint16_t time_ms,
                                       uint16_t vel_min) {
  move_profile_aim(p, error_q8, time_ms, vel_min, false);
  (void)velocity;
}

// Forget the plan when a move ends, so the next one starts at the floor.
static inline void move_profile_end(MoveProfile& p) {
  p.v_q2 = 0;
}
