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
// MOVE_PROFILE_ACCEL_MAX, error within +-1023 counts, vel_min and the speed
// byte's max_velocity within the cap (main.cpp's apply_move_speed cannot
// reach a quarter of it; a static_assert there keeps that so).

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
  // T under 2 ms makes T^2 / 4 zero: the clamp, not a division by zero (the
  // wire never sends under MOVE_PROFILE_TIME_MIN_MS; this is for any caller).
  if (time_ms < 2) return (uint16_t)(MOVE_PROFILE_ACCEL_MAX >> 4);
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

// The error left before the deadband from `err` counts of it, never negative.
static inline uint16_t move_profile_remaining(uint16_t err, int16_t deadband) {
  int16_t remaining = (int16_t)err - deadband;
  return remaining < 0 ? 0 : (uint16_t)remaining;
}

// Put the plan's reference on its braking curve for the `remaining` counts
// when it is above it: the speed from which the plan's deceleration reaches
// the floor at the deadband's edge, v^2 = fl^2 + 2 * a * remaining, in units
// of 64 counts/s so that the root is 16-bit (the ATtiny's flash is nearly
// full): (fl/64)^2 + a16 * remaining / 128, the product one 16 x 16 multiply
// under 2^26. A curve past 16 bits (over 16384 counts/s) is above the cap:
// nothing to do. The root is Newton's from the reference itself, which is
// above it: it lands on the curve or within one unit over, from where the
// step brakes at its rate.
static inline void move_profile_cap(MoveProfile& p, uint16_t remaining, uint8_t fl64) {
  uint32_t room = (uint32_t)fl64 * fl64 + (((uint32_t)p.accel16 * remaining) >> 7);
  if (room >> 16) return;
  uint16_t room16 = (uint16_t)room;
  uint8_t mag = (uint8_t)(p.v_q2 >> 8);
  if ((uint16_t)mag * mag <= room16) return;
  for (;;) {
    uint8_t next = (uint8_t)((mag + room16 / mag) >> 1);
    if (next >= mag) break;
    mag = next;
  }
  p.v_q2 = (uint16_t)mag << 8;  // the step lifts it to the floor if under
}

// Plan toward a new target `error_q8` (target - position) away. A fresh move,
// or a reversal, starts the reference at the floor in the error's direction
// with the acceleration for this distance. A move `continuing` in the same
// direction keeps its reference (no discontinuity a host could hear) and the
// larger of its acceleration and the new one, so a nearer target is braked
// for in time. A nearer target set so late that even that cannot brake from
// the speed the plan has (the reference is above the braking curve for what
// is left: the speed from which the deceleration reaches the floor at the
// `deadband`'s edge) is put on the curve here, the one discontinuity such a
// retarget costs, so move_profile_step() never has to brake faster than the
// rate. The curve is worked out in 64 counts/s, the cap within one such unit
// under it, in 16-bit arithmetic (the ATtiny's flash is nearly full): v/64 is
// at most 250, the curve's square in those units at most 62500.
static inline void move_profile_aim(MoveProfile& p, int32_t error_q8, uint16_t time_ms, uint16_t vel_min, int16_t deadband,
                                    bool continuing) {
  bool rising = error_q8 > 0;
  int32_t mag_q8 = rising ? error_q8 : -error_q8;
  uint16_t mag = (uint16_t)(mag_q8 >> MOVE_POS_SHIFT);
  uint16_t accel16 = move_profile_accel16(mag, time_ms);
  uint16_t floor_q2 = move_profile_floor_q2(vel_min);
  if (continuing && p.v_q2 != 0 && p.rising == rising) {
    if (accel16 > p.accel16) p.accel16 = accel16;
    move_profile_cap(p, move_profile_remaining(mag, deadband), (uint8_t)(floor_q2 >> 8));
  } else {
    p.accel16 = accel16;
    p.v_q2 = floor_q2;
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
// (MOVE_VEL_UNLIMITED; within MOVE_PROFILE_VEL_CAP). Braking is due when the reference could not come
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
  int32_t err_q8 = rising ? error_q8 : -error_q8;
  uint16_t remaining = move_profile_remaining((uint16_t)(err_q8 >> MOVE_POS_SHIFT), deadband);
  // Brake when the reference could not reach the floor within what is left:
  // v^2 - vmin^2 >= 2 * a * remaining (so the floor at the edge holds). v and
  // vmin are within the cap, so their squares are under 2^28; a in
  // sixteenths times the error left, times 32, is under 2^31.
  uint16_t mag = v >> 2;
  uint16_t fl = floor_q2 >> 2;
  uint32_t stop = (uint32_t)mag * mag;
  uint32_t room = (uint32_t)fl * fl + (((uint32_t)p.accel16 * remaining) << 5);
  if (stop >= room) {
    v = (v > (uint16_t)(floor_q2 + p.step_q2)) ? (uint16_t)(v - p.step_q2) : floor_q2;
  } else {
    v = (uint16_t)(v + p.step_q2);  // at most the cap plus one step: fits
  }
  uint16_t cap_q2 = (max_velocity != 0) ? (uint16_t)(max_velocity << 2) : (uint16_t)(MOVE_PROFILE_VEL_CAP << 2);
  if (v > cap_q2) v = cap_q2;
  if (v < floor_q2) v = floor_q2;
  p.v_q2 = v;
  p.rising = rising;
  int32_t out = v >> 2;
  return rising ? out : -out;
}

// Forget the plan when a move ends, so the next one starts at the floor.
static inline void move_profile_end(MoveProfile& p) {
  p.v_q2 = 0;
}
