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

// The smooth motion profile for remote moves, second design ("cruise and
// brake"): the move takes off and cruises exactly as the fast profile does
// (the reference at the cap, the velocity loop saturated, the take-up ceiling
// shaping the start), and only the end is planned: a constant-deceleration
// curve on the measured error that brings the reference from the speed the
// carriage has down to the Stribeck floor a run-in short of the deadband, in
// a brake time the host sets (REG_MOVE_PROFILE's ms, MOVE_BRAKE_TIME_DEFAULT_MS
// from reset). Pure functions over integers, so a host build can simulate a
// move against a model of the plant (test_move_brake); main.cpp owns the
// state and calls move_brake_aim() when a move starts or is retargeted and
// move_brake_step() once per control tick while one is under way.
//
// Why not the velocity triangle of the first design (move_profile.h, kept
// for reference until Bartek retires it with its tests): on the bench
// (2026-11-07, board 0) the triangle's falling reference was followed with a
// large, unit-dependent lag, because the velocity loop's feedforward
// (breakaway + v/k) keeps driving forward while kv alone has to pull the
// carriage down; the carriage arrived at the deadband at 1500-4000 counts/s,
// where the bridge brake cannot stop it inside 8 counts, and bounced. The
// fast profile is clean because its collapsing reference makes a large
// negative velocity error, so the drive goes negative and the brake is hard.
// This design keeps that take-off, decides the brake by time and distance from
// the measured speed (not from a plan the carriage never followed), follows a
// curve tied to the measured error so a lagging plant is braked harder, and
// doubles the velocity gain while braking (MOVE_BRAKE_KV_SHIFT), in the smooth
// profile only, so the arrival speed stays under what the bridge brake can
// absorb (about 1400 counts/s on board 0). The run-in (MOVE_BRAKE_RUN_IN) is
// where the carriage settles to its floor equilibrium before the deadband.
//
// Units: positions in Q8 counts (MOVE_POS_SHIFT), velocities in whole
// counts/s, the deceleration in sixteenths of counts/s^2, the braking curve
// in units of 16 counts/s so that its root is 16-bit (the floor, 560, is 35
// such units exactly). Ranges every product relies on: velocity within
// MOVE_BRAKE_VEL_CAP, deceleration within MOVE_BRAKE_ACCEL_MAX, error within
// +-1023 counts, the measured velocity within MOVE_VEL_CLAMP, the brake
// time within MOVE_PROFILE_TIME_MAX_MS.

#pragma once

#include <stdint.h>

#include "move_law.h"

// The brake time from reset, ms: how long the reference takes from the speed
// the carriage has to the floor. Full travel on the bench is about 130 ms of
// take-off and cruise at 8000 counts/s, so 120 ms of brake lands full travel
// near the Automatone's 0.25 s. REG_MOVE_PROFILE sets it at runtime.
#define MOVE_BRAKE_TIME_DEFAULT_MS (120)
// Counts before the deadband's edge at which the curve reaches the floor,
// so the carriage settles on its floor equilibrium before the bridge brake.
#define MOVE_BRAKE_RUN_IN (24)
// The velocity gain is shifted up by this while braking (smooth moves only).
#define MOVE_BRAKE_KV_SHIFT (1)
// The largest velocity gain move_drive() keeps in 32 bits (move_law.h).
#define MOVE_BRAKE_KV_MAX_Q16 (16383)
// The deceleration is clamped here: in sixteenths it fits 16 bits, and the
// curve's product stays under 2^32.
#define MOVE_BRAKE_ACCEL_MAX (524288UL)  // counts/s^2
// The reference is clamped here: twice what the plant can do, so a cruising
// reference saturates the velocity loop as the fast profile's does; the speed
// byte, not this, is how a host limits a move.
#define MOVE_BRAKE_VEL_CAP (16000)  // counts/s

struct MoveBrake {
  bool braking;      // on the braking curve; else cruising at the cap
  bool rising;       // the planned direction: toward the motor end
  uint16_t accel16;  // this brake's deceleration, counts/s^2 / 16
  uint16_t v16;      // the last reference on the curve, 16 counts/s units (the root's seed)
};

// The floor of the reference, within the cap.
static inline uint16_t move_brake_floor(uint16_t vel_min) {
  return (vel_min > MOVE_BRAKE_VEL_CAP) ? (uint16_t)MOVE_BRAKE_VEL_CAP : vel_min;
}

// The counts between the carriage and the run-in: the distance the brake has
// to work in, from `err` counts of error, never negative.
static inline uint16_t move_brake_distance(uint16_t err, int16_t deadband) {
  int16_t d = (int16_t)err - deadband - MOVE_BRAKE_RUN_IN;
  return d < 0 ? 0 : (uint16_t)d;
}

// Whether the brake is due: a reference falling at a constant rate from
// `velocity` (the carriage's speed toward the target, counts/s; under the
// floor, at rest or moving away, it counts as the floor) to the floor in
// `time_ms` covers (velocity + floor) / 2 * time_ms / 1000 counts, and the
// brake starts when the `distance` left to the run-in is no more than that.
// Within the run-in it is always due.
static inline bool move_brake_due(uint16_t distance, int32_t velocity, uint16_t time_ms, uint16_t floor_vel) {
  // the reference never asks for less than the floor, so a carriage under it
  // (at rest, moving away) is braked for as one at the floor
  if (velocity < (int32_t)floor_vel) velocity = floor_vel;
  // distance <= (v + floor) * T / 2000, as distance * 2000 <= (v + floor) * T:
  // the right side at most (MOVE_VEL_CLAMP + the floor) * 2000 = 6.7e7, the
  // left 1023 * 2000.
  return (uint32_t)distance * 2000UL <= (uint32_t)(velocity + floor_vel) * time_ms;
}

// The deceleration, in sixteenths, that takes `velocity` counts/s down to the
// floor over `distance` counts: (v^2 - floor^2) / (2 * distance) / 16, clamped
// to MOVE_BRAKE_ACCEL_MAX / 16. 0 (the curve flat at the floor) when the
// carriage is already at or under the floor, or there is no distance left.
static inline uint16_t move_brake_accel16(int32_t velocity, uint16_t distance, uint16_t floor_vel) {
  if (velocity <= (int32_t)floor_vel || distance == 0) return 0;
  // v^2 under MOVE_VEL_CLAMP^2 = 1.08e9; the quotient is a / 16 =
  // (v^2 - fl^2) / (32 * d)
  uint32_t a16 = ((uint32_t)velocity * velocity - (uint32_t)floor_vel * floor_vel) / ((uint32_t)distance << 5);
  return (a16 > (MOVE_BRAKE_ACCEL_MAX >> 4)) ? (uint16_t)(MOVE_BRAKE_ACCEL_MAX >> 4) : (uint16_t)a16;
}

// The braking curve's reference for `distance` counts left to the run-in,
// in 16 counts/s units: sqrt(floor16^2 + 2 * a * distance / 256) =
// sqrt(floor16^2 + accel16 * distance / 8). Newton's root descending from
// `seed`, which is at or above it (the previous tick's value plus a little:
// the curve only falls as the carriage closes; the cap at the brake's
// start): a seed under the root is returned as is. One division a tick once
// on the curve (the ATtiny's flash is nearly full).
static inline uint16_t move_brake_curve16(uint16_t accel16, uint16_t distance, uint16_t floor16, uint16_t seed) {
  // accel16 * distance is at most 32768 * 1023, under 2^25
  uint32_t room = (uint32_t)floor16 * floor16 + (((uint32_t)accel16 * distance) >> 3);
  uint16_t mag = seed ? seed : 1;
  for (;;) {
    uint32_t next = ((uint32_t)mag + room / mag) >> 1;
    if (next >= mag) break;
    mag = (uint16_t)next;
  }
  return mag < floor16 ? floor16 : mag;
}

// Plan toward a new target `error_q8` (target - position) away: a fresh
// move, a retarget or a reversal all start by cruising; the first step
// decides whether the brake is already due. A plan must be aimed before it is
// stepped.
static inline void move_brake_aim(MoveBrake& p, int32_t error_q8) {
  p.braking = false;
  p.rising = error_q8 > 0;
}

// Advance the plan by one control tick and return the velocity reference for
// move_drive(), signed toward the target. Cruising: the cap (`max_velocity`
// when that is not 0, MOVE_VEL_UNLIMITED, within MOVE_BRAKE_VEL_CAP as
// main.cpp's apply_move_speed keeps it; else MOVE_BRAKE_VEL_CAP), until the
// brake is due by `time_ms` and the measured `velocity` (signed, + toward the
// motor end), when the deceleration is taken from that speed and the distance
// left. Braking: the curve for the error left outside the `deadband` and the
// run-in, never under `vel_min` in magnitude, never over the cap. A reference
// planned the other way (an overshoot past the deadband without a new aim)
// becomes the floor toward the target, braking.
static inline int32_t move_brake_step(MoveBrake& p, int32_t error_q8, int32_t velocity, uint16_t time_ms,
                                      uint16_t max_velocity, uint16_t vel_min, int16_t deadband) {
  bool rising = error_q8 > 0;
  uint16_t fl = move_brake_floor(vel_min);
  uint16_t cap = (max_velocity != 0) ? max_velocity : (uint16_t)MOVE_BRAKE_VEL_CAP;
  int32_t err_q8 = rising ? error_q8 : -error_q8;
  uint16_t distance = move_brake_distance((uint16_t)(err_q8 >> MOVE_POS_SHIFT), deadband);
  int32_t toward = rising ? velocity : -velocity;
  if (p.rising != rising) {
    // an overshoot past the deadband: the floor back, as move_vref would
    // hand out for this error; never a new cruise
    p.rising = rising;
    p.braking = true;
    p.accel16 = 0;  // the curve is the floor from any seed
  } else if (!p.braking) {
    if (!move_brake_due(distance, toward, time_ms, fl)) return rising ? (int32_t)cap : -(int32_t)cap;
    p.braking = true;
    p.accel16 = move_brake_accel16(toward, distance, fl);
    p.v16 = cap >> 4;  // a seed above the curve for the first root
  }
  uint16_t v16 = move_brake_curve16(p.accel16, distance, fl >> 4, (uint16_t)(p.v16 + 2));
  p.v16 = v16;
  int32_t v = (int32_t)v16 << 4;
  if (v > (int32_t)cap) v = cap;
  if (v < (int32_t)fl) v = fl;
  return rising ? v : -v;
}

// The velocity gain for this tick: `kv_q16` while cruising, shifted up by
// MOVE_BRAKE_KV_SHIFT while braking, within MOVE_BRAKE_KV_MAX_Q16.
static inline uint16_t move_brake_kv_q16(const MoveBrake& p, uint16_t kv_q16) {
  if (!p.braking) return kv_q16;
  uint32_t kv = (uint32_t)kv_q16 << MOVE_BRAKE_KV_SHIFT;
  return (kv > MOVE_BRAKE_KV_MAX_Q16) ? (uint16_t)MOVE_BRAKE_KV_MAX_Q16 : (uint16_t)kv;
}

// Forget the plan when a move ends (the gain is the law's again).
static inline void move_brake_end(MoveBrake& p) {
  p.braking = false;
}
