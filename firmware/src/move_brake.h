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
// +-1023 counts, the brake time within MOVE_PROFILE_TIME_MAX_MS.

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
// `velocity` (the carriage's speed toward the target, counts/s, 0 when it
// is moving away) to the floor in `time_ms` covers (velocity + floor) / 2 *
// time_ms / 1000 counts, and the brake starts when the `distance` left to the
// run-in is no more than that. Within the run-in it is always due.
static inline bool move_brake_due(uint16_t distance, int32_t velocity, uint16_t time_ms, uint16_t floor_vel) {
  (void)distance;
  (void)velocity;
  (void)time_ms;
  (void)floor_vel;
  return false;
}

// The deceleration, in sixteenths, that takes `velocity` counts/s down to the
// floor over `distance` counts: (v^2 - floor^2) / (2 * distance) / 16, clamped
// to MOVE_BRAKE_ACCEL_MAX / 16. 0 (the curve flat at the floor) when the
// carriage is already at or under the floor, or there is no distance left.
static inline uint16_t move_brake_accel16(int32_t velocity, uint16_t distance, uint16_t floor_vel) {
  (void)velocity;
  (void)distance;
  (void)floor_vel;
  return 0;
}

// The braking curve's reference for `distance` counts left to the run-in,
// in 16 counts/s units: sqrt(floor16^2 + 2 * a * distance / 256) =
// sqrt(floor16^2 + accel16 * distance / 8). Newton's root from `seed`, which
// is at or above it (the previous tick's value plus a little: the curve only
// falls as the carriage closes); a seed under the root is returned as is,
// within one unit of it.
static inline uint16_t move_brake_curve16(uint16_t accel16, uint16_t distance, uint16_t floor16, uint16_t seed) {
  (void)accel16;
  (void)distance;
  (void)seed;
  return floor16;
}

// Plan toward a new target `error_q8` (target - position) away: a fresh
// move, a retarget or a reversal all start by cruising; the first step
// decides whether the brake is already due. A plan must be aimed before it is
// stepped.
static inline void move_brake_aim(MoveBrake& p, int32_t error_q8) {
  p.braking = false;
  p.rising = error_q8 > 0;
  p.accel16 = 0;
  p.v16 = 0;
}

// Advance the plan by one control tick and return the velocity reference for
// move_drive(), signed toward the target. Cruising: the cap (`max_velocity`
// when that is not 0, MOVE_VEL_UNLIMITED; else MOVE_BRAKE_VEL_CAP), until the
// brake is due by `time_ms` and the measured `velocity` (signed, + toward the
// motor end), when the deceleration is taken from that speed and the distance
// left. Braking: the curve for the error left outside the `deadband` and the
// run-in, never under `vel_min` in magnitude, never over the cap. A reference
// planned the other way (an overshoot past the deadband without a new aim)
// becomes the floor toward the target, braking.
static inline int32_t move_brake_step(MoveBrake& p, int32_t error_q8, int32_t velocity, uint16_t time_ms,
                                      uint16_t max_velocity, uint16_t vel_min, int16_t deadband) {
  (void)velocity;
  (void)time_ms;
  (void)max_velocity;
  (void)deadband;
  bool rising = error_q8 > 0;
  p.rising = rising;
  int32_t fl = move_brake_floor(vel_min);
  return rising ? fl : -fl;
}

// The velocity gain for this tick: `kv_q16` while cruising, shifted up by
// MOVE_BRAKE_KV_SHIFT while braking, within MOVE_BRAKE_KV_MAX_Q16.
static inline uint16_t move_brake_kv_q16(const MoveBrake& p, uint16_t kv_q16) {
  (void)p;
  return kv_q16;
}

// Forget the plan when a move ends.
static inline void move_brake_end(MoveBrake& p) {
  p.braking = false;
  p.accel16 = 0;
  p.v16 = 0;
}
