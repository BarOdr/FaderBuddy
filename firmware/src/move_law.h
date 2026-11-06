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

// The remote-movement control law in fixed point: the position filter, the
// velocity estimate, the position loop (error -> velocity reference), the
// velocity loop (reference -> duty) and the per-tick ramp step. Pure
// functions over integers, so a host build can exercise them (test_move_law)
// against the float law they replaced; main.cpp owns the state and the
// gains and calls them once per ADC conversion or control tick.
//
// Why fixed point: the ATtiny1616's flash is the binding constraint, and the
// float law pulled in about 1.1 KB of avr-libc soft float. The law is the
// same cascade as before ("The control law" in ABOUT_MOTOR_CONTROL.md); only
// the representation changed. Scales:
//
//   position   ADC counts in Q8 (counts * 256): 0-1023 counts fits int32 with
//              room for every product below
//   velocity   ADC counts per second, whole
//   duty       whole, 0-254, except the ramps, which accumulate in Q8
//   gains      Q16 where fractional (1/k, KV); whole otherwise

#pragma once

#include <stdint.h>

#define MOVE_POS_SHIFT (8)
#define MOVE_POS_ONE (1L << MOVE_POS_SHIFT)
#define MOVE_Q16_ONE (65536L)

// Position filter coefficient, Q16: 0.05 (the ALPHA the float filter ran).
#define MOVE_POS_ALPHA_Q16 (3277)
// Velocity estimate smoothing: the float VELOCITY_TAU_S of 4 ms, as the Q16
// alpha per microsecond of tick (1/4000 us * 65536).
#define MOVE_VEL_ALPHA_PER_US_Q16 (16)
// The longest tick the ramps and the velocity alpha are integrated over; a
// stalled loop beyond this is a resynchronisation, not a longer tick.
#define MOVE_DT_MAX_US (10000UL)

// The live gains, derived per unit by apply_motor_calibration() (motor_cal.cpp)
// from the measured plant or the MOVE_* defaults. Per-direction arrays are
// indexed [MOTORCAL_FALLING] = 0, [MOTORCAL_RISING] = 1.
struct MoveLaw {
  int16_t bd[2];          // breakaway duty
  uint16_t inv_k_q16[2];  // 1/k, duty per (ADC count/sec), Q16
  uint16_t kv_q16;        // velocity loop gain, duty per (ADC count/sec), Q16
  uint16_t vel_min;       // Stribeck floor on the velocity reference, counts/s
  int16_t deadband;       // on-target window, counts
  int16_t takeup_duty;    // drive ceiling when a move is armed
};

// The gain derivation apply_motor_calibration() used in float: 1/k and the
// velocity loop gain 1.16/k_avg (MOTORCAL_KV_LOOP_GAIN), both Q16, k clamped
// away from zero.
#define MOVE_KV_LOOP_GAIN_Q16 (76022UL)  // 1.16 * 65536

static inline uint16_t move_inv_k_q16(uint16_t k) {
  (void)k;
  return 0;
}

static inline uint16_t move_kv_q16(uint16_t k_avg) {
  (void)k_avg;
  return 0;
}

// Fold one ADC conversion into the position filter. `adc_raw` is the 2x
// accumulated reading (0-2046); the filter is pos += 0.05 * (adc/2 - pos),
// as the float EWMA was.
static inline int32_t move_pos_filter(int32_t pos_q8, uint16_t adc_raw) {
  (void)adc_raw;
  return pos_q8;
}

// Advance the velocity estimate by one control tick: the position change over
// the measured tick, smoothed with the 4 ms time constant (alpha capped at 1).
static inline int32_t move_velocity_filter(int32_t velocity, int32_t dpos_q8, uint32_t dt_us) {
  (void)dpos_q8;
  (void)dt_us;
  return velocity;
}

// Position loop: the signed velocity reference for `error_q8`
// (target - position): slope * error, capped at `max_velocity` when that is
// not 0 (MOVE_VEL_UNLIMITED), never below `vel_min` in magnitude.
static inline int32_t move_vref(int32_t error_q8, uint16_t slope, uint16_t max_velocity, uint16_t vel_min) {
  (void)error_q8;
  (void)slope;
  (void)max_velocity;
  (void)vel_min;
  return 0;
}

// Velocity loop: the signed slow-decay duty that delivers `v_ref` given the
// measured `velocity`: feedforward (plant inverted, breakaway + |v_ref|/k in
// the direction of v_ref) plus kv * (v_ref - velocity). Truncated toward zero
// as the float law's output was when cast; unclamped, the caller owns the
// deadband and the drive ceiling.
static inline int16_t move_drive(int32_t v_ref, int32_t velocity, const MoveLaw& g) {
  (void)v_ref;
  (void)velocity;
  (void)g;
  return 0;
}

// How much a ramp of `rate` duty per second advances in a tick of `dt_us`, in
// Q8 duty. For the stall escape and the take-up ceiling. `rate` up to 3500
// and `dt_us` up to MOVE_DT_MAX_US (longer ticks count as that).
static inline int32_t move_ramp_step_q8(uint16_t rate, uint32_t dt_us) {
  (void)rate;
  (void)dt_us;
  return 0;
}

// Whole counts from a Q8 position or error, truncated toward zero as a float
// cast truncates.
static inline int16_t move_q8_to_int(int32_t q8) {
  (void)q8;
  return 0;
}
