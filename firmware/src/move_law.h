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
//
// Ranges every product here relies on: position and error within +-1023
// counts, velocity within +-MOVE_VEL_CLAMP, slope at most MOVE_VREF_SLOPE_MAX
// (60), 1/k with k >= 2
// (the characterisation never goes under K_MIN), kv_q16 at most 16383.
// Resolution: the position filter settles within 10/256 of a count of its
// input (the Q16 alpha rounds a smaller difference to no step), and one Q8
// position step per 500 us tick is 7.8 counts/s of velocity.

#pragma once

#include <stdint.h>

#define MOVE_POS_SHIFT (8)
#define MOVE_POS_ONE (1L << MOVE_POS_SHIFT)
#define MOVE_Q16_ONE (65536L)

// Position filter coefficient, Q16: 0.05 (the ALPHA the float filter ran).
#define MOVE_POS_ALPHA_Q16 (3277)
// Velocity estimate smoothing time constant (the float VELOCITY_TAU_S, 4 ms).
#define MOVE_VEL_TAU_US (4000UL)
// The velocity estimate is clamped here before smoothing. The plant tops out
// near 8000 counts/s; a bigger difference is a glitch (priming, a dropped
// tick), and keeping it bounded keeps every product below in 32 bits.
#define MOVE_VEL_CLAMP (32767L)
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
  if (k == 0) k = 1;
  uint32_t q = (MOVE_Q16_ONE + k / 2) / k;
  return (q > 65535UL) ? 65535U : (uint16_t)q;
}

static inline uint16_t move_kv_q16(uint16_t k_avg) {
  if (k_avg == 0) k_avg = 1;
  uint32_t q = (MOVE_KV_LOOP_GAIN_Q16 + k_avg / 2) / k_avg;
  return (q > 65535UL) ? 65535U : (uint16_t)q;
}

// Fold one ADC conversion into the position filter. `adc_raw` is the 2x
// accumulated reading (0-2046); the filter is pos += 0.05 * (adc/2 - pos),
// as the float EWMA was.
static inline int32_t move_pos_filter(int32_t pos_q8, uint16_t adc_raw) {
  // adc/2 in Q8 is adc << 7; the difference is at most +-1023 counts (2^18 in
  // Q8), times the Q16 alpha stays under 2^31. Rounded, so a constant input
  // settles on itself rather than one step below it.
  int32_t diff = ((int32_t)adc_raw << (MOVE_POS_SHIFT - 1)) - pos_q8;
  return pos_q8 + ((diff * MOVE_POS_ALPHA_Q16 + (1L << 15)) >> 16);
}

// Advance the velocity estimate by one control tick: the position change over
// the measured tick, smoothed with the 4 ms time constant (alpha capped at 1).
static inline int32_t move_velocity_filter(int32_t velocity, int32_t dpos_q8, uint32_t dt_us) {
  if (dt_us == 0) return velocity;
  // counts/s = dpos_q8 * (1e6 / 256) / dt_us; 3906 for 3906.25 is 0.006% low,
  // and dpos_q8 is at most 2^18, so the product stays under 2^31.
  int32_t dv = (dpos_q8 * 3906L) / (int32_t)dt_us;
  if (dv > MOVE_VEL_CLAMP) dv = MOVE_VEL_CLAMP;
  if (dv < -MOVE_VEL_CLAMP) dv = -MOVE_VEL_CLAMP;
  // alpha = dt / tau, capped at 1, in Q12: at Q8 the 500 us tick's alpha
  // (0.125) would be quantised to within 0.85%, enough to drift the estimate
  // by tens of counts/s through a transient. (dv - velocity) is under 2^17,
  // times 2^12 fits. Rounded: a floor would bias the estimate low.
  uint32_t alpha_q12 = (dt_us >= MOVE_VEL_TAU_US) ? 4096UL : (dt_us * 4096UL) / MOVE_VEL_TAU_US;
  return velocity + (((dv - velocity) * (int32_t)alpha_q12 + 2048L) >> 12);
}

// The largest slope a host may set at runtime (the lab's REG_DEBUG_GAINS).
// It is what keeps move_drive's products in 32 bits at the lab's other
// extremes: |v_ref| at most 60 * 1023 = 61380, times 1/k at k = 2 (32768 in
// Q16) is 2.01e9, and the velocity error at most 61380 + 32767 = 94147,
// times the lab's largest kv (16383) is 1.54e9.
#define MOVE_VREF_SLOPE_MAX (60)

// The slope a REG_DEBUG_GAINS write of `per_mille` (slope x 1000) selects.
static inline uint16_t move_law_slope_from_wire(uint32_t per_mille) {
  uint32_t slope = per_mille / 1000UL;
  return (slope > MOVE_VREF_SLOPE_MAX) ? (uint16_t)MOVE_VREF_SLOPE_MAX : (uint16_t)slope;
}

// Position loop: the signed velocity reference for `error_q8`
// (target - position): slope * error, capped at `max_velocity` when that is
// not 0 (MOVE_VEL_UNLIMITED), never below `vel_min` in magnitude.
static inline int32_t move_vref(int32_t error_q8, uint16_t slope, uint16_t max_velocity, uint16_t vel_min) {
  // error == 0 counts as falling, as the float law's sign did.
  bool rising = error_q8 > 0;
  int32_t mag_q8 = rising ? error_q8 : -error_q8;
  // slope (whole, at most MOVE_VREF_SLOPE_MAX) times 2^18 fits.
  int32_t mag = (mag_q8 * (int32_t)slope) >> MOVE_POS_SHIFT;  // |v_ref|
  if (max_velocity != 0 && mag > (int32_t)max_velocity) mag = max_velocity;
  if (mag < (int32_t)vel_min) mag = vel_min;
  return rising ? mag : -mag;
}

// Velocity loop: the signed slow-decay duty that delivers `v_ref` given the
// measured `velocity`: feedforward (plant inverted, breakaway + |v_ref|/k in
// the direction of v_ref) plus kv * (v_ref - velocity). Truncated toward zero
// as the float law's output was when cast; unclamped, the caller owns the
// deadband and the drive ceiling. The overload with `kv_q16` runs the loop
// with that gain in place of the law's (the smooth profile's brake raises
// it, move_brake.h); it must stay within 16383, as the law's is.
static inline int16_t move_drive(int32_t v_ref, int32_t velocity, const MoveLaw& g, uint16_t kv_q16);

static inline int16_t move_drive(int32_t v_ref, int32_t velocity, const MoveLaw& g) {
  return move_drive(v_ref, velocity, g, g.kv_q16);
}

static inline int16_t move_drive(int32_t v_ref, int32_t velocity, const MoveLaw& g, uint16_t kv_q16) {
  (void)kv_q16;
  bool rising = v_ref > 0;
  int32_t mag = rising ? v_ref : -v_ref;
  uint8_t d = rising ? 1 : 0;  // MOTORCAL_RISING : MOTORCAL_FALLING
  // |v_ref| is at most MOVE_VREF_SLOPE_MAX * 1023 = 61380 and 1/k at most
  // 2^15 in Q16 (k >= 2), so the feedforward product is under 2.01e9 and
  // fits; its Q16 drops to Q8 duty.
  int32_t ff_q8 = ((int32_t)g.bd[d] << MOVE_POS_SHIFT) + ((mag * (int32_t)g.inv_k_q16[d]) >> 8);
  // Velocity error at most 61380 + MOVE_VEL_CLAMP = 94147, kv_q16 at most
  // 16383 (k_avg >= 8 gives 9503; DEBUG_GAINS is clamped to 16383), so this
  // product is under 1.55e9 and fits too.
  int32_t verr = (rising ? mag : -mag) - velocity;
  int32_t fb_q8 = (verr * (int32_t)g.kv_q16) >> 8;
  int32_t u_q8 = (rising ? ff_q8 : -ff_q8) + fb_q8;
  // Truncate toward zero, as (int16_t)float did. The result can exceed int16
  // only for an unphysical input; the caller clamps to +-MOVE_MAX_DUTY anyway.
  int32_t u = u_q8 / MOVE_POS_ONE;
  if (u > 32767) u = 32767;
  if (u < -32767) u = -32767;
  return (int16_t)u;
}

// How much a ramp of `rate` duty per second advances in a tick of `dt_us`, in
// Q8 duty. For the stall escape and the take-up ceiling. `rate` up to 3500
// and `dt_us` up to MOVE_DT_MAX_US (longer ticks count as that).
static inline int32_t move_ramp_step_q8(uint16_t rate, uint32_t dt_us) {
  if (dt_us > MOVE_DT_MAX_US) dt_us = MOVE_DT_MAX_US;
  // rate * dt_us / 1e6 duty, in Q8: 256 / 1e6 = 16 / 62500; the product is at
  // most 3500 * 10000 * 16 = 5.6e8.
  // Rounded, so the truncation does not accumulate into a lag over a ramp.
  return (int32_t)(((uint32_t)rate * dt_us * 16UL + 31250UL) / 62500UL);
}

// Whole counts from a Q8 position or error, truncated toward zero as a float
// cast truncates.
static inline int16_t move_q8_to_int(int32_t q8) {
  return (int16_t)(q8 / MOVE_POS_ONE);  // C division truncates toward zero
}
