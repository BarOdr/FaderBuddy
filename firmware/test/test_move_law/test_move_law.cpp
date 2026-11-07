// Host tests of the fixed-point control law (firmware/src/move_law.h) against
// the float law it replaced. The float reference below is the law as main.cpp
// ran it up to FW 1.8 (position EWMA, velocity estimate, control_law(), stall
// ramp and take-up ceiling), copied verbatim with its gains derived the way
// apply_motor_calibration() derived them. Each test sweeps the inputs, prints
// the largest deviation it saw and asserts the bound the representation
// justifies: the fixed-point law must be the same law, not merely similar.
//
//   pio test -e native_test

#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "move_law.h"

#define MOTORCAL_FALLING (0)
#define MOTORCAL_RISING (1)

// ---------------------------------------------------------------------------
// The float reference (FW 1.8). Do not "improve" it: it is the specification.
// ---------------------------------------------------------------------------

static const float REF_ALPHA = 0.05;
static const float REF_VELOCITY_TAU_S = 0.004f;
static const float REF_VREF_SLOPE = 50.0f;
static const float REF_VEL_UNLIMITED = 0.0f;
static const float REF_RAMP_MAX = 70.0f;
static const int16_t REF_MAX_DUTY = 254;

struct RefGains {
  float bd[2];
  float inv_k[2];
  float kv;
  float vel_min;
};

static float ref_pos_filter(float input_ewma, uint16_t adc_val) {
  return adc_val * REF_ALPHA / 2 + input_ewma * (1 - REF_ALPHA);
}

static float ref_velocity_filter(float velocity_ewma, float input_ewma, float last_control_ewma, uint32_t control_dt_us) {
  float dt = control_dt_us * 1e-6f;
  if (dt > 0.0f) {
    float dv = (input_ewma - last_control_ewma) / dt;
    float alpha_v = dt / REF_VELOCITY_TAU_S;
    if (alpha_v > 1.0f) alpha_v = 1.0f;
    velocity_ewma += alpha_v * (dv - velocity_ewma);
  }
  return velocity_ewma;
}

static float ref_control_law(float error, float max_velocity, float velocity_ewma, const RefGains& g) {
  float sign = (error > 0) ? 1.0f : -1.0f;
  float mag = REF_VREF_SLOPE * error * sign;  // |v_ref|
  if (max_velocity > REF_VEL_UNLIMITED && mag > max_velocity) mag = max_velocity;
  if (mag < g.vel_min) mag = g.vel_min;

  uint8_t d = (error > 0) ? MOTORCAL_RISING : MOTORCAL_FALLING;
  return sign * (g.bd[d] + mag * g.inv_k[d]) + g.kv * (sign * mag - velocity_ewma);
}

// apply_motor_calibration(), the part that produced the float gains.
static RefGains ref_gains(int16_t bd_falling, int16_t bd_rising, uint16_t k_falling, uint16_t k_rising, uint16_t vel_min) {
  RefGains g;
  uint16_t k_avg = (k_falling + k_rising) / 2;
  g.kv = 1.16f / (float)k_avg;
  g.bd[MOTORCAL_FALLING] = bd_falling;
  g.bd[MOTORCAL_RISING] = bd_rising;
  g.inv_k[MOTORCAL_FALLING] = 1.0f / (float)(k_falling ? k_falling : 1);
  g.inv_k[MOTORCAL_RISING] = 1.0f / (float)(k_rising ? k_rising : 1);
  g.vel_min = vel_min;
  return g;
}

static MoveLaw fixed_gains(int16_t bd_falling, int16_t bd_rising, uint16_t k_falling, uint16_t k_rising, uint16_t vel_min) {
  MoveLaw g;
  g.bd[MOTORCAL_FALLING] = bd_falling;
  g.bd[MOTORCAL_RISING] = bd_rising;
  g.inv_k_q16[MOTORCAL_FALLING] = move_inv_k_q16(k_falling);
  g.inv_k_q16[MOTORCAL_RISING] = move_inv_k_q16(k_rising);
  g.kv_q16 = move_kv_q16((k_falling + k_rising) / 2);
  g.vel_min = vel_min;
  g.deadband = 8;
  g.takeup_duty = 130;
  return g;
}

static int16_t clamp_duty(int32_t u) {
  if (u > REF_MAX_DUTY) return REF_MAX_DUTY;
  if (u < -REF_MAX_DUTY) return -REF_MAX_DUTY;
  return (int16_t)u;
}

static float fabs_(float x) { return x < 0 ? -x : x; }

// Deterministic noise so the sweeps are repeatable across hosts.
static uint32_t lcg_state = 12345;
static uint32_t lcg() {
  lcg_state = lcg_state * 1664525u + 1013904223u;
  return lcg_state >> 8;
}

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Position filter
// ---------------------------------------------------------------------------

void test_position_filter_tracks_the_float_ewma() {
  // A fader sweep with ADC dither, full scale down and back up, plus a hold.
  float ref = 0;
  int32_t pos = 0;
  float worst = 0;
  for (int i = 0; i < 20000; ++i) {
    int phase = i / 5000;
    int t = i % 5000;
    int16_t centre;
    if (phase == 0) centre = (int16_t)(t * 2046L / 5000);
    else if (phase == 1) centre = 2046;
    else if (phase == 2) centre = (int16_t)(2046 - t * 2046L / 5000);
    else centre = 1000;
    int16_t raw = centre + (int16_t)(lcg() % 5) - 2;
    if (raw < 0) raw = 0;
    if (raw > 2046) raw = 2046;
    ref = ref_pos_filter(ref, (uint16_t)raw);
    pos = move_pos_filter(pos, (uint16_t)raw);
    float dev = fabs_(ref - pos / 256.0f);
    if (dev > worst) worst = dev;
  }
  printf("position filter: max deviation %.4f ADC counts\n", worst);
  // One ADC count is 1/16 mm; a tenth of a count is below the dither.
  TEST_ASSERT_TRUE(worst < 0.1f);
}

void test_position_filter_settles_on_a_constant_input() {
  int32_t pos = 0;
  for (int i = 0; i < 2000; ++i) pos = move_pos_filter(pos, 1400);
  // Within 10/256 of a count: the Q16 alpha rounds a smaller difference to no
  // step (65536 / 3277 / 2). The float filter converged exactly; 0.04 count
  // is 2.5 um of travel, under the ADC dither.
  TEST_ASSERT_INT32_WITHIN(10, 700L * MOVE_POS_ONE, pos);
  for (int i = 0; i < 2000; ++i) pos = move_pos_filter(pos, 0);
  TEST_ASSERT_INT32_WITHIN(10, 0, pos);
}

// ---------------------------------------------------------------------------
// Velocity estimate
// ---------------------------------------------------------------------------

void test_velocity_filter_tracks_the_float_estimate() {
  // A move at up to full plant speed (8000 counts/s) over 800 counts, seen
  // through the position filter, with the tick wandering around its nominal
  // 500 us: 25 ms of acceleration, 50 ms flat, 25 ms of deceleration.
  float ref_pos = 100 * 1.0f, ref_last = ref_pos, ref_vel = 0;
  int32_t pos = 100 * MOVE_POS_ONE, last = pos, vel = 0;
  float true_pos = 100;
  float worst = 0;
  int32_t peak = 0;
  for (int i = 0; i < 400; ++i) {
    uint32_t dt_us = 450 + lcg() % 100;
    float v = (i < 50) ? 8000.0f * i / 50 : (i < 150) ? 8000.0f : (i < 200) ? 8000.0f - 8000.0f * (i - 150) / 50 : 0.0f;
    true_pos += v * dt_us * 1e-6f;
    if (true_pos > 1000) true_pos = 1000;
    // Five conversions per tick, dithered.
    for (int c = 0; c < 5; ++c) {
      int32_t raw = (int32_t)(true_pos * 2) + (int32_t)(lcg() % 3) - 1;
      if (raw < 0) raw = 0;
      ref_pos = ref_pos_filter(ref_pos, (uint16_t)raw);
      pos = move_pos_filter(pos, (uint16_t)raw);
    }
    ref_vel = ref_velocity_filter(ref_vel, ref_pos, ref_last, dt_us);
    ref_last = ref_pos;
    vel = move_velocity_filter(vel, pos - last, dt_us);
    last = pos;
    float dev = fabs_(ref_vel - vel);
    if (dev > worst) worst = dev;
    if (vel > peak) peak = vel;
  }
  printf("velocity estimate: max deviation %.2f counts/s (peak %ld)\n", worst, (long)peak);
  TEST_ASSERT_TRUE(peak > 7000);
  // One Q8 position step per tick is 7.8 counts/s; allow two of them. Through
  // KV (0.04 duty per count/s) that is under one duty.
  TEST_ASSERT_TRUE(worst < 16.0f);
}

void test_velocity_filter_caps_alpha_at_one_on_a_long_tick() {
  // A 10 ms tick: alpha would be 2.5, so the estimate just takes dv.
  int32_t vel = move_velocity_filter(5000, 4 * MOVE_POS_ONE, 10000);
  TEST_ASSERT_INT32_WITHIN(2, 400, vel);
  // And a tick of zero length does not divide by zero or move the estimate.
  TEST_ASSERT_EQUAL_INT32(5000, move_velocity_filter(5000, 4 * MOVE_POS_ONE, 0));
}

// ---------------------------------------------------------------------------
// Control law
// ---------------------------------------------------------------------------

struct Plant {
  int16_t bd_f, bd_r;
  uint16_t k_f, k_r;
  uint16_t vel_min;
};

// The reference unit's defaults, the calibration bounds, and a stiff and a
// loose unit in between.
static const Plant PLANTS[] = {
    {86, 68, 29, 29, 560},   // MOVE_* defaults
    {120, 110, 8, 9, 1500},  // K_MIN, VEL_MIN_HI
    {40, 35, 90, 85, 200},   // K_MAX, VEL_MIN_LO
    {100, 60, 20, 40, 700},  // asymmetric
};
static const uint16_t MAX_VELOCITIES[] = {0, 600, 1462, 3000, 4092};

void test_control_law_matches_the_float_law_over_a_sweep() {
  int32_t worst = 0;
  long samples = 0;
  for (unsigned p = 0; p < sizeof(PLANTS) / sizeof(PLANTS[0]); ++p) {
    const Plant& pl = PLANTS[p];
    RefGains rg = ref_gains(pl.bd_f, pl.bd_r, pl.k_f, pl.k_r, pl.vel_min);
    MoveLaw fg = fixed_gains(pl.bd_f, pl.bd_r, pl.k_f, pl.k_r, pl.vel_min);
    for (unsigned m = 0; m < sizeof(MAX_VELOCITIES) / sizeof(MAX_VELOCITIES[0]); ++m) {
      uint16_t max_v = MAX_VELOCITIES[m];
      for (int32_t error_q8 = -1023 * MOVE_POS_ONE; error_q8 <= 1023 * MOVE_POS_ONE; error_q8 += 37) {
        for (int32_t velocity = -8000; velocity <= 8000; velocity += 1009) {
          float ref = ref_control_law(error_q8 / 256.0f, max_v, velocity, rg);
          int16_t ref_duty = clamp_duty((int16_t)ref);
          int32_t v_ref = move_vref(error_q8, 50, max_v, pl.vel_min);
          int16_t duty = clamp_duty(move_drive(v_ref, velocity, fg));
          int32_t dev = labs((long)ref_duty - duty);
          if (dev > worst) worst = dev;
          ++samples;
        }
      }
    }
  }
  printf("control law: max deviation %ld duty over %ld samples\n", (long)worst, samples);
  // The gains carry 16 fractional bits and the sum 8, so inside the +-254
  // duty the two laws can differ only by a rounding-boundary crossing.
  TEST_ASSERT_TRUE(worst <= 1);
}

void test_control_law_at_zero_error_pulls_down_like_the_float_law() {
  // error == 0 took the falling branch in the float law (sign -1); keep it,
  // the haptics rely on the law being defined there.
  const Plant& pl = PLANTS[0];
  RefGains rg = ref_gains(pl.bd_f, pl.bd_r, pl.k_f, pl.k_r, pl.vel_min);
  MoveLaw fg = fixed_gains(pl.bd_f, pl.bd_r, pl.k_f, pl.k_r, pl.vel_min);
  int16_t ref_duty = (int16_t)ref_control_law(0.0f, 0, 0, rg);
  int32_t v_ref = move_vref(0, 50, 0, pl.vel_min);
  TEST_ASSERT_EQUAL_INT32(-(int32_t)pl.vel_min, v_ref);
  TEST_ASSERT_INT16_WITHIN(1, ref_duty, move_drive(v_ref, 0, fg));
}

void test_vref_floors_caps_and_signs_like_the_float_law() {
  TEST_ASSERT_EQUAL_INT32(560, move_vref(1 * MOVE_POS_ONE, 50, 0, 560));       // floored
  TEST_ASSERT_EQUAL_INT32(-560, move_vref(-1 * MOVE_POS_ONE, 50, 0, 560));
  TEST_ASSERT_EQUAL_INT32(1000, move_vref(20 * MOVE_POS_ONE, 50, 0, 560));     // slope
  TEST_ASSERT_EQUAL_INT32(-1000, move_vref(-20 * MOVE_POS_ONE, 50, 0, 560));
  TEST_ASSERT_EQUAL_INT32(800, move_vref(20 * MOVE_POS_ONE, 50, 800, 560));    // capped
  TEST_ASSERT_EQUAL_INT32(51150, move_vref(1023 * MOVE_POS_ONE, 50, 0, 560));  // unlimited
  TEST_ASSERT_EQUAL_INT32(560, move_vref(20 * MOVE_POS_ONE, 50, 300, 560));    // floor beats cap
}

void test_gain_derivation_matches_the_float_derivation() {
  // 1/k and 1.16/k_avg to within half a Q16 step over the characterisation's
  // range and below it (k = 1 does not fit Q16 and is clamped; the
  // characterisation never produces it, K_MIN is 8).
  for (uint16_t k = 2; k <= 120; ++k) {
    float want = 1.0f / (float)k;
    TEST_ASSERT_FLOAT_WITHIN(0.5f / 65536, want, move_inv_k_q16(k) / 65536.0f);
    float want_kv = 1.16f / (float)k;
    if (k >= 8) TEST_ASSERT_FLOAT_WITHIN(0.5f / 65536, want_kv, move_kv_q16(k) / 65536.0f);
  }
  TEST_ASSERT_EQUAL_UINT16(move_inv_k_q16(1), move_inv_k_q16(0));
  TEST_ASSERT_EQUAL_UINT16(move_kv_q16(1), move_kv_q16(0));
}

// ---------------------------------------------------------------------------
// Ramps: stall escape and take-up ceiling
// ---------------------------------------------------------------------------

void test_ramp_step_matches_rate_times_dt() {
  float worst = 0;
  const uint16_t rates[] = {250, 1200, 3500};
  for (unsigned r = 0; r < 3; ++r) {
    for (uint32_t dt_us = 1; dt_us <= 10000; dt_us += 7) {
      float want = rates[r] * dt_us * 1e-6f;
      float got = move_ramp_step_q8(rates[r], dt_us) / 256.0f;
      float dev = fabs_(want - got);
      if (dev > worst) worst = dev;
    }
  }
  printf("ramp step: max deviation %.5f duty per tick\n", worst);
  TEST_ASSERT_TRUE(worst < 1.0f / 256 + 0.0005f);
  // Longer ticks integrate as the longest allowed tick.
  TEST_ASSERT_EQUAL_INT32(move_ramp_step_q8(3500, MOVE_DT_MAX_US), move_ramp_step_q8(3500, 1000000UL));
}

void test_stall_ramp_and_ceiling_integrate_like_the_float_ramps() {
  // Stalled for 400 ms (the ramp winds to its cap), free for 50 ms (it sheds),
  // with the take-up ceiling opening from 130 throughout.
  float ref_ramp = 0, ref_ceiling = 130;
  int32_t ramp_q8 = 0, ceiling_q8 = 130L * MOVE_POS_ONE;
  int32_t worst_ramp = 0, worst_ceiling = 0;
  for (int i = 0; i < 900; ++i) {
    uint32_t dt_us = 450 + lcg() % 100;
    float dt = dt_us * 1e-6f;
    bool stalled = i < 800;
    if (stalled) {
      ref_ramp += 250.0f * dt;
      if (ref_ramp > REF_RAMP_MAX) ref_ramp = REF_RAMP_MAX;
      ramp_q8 += move_ramp_step_q8(250, dt_us);
      if (ramp_q8 > (int32_t)REF_RAMP_MAX * MOVE_POS_ONE) ramp_q8 = (int32_t)REF_RAMP_MAX * MOVE_POS_ONE;
    } else {
      ref_ramp -= 3500.0f * dt;
      if (ref_ramp < 0) ref_ramp = 0;
      ramp_q8 -= move_ramp_step_q8(3500, dt_us);
      if (ramp_q8 < 0) ramp_q8 = 0;
    }
    ref_ceiling += 1200.0f * dt;
    if (ref_ceiling > REF_MAX_DUTY) ref_ceiling = REF_MAX_DUTY;
    ceiling_q8 += move_ramp_step_q8(1200, dt_us);
    if (ceiling_q8 > (int32_t)REF_MAX_DUTY * MOVE_POS_ONE) ceiling_q8 = (int32_t)REF_MAX_DUTY * MOVE_POS_ONE;

    int32_t dev = labs((long)(int16_t)ref_ramp - move_q8_to_int(ramp_q8));
    if (dev > worst_ramp) worst_ramp = dev;
    dev = labs((long)(int16_t)ref_ceiling - move_q8_to_int(ceiling_q8));
    if (dev > worst_ceiling) worst_ceiling = dev;
    if (i == 799) TEST_ASSERT_EQUAL_INT16(70, move_q8_to_int(ramp_q8));
  }
  TEST_ASSERT_EQUAL_INT16(0, move_q8_to_int(ramp_q8));
  TEST_ASSERT_EQUAL_INT16(254, move_q8_to_int(ceiling_q8));
  printf("ramps: max deviation stall %ld, ceiling %ld duty\n", (long)worst_ramp, (long)worst_ceiling);
  TEST_ASSERT_TRUE(worst_ramp <= 1);
  TEST_ASSERT_TRUE(worst_ceiling <= 1);
}

// ---------------------------------------------------------------------------
// Q8 to whole counts, as a float cast truncates
// ---------------------------------------------------------------------------

// The lab's REG_DEBUG_GAINS can set the slope, kv and k independently, and
// move_drive's products must stay in 32 bits for every combination it allows.
// The slope is the one that has to be clamped: at the lab's largest kv
// (16383) and smallest k (2, inv_k 32768) a slope of 96 pushes both products
// past 2^31 and the drive wraps to the wrong sign, which the float law never
// did. Within the clamp the law is still the float law.
static float ref_law_with_slope(float error, float slope, float velocity_ewma, const RefGains& g) {
  float sign = (error > 0) ? 1.0f : -1.0f;
  float mag = slope * error * sign;
  if (mag < g.vel_min) mag = g.vel_min;
  uint8_t d = (error > 0) ? MOTORCAL_RISING : MOTORCAL_FALLING;
  return sign * (g.bd[d] + mag * g.inv_k[d]) + g.kv * (sign * mag - velocity_ewma);
}

void test_lab_slope_is_clamped_where_the_law_stays_exact() {
  TEST_ASSERT_EQUAL_UINT16(50, move_law_slope_from_wire(50000));
  TEST_ASSERT_EQUAL_UINT16(MOVE_VREF_SLOPE_MAX, move_law_slope_from_wire(96000));
  TEST_ASSERT_EQUAL_UINT16(MOVE_VREF_SLOPE_MAX, move_law_slope_from_wire(0xFFFFFFFFUL));
  TEST_ASSERT_EQUAL_UINT16(0, move_law_slope_from_wire(999));
  // the lab's extremes at the clamped slope, against the float law
  MoveLaw fg;
  fg.bd[0] = 86; fg.bd[1] = 68;
  fg.inv_k_q16[0] = fg.inv_k_q16[1] = move_inv_k_q16(2);
  fg.kv_q16 = 16383;
  fg.vel_min = 560; fg.deadband = 8; fg.takeup_duty = 130;
  RefGains rg;
  rg.bd[0] = 86; rg.bd[1] = 68;
  rg.inv_k[0] = rg.inv_k[1] = 0.5f;
  rg.kv = 16383 / 65536.0f;
  rg.vel_min = 560;
  int32_t worst = 0;
  for (int32_t error_q8 = -1023 * MOVE_POS_ONE; error_q8 <= 1023 * MOVE_POS_ONE; error_q8 += 1023) {
    for (int32_t velocity = -MOVE_VEL_CLAMP; velocity <= MOVE_VEL_CLAMP; velocity += 4681) {
      float ref = ref_law_with_slope(error_q8 / 256.0f, MOVE_VREF_SLOPE_MAX, velocity, rg);
      int16_t ref_duty = clamp_duty((int16_t)(ref > 32767.0f ? 32767.0f : (ref < -32767.0f ? -32767.0f : ref)));
      int32_t v_ref = move_vref(error_q8, MOVE_VREF_SLOPE_MAX, 0, 560);
      int16_t duty = clamp_duty(move_drive(v_ref, velocity, fg));
      int32_t dev = labs((long)ref_duty - duty);
      if (dev > worst) worst = dev;
    }
  }
  TEST_ASSERT_TRUE(worst <= 1);
}

void test_q8_to_int_truncates_toward_zero() {
  TEST_ASSERT_EQUAL_INT16(0, move_q8_to_int(0));
  TEST_ASSERT_EQUAL_INT16(0, move_q8_to_int(255));
  TEST_ASSERT_EQUAL_INT16(1, move_q8_to_int(256));
  TEST_ASSERT_EQUAL_INT16(0, move_q8_to_int(-255));
  TEST_ASSERT_EQUAL_INT16(-1, move_q8_to_int(-256));
  TEST_ASSERT_EQUAL_INT16(-7, move_q8_to_int(-7 * 256 - 200));
  TEST_ASSERT_EQUAL_INT16(1023, move_q8_to_int(1023L * 256 + 255));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_position_filter_tracks_the_float_ewma);
  RUN_TEST(test_position_filter_settles_on_a_constant_input);
  RUN_TEST(test_velocity_filter_tracks_the_float_estimate);
  RUN_TEST(test_velocity_filter_caps_alpha_at_one_on_a_long_tick);
  RUN_TEST(test_control_law_matches_the_float_law_over_a_sweep);
  RUN_TEST(test_control_law_at_zero_error_pulls_down_like_the_float_law);
  RUN_TEST(test_vref_floors_caps_and_signs_like_the_float_law);
  RUN_TEST(test_gain_derivation_matches_the_float_derivation);
  RUN_TEST(test_ramp_step_matches_rate_times_dt);
  RUN_TEST(test_stall_ramp_and_ceiling_integrate_like_the_float_ramps);
  RUN_TEST(test_lab_slope_is_clamped_where_the_law_stays_exact);
  RUN_TEST(test_q8_to_int_truncates_toward_zero);
  return UNITY_END();
}
