// Host tests for the smooth motion profile's second design (move_brake.h,
// "cruise and brake"): the brake's arithmetic, and whole moves simulated
// against a model of the plant through the real control cascade
// (move_law.h), so the shape of a move can be judged without a fader. The
// plant model is the reference unit's measured first-order response
// (ABOUT_MOTOR_CONTROL.md, "The plant"), as in test_move_profile; it tops out
// near 5400 counts/s where the bench reaches 8000, so the simulated times
// are a design check, and the bench measures.

#include <unity.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_control.h"
#include "move_brake.h"
#include "shared/i2c_data.h"

// --------------------------------------------------------------------------
// The reference unit's plant and the law's gains for it (MOVE_* defaults).
// --------------------------------------------------------------------------

#define PLANT_K (29)        // counts/s per duty above breakaway
#define PLANT_BD_RISING (68)
#define PLANT_BD_FALLING (86)
#define PLANT_TAU_US (5000)  // coast time constant
#define PLANT_BRAKE_TAU_US (2000)
#define TICK_US (500)
#define ADC_PER_TICK (5)
#define VEL_MIN (560)
#define FLOOR16 (VEL_MIN / 16)
#define DEADBAND (8)
#define TAKEUP_DUTY (130)
#define TAKEUP_RATE (1200)
#define MAX_DUTY (254)
#define STEADY_MS (300)
#define SIM_LIMIT_MS (3000)
#define TIME_MS (MOVE_BRAKE_TIME_DEFAULT_MS)
// Less than 1% of travel, the Automatone's figure.
#define OVERSHOOT_MAX (10)

static MoveLaw law;

static void law_defaults() {
  law.bd[MOTORCAL_FALLING] = PLANT_BD_FALLING;
  law.bd[MOTORCAL_RISING] = PLANT_BD_RISING;
  law.inv_k_q16[MOTORCAL_FALLING] = move_inv_k_q16(PLANT_K);
  law.inv_k_q16[MOTORCAL_RISING] = move_inv_k_q16(PLANT_K);
  law.kv_q16 = move_kv_q16(PLANT_K);
  law.vel_min = VEL_MIN;
  law.deadband = DEADBAND;
  law.takeup_duty = TAKEUP_DUTY;
}

static int32_t q8(int32_t counts) { return counts << MOVE_POS_SHIFT; }

// --------------------------------------------------------------------------
// A simulated move: the loop of main.cpp's MODE_REMOTE_MOVEMENT_IN_PROGRESS
// branch without the stall escape (the model has no stall: the floor's
// feedforward is above breakaway), against the plant model.
// --------------------------------------------------------------------------

struct Sim {
  bool arrived;          // entered the deadband and stayed STEADY_MS
  uint32_t arrival_us;   // when it entered the deadband for the last time
  uint32_t t10_us;       // 10% of the distance covered
  uint32_t t90_us;       // 90% of the distance covered
  int32_t overshoot;     // furthest past the target, counts (0 if never)
  int32_t vref_floor;    // smallest |v_ref| seen while outside the deadband
  int32_t peak_vref;
  uint32_t brake_us;     // when the plan started braking (0 if never)
  double arrival_vel;    // the carriage's speed when it entered the deadband
  uint32_t ticks;
};

struct Retarget {
  uint32_t at_us;   // 0 = none
  int16_t target;
};

static Sim simulate(int16_t start, int16_t target, uint16_t time_ms, uint16_t max_velocity, Retarget retarget) {
  Sim s = {};
  double pos = start;        // counts
  double vel = 0;            // counts/s
  int32_t pos_q8 = q8(start);
  int32_t last_pos_q8 = pos_q8;
  int32_t velocity_est = 0;
  int32_t ceiling_q8 = q8(TAKEUP_DUTY);
  MoveBrake p = {};
  int16_t cur_target = target;
  int32_t error0 = (int32_t)target - start;
  int32_t error0_abs = error0 < 0 ? -error0 : error0;
  s.vref_floor = 0x7fffffff;
  bool retargeted = (retarget.at_us == 0);

  move_brake_aim(p, q8(cur_target) - pos_q8);

  uint32_t steady_since_us = 0;
  bool in_deadband = false;
  for (uint32_t t = 0; t < (uint32_t)SIM_LIMIT_MS * 1000; t += TICK_US) {
    if (!retargeted && t >= retarget.at_us) {
      retargeted = true;
      int32_t old_dir = (cur_target > start) ? 1 : -1;
      cur_target = retarget.target;
      int32_t new_err = q8(cur_target) - pos_q8;
      int32_t new_dir = (new_err > 0) ? 1 : -1;
      move_brake_aim(p, new_err);
      if (new_dir != old_dir) ceiling_q8 = q8(TAKEUP_DUTY);
    }
    // sensing: ADC conversions since the last tick
    for (int i = 0; i < ADC_PER_TICK; i++) {
      double p_adc = pos < 0 ? 0 : (pos > 1023 ? 1023 : pos);
      uint16_t adc_raw = (uint16_t)(p_adc * 2.0 + 0.5);
      pos_q8 = move_pos_filter(pos_q8, adc_raw);
    }
    velocity_est = move_velocity_filter(velocity_est, pos_q8 - last_pos_q8, TICK_US);
    last_pos_q8 = pos_q8;

    int32_t error_q8 = q8(cur_target) - pos_q8;
    int32_t deadband_q8 = q8(law.deadband);
    int16_t drive = 0;
    bool braked = false;
    if (error_q8 > deadband_q8 || error_q8 < -deadband_q8) {
      in_deadband = false;
      int32_t v_ref = move_brake_step(p, error_q8, velocity_est, time_ms, max_velocity, law.vel_min, law.deadband);
      if (p.braking && s.brake_us == 0) s.brake_us = t;
      int32_t v_abs = v_ref < 0 ? -v_ref : v_ref;
      if (v_abs < s.vref_floor) s.vref_floor = v_abs;
      if (v_abs > s.peak_vref) s.peak_vref = v_abs;
      int32_t u = move_drive(v_ref, velocity_est, law, move_brake_kv_q16(p, law.kv_q16));
      ceiling_q8 += move_ramp_step_q8(TAKEUP_RATE, TICK_US);
      if (ceiling_q8 > q8(MAX_DUTY)) ceiling_q8 = q8(MAX_DUTY);
      int16_t limit = move_q8_to_int(ceiling_q8);
      drive = (u > limit) ? limit : (u < -limit) ? -limit : (int16_t)u;
    } else {
      if (!in_deadband) {
        in_deadband = true;
        steady_since_us = t;
        s.arrival_us = t;
        s.arrival_vel = vel < 0 ? -vel : vel;
      } else if (t - steady_since_us >= (uint32_t)STEADY_MS * 1000) {
        s.arrived = true;
        break;
      }
      braked = true;
    }
    s.ticks++;

    // plant: first-order toward the steady-state speed of this drive
    double v_ss;
    double tau_us = PLANT_TAU_US;
    if (braked) {
      v_ss = 0;
      tau_us = PLANT_BRAKE_TAU_US;
    } else {
      double mag = drive < 0 ? -drive : drive;
      double bd = (drive > 0) ? PLANT_BD_RISING : PLANT_BD_FALLING;
      if (mag <= bd) {
        v_ss = 0;
      } else {
        v_ss = (mag - bd) * PLANT_K;
        if (drive < 0) v_ss = -v_ss;
      }
    }
    vel += (v_ss - vel) * ((double)TICK_US / tau_us);
    pos += vel * ((double)TICK_US / 1e6);
    if (pos < 0) { pos = 0; vel = 0; }
    if (pos > 1023) { pos = 1023; vel = 0; }

    int32_t covered = (int32_t)((pos - start) * (error0 < 0 ? -1 : 1) + 0.5);
    if (s.t10_us == 0 && covered * 10 >= error0_abs) s.t10_us = t;
    if (s.t90_us == 0 && covered * 10 >= error0_abs * 9) s.t90_us = t;
    int32_t past = (int32_t)((pos - cur_target) * ((cur_target > start) ? 1 : -1) + 0.5);
    if (past > s.overshoot) s.overshoot = past;
  }
  if (s.vref_floor == 0x7fffffff) s.vref_floor = 0;
  return s;
}

static Retarget no_retarget() {
  Retarget r = {0, 0};
  return r;
}

static void print_sim(const char* name, const Sim& s) {
  unsigned long t1090 = (s.t90_us > s.t10_us) ? (s.t90_us - s.t10_us) / 1000 : 0;
  printf("  %-28s arrived=%d t=%lu ms 10-90=%lu ms brake_at=%lu ms overshoot=%ld peak=%ld floor=%ld arrival_vel=%.0f\n",
         name, (int)s.arrived, (unsigned long)(s.arrival_us / 1000), t1090, (unsigned long)(s.brake_us / 1000),
         (long)s.overshoot, (long)s.peak_vref, (long)s.vref_floor, s.arrival_vel);
}

// --------------------------------------------------------------------------
// Tests
// --------------------------------------------------------------------------

void setUp() { law_defaults(); }
void tearDown() {}

void test_cruise_reference_is_the_cap() {
  MoveBrake p = {};
  move_brake_aim(p, q8(500));
  // far from the target, from rest: cruising, the velocity loop saturated
  TEST_ASSERT_EQUAL_INT32(MOVE_BRAKE_VEL_CAP, move_brake_step(p, q8(500), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  TEST_ASSERT_FALSE(p.braking);
  // the speed byte caps it
  TEST_ASSERT_EQUAL_INT32(3000, move_brake_step(p, q8(500), 2000, TIME_MS, 3000, VEL_MIN, DEADBAND));
  // falling mirrors rising
  move_brake_aim(p, q8(-500));
  TEST_ASSERT_EQUAL_INT32(-MOVE_BRAKE_VEL_CAP, move_brake_step(p, q8(-500), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  TEST_ASSERT_FALSE(p.braking);
}

void test_brake_is_due_by_time_from_the_measured_speed() {
  // From 8000 counts/s to the floor in 120 ms: (8000 + 560) / 2 * 0.12 =
  // 513.6 counts, so the brake is due at 513 counts to the run-in, not 514.
  TEST_ASSERT_FALSE(move_brake_due(514, 8000, 120, VEL_MIN));
  TEST_ASSERT_TRUE(move_brake_due(513, 8000, 120, VEL_MIN));
  // a longer brake time starts it earlier
  TEST_ASSERT_TRUE(move_brake_due(1000, 8000, 240, VEL_MIN));
  TEST_ASSERT_FALSE(move_brake_due(1028, 8000, 240, VEL_MIN));
  // at the floor's speed only the floor's own distance over the brake
  // time: 560 * 0.12 = 67.2
  TEST_ASSERT_TRUE(move_brake_due(67, 560, 120, VEL_MIN));
  TEST_ASSERT_FALSE(move_brake_due(68, 560, 120, VEL_MIN));
  // moving away, or at rest, counts as the floor's distance
  TEST_ASSERT_TRUE(move_brake_due(67, 0, 120, VEL_MIN));
  TEST_ASSERT_TRUE(move_brake_due(67, -5000, 120, VEL_MIN));
  TEST_ASSERT_FALSE(move_brake_due(68, -5000, 120, VEL_MIN));
  // no distance left: due at any speed and time
  TEST_ASSERT_TRUE(move_brake_due(0, 0, 50, VEL_MIN));
  // the distance to the run-in: error minus the deadband minus the run-in
  TEST_ASSERT_EQUAL_UINT16(513, move_brake_distance(513 + DEADBAND + MOVE_BRAKE_RUN_IN, DEADBAND));
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_distance(DEADBAND + MOVE_BRAKE_RUN_IN, DEADBAND));
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_distance(3, DEADBAND));
}

void test_deceleration_from_the_entry_speed_and_distance() {
  // (8000^2 - 560^2) / (2 * 513) = 62064 counts/s^2: 3879 in sixteenths
  TEST_ASSERT_UINT16_WITHIN(1, 3879, move_brake_accel16(8000, 513, VEL_MIN));
  // (5400^2 - 560^2) / (2 * 300) = 48070: 3004
  TEST_ASSERT_UINT16_WITHIN(1, 3004, move_brake_accel16(5400, 300, VEL_MIN));
  // at or under the floor: flat
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_accel16(560, 100, VEL_MIN));
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_accel16(200, 100, VEL_MIN));
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_accel16(-8000, 100, VEL_MIN));
  // no distance: flat too (the curve is the floor from here)
  TEST_ASSERT_EQUAL_UINT16(0, move_brake_accel16(8000, 0, VEL_MIN));
  // clamped
  TEST_ASSERT_EQUAL_UINT16(MOVE_BRAKE_ACCEL_MAX >> 4, move_brake_accel16(16000, 1, VEL_MIN));
  TEST_ASSERT_EQUAL_UINT16(MOVE_BRAKE_ACCEL_MAX >> 4, move_brake_accel16(32000, 1, VEL_MIN));
}

void test_curve_falls_to_the_floor_at_the_run_in() {
  // the brake planned above: 8000 -> 560 over 513 counts
  uint16_t a16 = move_brake_accel16(8000, 513, VEL_MIN);
  // at the entry distance the curve is the entry speed (within a unit)
  TEST_ASSERT_UINT16_WITHIN(1, 8000 / 16, move_brake_curve16(a16, 513, FLOOR16, 1000));
  // halfway: sqrt(560^2 + 2 * 62064 * 256) = 5665
  TEST_ASSERT_UINT16_WITHIN(1, 5665 / 16, move_brake_curve16(a16, 256, FLOOR16, 1000));
  // sqrt(560^2 + 2 * 62064 * 100) = 3567
  TEST_ASSERT_UINT16_WITHIN(1, 3567 / 16, move_brake_curve16(a16, 100, FLOOR16, 1000));
  // the floor at the run-in
  TEST_ASSERT_EQUAL_UINT16(FLOOR16, move_brake_curve16(a16, 0, FLOOR16, 1000));
  // seeded from just above the root, the same answers
  TEST_ASSERT_UINT16_WITHIN(1, 5665 / 16, move_brake_curve16(a16, 256, FLOOR16, 5665 / 16 + 2));
  // seeded from under it (the position jittered up a count): the seed
  TEST_ASSERT_EQUAL_UINT16(5665 / 16 - 1, move_brake_curve16(a16, 256, FLOOR16, 5665 / 16 - 1));
  // flat: the floor everywhere
  TEST_ASSERT_EQUAL_UINT16(FLOOR16, move_brake_curve16(0, 500, FLOOR16, 1000));
  // the largest product stays in range: at the clamp over full travel,
  // sqrt(35^2 + 32768 * 1023 / 8) = 2047
  TEST_ASSERT_EQUAL_UINT16(2047, move_brake_curve16(MOVE_BRAKE_ACCEL_MAX >> 4, 1023, FLOOR16, 4000));
}

void test_step_brakes_on_the_curve_and_ends_at_the_floor() {
  MoveBrake p = {};
  // cruising at 8000 counts/s, 600 counts out: not due yet (545 is the edge)
  move_brake_aim(p, q8(600));
  TEST_ASSERT_EQUAL_INT32(MOVE_BRAKE_VEL_CAP, move_brake_step(p, q8(600), 8000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  TEST_ASSERT_FALSE(p.braking);
  // at 545 counts of error (513 to the run-in): the brake starts at the
  // measured speed
  int32_t v = move_brake_step(p, q8(545), 8000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_INT32_WITHIN(16, 8000, v);
  // the curve falls with the error, never rises, never under the floor, and
  // is the floor from the run-in on; the measured speed no longer matters
  int32_t last = v;
  for (int32_t err = 544; err > DEADBAND; err--) {
    v = move_brake_step(p, q8(err), 8000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
    TEST_ASSERT_TRUE_MESSAGE(v <= last, "reference rose while braking");
    TEST_ASSERT_TRUE_MESSAGE(v >= VEL_MIN, "reference under the floor");
    if (err <= DEADBAND + MOVE_BRAKE_RUN_IN) TEST_ASSERT_EQUAL_INT32(VEL_MIN, v);
    last = v;
  }
  // a position that jitters back up a count does not lift it by more than
  // one unit of the curve
  v = move_brake_step(p, q8(DEADBAND + 1), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, v);
  // the speed byte caps the curve too
  move_brake_aim(p, q8(545));
  v = move_brake_step(p, q8(545), 8000, TIME_MS, 3000, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_EQUAL_INT32(3000, v);
}

void test_short_move_crawls_at_the_floor() {
  // 40 counts from rest: within the brake's reach at once, no speed to shed
  MoveBrake p = {};
  move_brake_aim(p, q8(40));
  int32_t v = move_brake_step(p, q8(40), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, v);
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, move_brake_step(p, q8(20), 500, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  move_brake_aim(p, q8(-40));
  TEST_ASSERT_EQUAL_INT32(-VEL_MIN, move_brake_step(p, q8(-40), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
}

void test_wrong_way_reference_is_the_floor_toward_the_target() {
  MoveBrake p = {};
  move_brake_aim(p, q8(300));
  move_brake_step(p, q8(300), 0, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  // overshot past the deadband without a new aim: the floor back, braking
  int32_t v = move_brake_step(p, q8(-12), -1500, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_EQUAL_INT32(-VEL_MIN, v);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_FALSE(p.rising);
  // and stays the floor, not a new cruise, while it comes back
  TEST_ASSERT_EQUAL_INT32(-VEL_MIN, move_brake_step(p, q8(-10), 400, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  // a bounce the other way: the same
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, move_brake_step(p, q8(9), -400, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
}

void test_retarget_replans_from_the_measured_speed() {
  MoveBrake p = {};
  move_brake_aim(p, q8(800));
  // braking toward 800 from 545 out
  move_brake_step(p, q8(545), 8000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  move_brake_step(p, q8(400), 6000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  // a farther target: cruise again
  move_brake_aim(p, q8(900));
  TEST_ASSERT_FALSE(p.braking);
  TEST_ASSERT_EQUAL_INT32(MOVE_BRAKE_VEL_CAP, move_brake_step(p, q8(900), 6000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND));
  // a nearer target, set late: braking at once from the speed it has, on a
  // curve that is continuous with that speed (one unit)
  move_brake_aim(p, q8(200));
  int32_t v = move_brake_step(p, q8(200), 6000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_INT32_WITHIN(16, 6000, v);
  // one so late that no deceleration within the clamp reaches the floor:
  // the curve is under the speed, so the loop brakes as hard as it can
  move_brake_aim(p, q8(40));
  v = move_brake_step(p, q8(40), 6000, TIME_MS, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(p.braking);
  TEST_ASSERT_TRUE(v < 6000);
  TEST_ASSERT_TRUE(v >= VEL_MIN);
}

void test_kv_is_raised_only_while_braking() {
  MoveBrake p = {};
  move_brake_aim(p, q8(500));
  TEST_ASSERT_EQUAL_UINT16(MOVE_KV_Q16, move_brake_kv_q16(p, MOVE_KV_Q16));
  p.braking = true;
  TEST_ASSERT_EQUAL_UINT16(MOVE_KV_Q16 << MOVE_BRAKE_KV_SHIFT, move_brake_kv_q16(p, MOVE_KV_Q16));
  // the lab's largest gain stays within what move_drive keeps in 32 bits
  TEST_ASSERT_EQUAL_UINT16(MOVE_BRAKE_KV_MAX_Q16, move_brake_kv_q16(p, 16383));
  TEST_ASSERT_EQUAL_UINT16(MOVE_BRAKE_KV_MAX_Q16, move_brake_kv_q16(p, 9000));
  move_brake_end(p);
  TEST_ASSERT_EQUAL_UINT16(MOVE_KV_Q16, move_brake_kv_q16(p, MOVE_KV_Q16));
}

void test_move_drive_takes_a_gain() {
  // the law's own gain: the same drive as the three-argument loop
  TEST_ASSERT_EQUAL_INT16(move_drive(1000, 2000, law), move_drive(1000, 2000, law, law.kv_q16));
  TEST_ASSERT_EQUAL_INT16(move_drive(-1000, 500, law), move_drive(-1000, 500, law, law.kv_q16));
  // twice the gain: twice the feedback on the same velocity error. kv 0.04,
  // error -1000: -40 duty; at 0.08: -80.
  int16_t once = move_drive(1000, 2000, law, 2621);
  int16_t twice = move_drive(1000, 2000, law, 5242);
  TEST_ASSERT_INT16_WITHIN(1, -40, twice - once);
}

void test_simulated_full_move_is_fast_and_lands_without_overshoot() {
  Sim up = simulate(0, 1023, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  print_sim("full up", up);
  TEST_ASSERT_TRUE(up.arrived);
  // the Automatone's 0.22-0.26 s; the model's plant is slower than the bench
  TEST_ASSERT_TRUE_MESSAGE(up.arrival_us >= 150000 && up.arrival_us <= 350000, "full travel outside 150-350 ms");
  TEST_ASSERT_TRUE_MESSAGE(up.overshoot <= OVERSHOOT_MAX, "overshot by more than 1%");
  // it braked: the reference came down to the floor before the deadband
  TEST_ASSERT_TRUE(up.brake_us > 0);
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, up.vref_floor);
  // it cruised at the cap first: the take-off is the fast profile's
  TEST_ASSERT_EQUAL_INT32(MOVE_BRAKE_VEL_CAP, up.peak_vref);
  // and arrived at a speed the bridge brake stops inside the deadband
  TEST_ASSERT_TRUE_MESSAGE(up.arrival_vel < 1400, "arrived faster than the brake can absorb");
}

void test_simulated_falling_move_mirrors_a_rising_one() {
  Sim up = simulate(0, 1023, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim down = simulate(1023, 0, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  print_sim("full down", down);
  TEST_ASSERT_TRUE(down.arrived);
  TEST_ASSERT_TRUE_MESSAGE(down.overshoot <= OVERSHOOT_MAX, "overshot by more than 1%");
  int32_t diff = (int32_t)down.arrival_us - (int32_t)up.arrival_us;
  if (diff < 0) diff = -diff;
  TEST_ASSERT_TRUE_MESSAGE(diff * 100 <= (int32_t)up.arrival_us * 15, "up and down differ by more than 15%");
}

void test_simulated_shorter_moves_are_quicker_and_land() {
  Sim full = simulate(0, 1023, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim half = simulate(0, 512, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim quarter = simulate(0, 256, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim brake_only = simulate(0, 80, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim crawl = simulate(0, 40, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  print_sim("half", half);
  print_sim("quarter", quarter);
  print_sim("80 counts", brake_only);
  print_sim("40 counts", crawl);
  TEST_ASSERT_TRUE(half.arrived && quarter.arrived && brake_only.arrived && crawl.arrived);
  TEST_ASSERT_TRUE(half.arrival_us < full.arrival_us);
  TEST_ASSERT_TRUE(quarter.arrival_us < half.arrival_us);
  TEST_ASSERT_TRUE(half.overshoot <= OVERSHOOT_MAX);
  TEST_ASSERT_TRUE(quarter.overshoot <= OVERSHOOT_MAX);
  TEST_ASSERT_TRUE(brake_only.overshoot <= OVERSHOOT_MAX);
  TEST_ASSERT_TRUE(crawl.overshoot <= OVERSHOOT_MAX);
  // the short one never left the floor
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, crawl.peak_vref);
}

void test_simulated_brake_time_scales_the_brake_only() {
  Sim quick = simulate(0, 1023, 50, MOVE_VEL_UNLIMITED, no_retarget());
  Sim def = simulate(0, 1023, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  Sim slow = simulate(0, 1023, 400, MOVE_VEL_UNLIMITED, no_retarget());
  print_sim("T=50", quick);
  print_sim("T=400", slow);
  TEST_ASSERT_TRUE(quick.arrived && slow.arrived);
  // a longer brake starts earlier and lands later
  TEST_ASSERT_TRUE(slow.brake_us < def.brake_us);
  TEST_ASSERT_TRUE(def.brake_us < quick.brake_us);
  TEST_ASSERT_TRUE(slow.arrival_us > def.arrival_us);
  TEST_ASSERT_TRUE(quick.arrival_us <= def.arrival_us);
  // the take-off is the same whatever the time: 10% in the same ticks
  TEST_ASSERT_EQUAL_UINT32(def.t10_us, slow.t10_us);
  TEST_ASSERT_EQUAL_UINT32(def.t10_us, quick.t10_us);
  TEST_ASSERT_TRUE(slow.overshoot <= OVERSHOOT_MAX);
  TEST_ASSERT_TRUE(quick.overshoot <= OVERSHOOT_MAX);
}

void test_simulated_speed_byte_still_caps_a_smooth_move() {
  Sim capped = simulate(0, 1023, TIME_MS, 2000, no_retarget());
  print_sim("capped 2000", capped);
  TEST_ASSERT_TRUE(capped.arrived);
  TEST_ASSERT_EQUAL_INT32(2000, capped.peak_vref);
  TEST_ASSERT_TRUE(capped.overshoot <= OVERSHOOT_MAX);
  Sim free = simulate(0, 1023, TIME_MS, MOVE_VEL_UNLIMITED, no_retarget());
  TEST_ASSERT_TRUE(capped.arrival_us > free.arrival_us);
}

void test_simulated_retargets_land() {
  // farther, while braking toward 500: cruises again, lands at 900
  Retarget farther = {150000, 900};
  Sim f = simulate(0, 500, TIME_MS, MOVE_VEL_UNLIMITED, farther);
  print_sim("retarget farther", f);
  TEST_ASSERT_TRUE(f.arrived);
  TEST_ASSERT_TRUE(f.overshoot <= OVERSHOOT_MAX);
  // nearer, set late while cruising toward 1000: brakes at once
  Retarget nearer = {120000, 450};
  Sim n = simulate(0, 1000, TIME_MS, MOVE_VEL_UNLIMITED, nearer);
  print_sim("retarget nearer", n);
  TEST_ASSERT_TRUE(n.arrived);
  TEST_ASSERT_TRUE(n.overshoot <= OVERSHOOT_MAX);
  // a reversal: back to where it came from
  Retarget back = {150000, 0};
  Sim r = simulate(0, 1000, TIME_MS, MOVE_VEL_UNLIMITED, back);
  print_sim("reversal", r);
  TEST_ASSERT_TRUE(r.arrived);
  TEST_ASSERT_TRUE(r.overshoot <= OVERSHOOT_MAX);
}

void test_default_brake_time_is_on_the_wire() {
  TEST_ASSERT_EQUAL_UINT16(120, MOVE_BRAKE_TIME_DEFAULT_MS);
  TEST_ASSERT_TRUE(MOVE_BRAKE_TIME_DEFAULT_MS >= MOVE_PROFILE_TIME_MIN_MS);
  TEST_ASSERT_TRUE(MOVE_BRAKE_TIME_DEFAULT_MS <= MOVE_PROFILE_TIME_MAX_MS);
  TEST_ASSERT_EQUAL_UINT16(MOVE_BRAKE_TIME_DEFAULT_MS, move_profile_time_from_wire(0, MOVE_BRAKE_TIME_DEFAULT_MS));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_cruise_reference_is_the_cap);
  RUN_TEST(test_brake_is_due_by_time_from_the_measured_speed);
  RUN_TEST(test_deceleration_from_the_entry_speed_and_distance);
  RUN_TEST(test_curve_falls_to_the_floor_at_the_run_in);
  RUN_TEST(test_step_brakes_on_the_curve_and_ends_at_the_floor);
  RUN_TEST(test_short_move_crawls_at_the_floor);
  RUN_TEST(test_wrong_way_reference_is_the_floor_toward_the_target);
  RUN_TEST(test_retarget_replans_from_the_measured_speed);
  RUN_TEST(test_kv_is_raised_only_while_braking);
  RUN_TEST(test_move_drive_takes_a_gain);
  RUN_TEST(test_simulated_full_move_is_fast_and_lands_without_overshoot);
  RUN_TEST(test_simulated_falling_move_mirrors_a_rising_one);
  RUN_TEST(test_simulated_shorter_moves_are_quicker_and_land);
  RUN_TEST(test_simulated_brake_time_scales_the_brake_only);
  RUN_TEST(test_simulated_speed_byte_still_caps_a_smooth_move);
  RUN_TEST(test_simulated_retargets_land);
  RUN_TEST(test_default_brake_time_is_on_the_wire);
  return UNITY_END();
}
