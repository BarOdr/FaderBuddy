// Host tests for the smooth motion profile (move_profile.h): the planner's
// arithmetic, and whole moves simulated against a model of the plant through
// the real control cascade (move_law.h), so the shape of a move can be judged
// without a fader. The plant model is the reference unit's measured
// first-order response (ABOUT_MOTOR_CONTROL.md, "The plant"): breakaway,
// k counts/s per duty above it, a 5 ms coast time constant, and the control
// loop's own sensing (the position EWMA at the ADC rate, the velocity
// estimate per tick). It is a model: the numbers below are what the design
// predicts, and the bench measures.

#include <unity.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_control.h"
#include "move_profile.h"
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
#define DEADBAND (8)
#define TAKEUP_DUTY (130)
#define TAKEUP_RATE (1200)
#define MAX_DUTY (254)
#define STEADY_MS (300)
#define SIM_LIMIT_MS (3000)
#define TIME_MS (MOVE_PROFILE_TIME_DEFAULT_MS)

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

// --------------------------------------------------------------------------
// A simulated move: the loop of main.cpp's MODE_REMOTE_MOVEMENT_IN_PROGRESS
// branch without the stall escape (the model has no stall: the floor's
// feedforward is above breakaway), driven either by the profile or by the
// legacy reference (move_vref), against the plant model.
// --------------------------------------------------------------------------

struct Sim {
  // outcome
  bool arrived;          // entered the deadband and stayed STEADY_MS
  uint32_t arrival_us;   // when it entered the deadband for the last time
  uint32_t t10_us;       // 10% of the distance covered
  uint32_t t90_us;       // 90% of the distance covered
  int32_t overshoot;     // furthest past the target, counts (0 if never)
  int32_t vref_floor;    // smallest |v_ref| seen while outside the deadband
  int32_t vref_jump;     // largest |v_ref change| between ticks
  int32_t first_drive;   // |drive| on the first tick
  int32_t drive_jump;    // largest |drive change| between driven ticks
  int32_t peak_vref;
  uint32_t ticks;
};

struct Retarget {
  uint32_t at_us;   // 0 = none
  int16_t target;
};

static Sim simulate(int16_t start, int16_t target, uint16_t time_ms, uint16_t max_velocity, bool legacy,
                    Retarget retarget) {
  Sim s = {};
  double pos = start;        // counts
  double vel = 0;            // counts/s
  int32_t pos_q8 = (int32_t)start << MOVE_POS_SHIFT;
  int32_t last_pos_q8 = pos_q8;
  int32_t velocity_est = 0;
  int32_t ceiling_q8 = (int32_t)TAKEUP_DUTY << MOVE_POS_SHIFT;
  int32_t last_vref = 0;
  bool have_last_vref = false;
  int16_t last_drive = 0;
  bool have_last_drive = false;
  MoveProfile p = {0, 0};
  int16_t cur_target = target;
  int32_t error0 = (int32_t)target - start;
  int32_t error0_abs = error0 < 0 ? -error0 : error0;
  s.vref_floor = 0x7fffffff;
  bool retargeted = (retarget.at_us == 0);

  move_profile_aim(p, ((int32_t)cur_target << MOVE_POS_SHIFT) - pos_q8, time_ms, law.vel_min, law.deadband, false);

  uint32_t steady_since_us = 0;
  bool in_deadband = false;
  for (uint32_t t = 0; t < (uint32_t)SIM_LIMIT_MS * 1000; t += TICK_US) {
    if (!retargeted && t >= retarget.at_us) {
      retargeted = true;
      int32_t old_dir = (cur_target > start) ? 1 : -1;
      cur_target = retarget.target;
      int32_t new_err = ((int32_t)cur_target << MOVE_POS_SHIFT) - pos_q8;
      int32_t new_dir = (new_err > 0) ? 1 : -1;
      move_profile_aim(p, new_err, time_ms, law.vel_min, law.deadband, new_dir == old_dir);
      if (new_dir != old_dir) ceiling_q8 = (int32_t)TAKEUP_DUTY << MOVE_POS_SHIFT;
    }
    // sensing: ADC conversions since the last tick
    for (int i = 0; i < ADC_PER_TICK; i++) {
      double p_adc = pos < 0 ? 0 : (pos > 1023 ? 1023 : pos);
      uint16_t adc_raw = (uint16_t)(p_adc * 2.0 + 0.5);
      pos_q8 = move_pos_filter(pos_q8, adc_raw);
    }
    velocity_est = move_velocity_filter(velocity_est, pos_q8 - last_pos_q8, TICK_US);
    last_pos_q8 = pos_q8;

    int32_t error_q8 = ((int32_t)cur_target << MOVE_POS_SHIFT) - pos_q8;
    int32_t deadband_q8 = (int32_t)law.deadband << MOVE_POS_SHIFT;
    int16_t drive = 0;
    bool braking = false;
    if (error_q8 > deadband_q8 || error_q8 < -deadband_q8) {
      in_deadband = false;
      int32_t v_ref = legacy ? move_vref(error_q8, 50, max_velocity, law.vel_min)
                             : move_profile_step(p, error_q8, max_velocity, law.vel_min, law.deadband);
      int32_t v_abs = v_ref < 0 ? -v_ref : v_ref;
      if (v_abs < s.vref_floor) s.vref_floor = v_abs;
      if (v_abs > s.peak_vref) s.peak_vref = v_abs;
      if (have_last_vref) {
        int32_t jump = v_ref - last_vref;
        if (jump < 0) jump = -jump;
        if (jump > s.vref_jump) s.vref_jump = jump;
      }
      last_vref = v_ref;
      have_last_vref = true;
      int32_t u = move_drive(v_ref, velocity_est, law);
      ceiling_q8 += move_ramp_step_q8(TAKEUP_RATE, TICK_US);
      if (ceiling_q8 > ((int32_t)MAX_DUTY << MOVE_POS_SHIFT)) ceiling_q8 = (int32_t)MAX_DUTY << MOVE_POS_SHIFT;
      int16_t limit = move_q8_to_int(ceiling_q8);
      drive = (u > limit) ? limit : (u < -limit) ? -limit : (int16_t)u;
      if (s.ticks == 0) s.first_drive = drive < 0 ? -drive : drive;
      if (have_last_drive) {
        int32_t jump = (int32_t)drive - last_drive;
        if (jump < 0) jump = -jump;
        if (jump > s.drive_jump) s.drive_jump = jump;
      }
      last_drive = drive;
      have_last_drive = true;
    } else {
      have_last_drive = false;
      if (!in_deadband) {
        in_deadband = true;
        steady_since_us = t;
        s.arrival_us = t;
      } else if (t - steady_since_us >= (uint32_t)STEADY_MS * 1000) {
        s.arrived = true;
        break;
      }
      braking = true;
    }
    s.ticks++;

    // plant: first-order toward the steady-state speed of this drive
    double v_ss;
    double tau_us = PLANT_TAU_US;
    if (braking) {
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

    // progress along the original move, for the 10-90% time
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
  // a move that ends inside the deadband short of 90% has no 10-90 time
  unsigned long t1090 = (s.t90_us > s.t10_us) ? (s.t90_us - s.t10_us) / 1000 : 0;
  printf("  %-26s arrived=%d t=%lu ms 10-90=%lu ms overshoot=%ld peak=%ld floor=%ld jump=%ld first_drive=%ld drive_jump=%ld\n",
         name, (int)s.arrived, (unsigned long)(s.arrival_us / 1000), t1090, (long)s.overshoot, (long)s.peak_vref,
         (long)s.vref_floor, (long)s.vref_jump, (long)s.first_drive, (long)s.drive_jump);
}

// The largest change the planner may make to the reference in one tick: its
// acceleration over the tick, plus the floor/cap rounding.
static int32_t max_step(uint32_t accel) {
  return (int32_t)((uint64_t)accel * TICK_US / 1000000UL) + 2;
}

// The plan's state in the units the tests reason in: a signed reference in
// counts/s and an acceleration in counts/s^2.
static void set_vref(MoveProfile& p, int32_t v) {
  p.rising = v > 0;
  p.v_q2 = (uint16_t)((v > 0 ? v : -v) << 2);
}
static uint32_t accel_of(const MoveProfile& p) { return (uint32_t)p.accel16 << 4; }

// --------------------------------------------------------------------------
// Tests
// --------------------------------------------------------------------------

void setUp() { law_defaults(); }
void tearDown() {}

void test_accel_is_four_d_over_t_squared() {
  // 4 * 1023 / 0.24^2 = 71042
  TEST_ASSERT_UINT32_WITHIN(710, 71042, move_profile_accel(1023, 240));
  // 4 * 20 / 0.24^2 = 1389
  TEST_ASSERT_UINT32_WITHIN(28, 1389, move_profile_accel(20, 240));
  // 4 * 512 / 2^2 = 512
  TEST_ASSERT_UINT32_WITHIN(6, 512, move_profile_accel(512, 2000));
  // 4 * 1023 / 0.05^2 = 1.6M: clamped
  TEST_ASSERT_EQUAL_UINT32(MOVE_PROFILE_ACCEL_MAX, move_profile_accel(1023, 50));
  TEST_ASSERT_EQUAL_UINT32(0, move_profile_accel(0, 240));
}

void test_soft_start_ramps_from_the_floor() {
  MoveProfile p = {0, 0};
  int32_t error_q8 = (int32_t)1000 << MOVE_POS_SHIFT;
  move_profile_aim(p, error_q8, TIME_MS, VEL_MIN, DEADBAND, false);
  uint32_t accel = move_profile_accel(1000, TIME_MS);
  int32_t last = VEL_MIN - 1;
  int32_t v = 0;
  // 20 ms of ticks with the error held (the plant has not moved yet)
  for (int i = 0; i < 40; i++) {
    v = move_profile_step(p, error_q8, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
    TEST_ASSERT_TRUE_MESSAGE(v >= last, "reference must not fall while accelerating");
    TEST_ASSERT_TRUE_MESSAGE(v - last <= max_step(accel), "reference stepped by more than accel*dt");
    last = v;
  }
  // the first tick is the floor (plus at most one step)
  TEST_ASSERT_TRUE(v > VEL_MIN);
  // 560 + 69444 * 0.02 = 1949
  int32_t expected = VEL_MIN + (int32_t)((uint64_t)accel * 20000 / 1000000UL);
  TEST_ASSERT_INT32_WITHIN(expected / 20, expected, v);
}

void test_falling_move_mirrors_a_rising_one() {
  MoveProfile up = {0, 0}, down = {0, 0};
  int32_t e = (int32_t)600 << MOVE_POS_SHIFT;
  move_profile_aim(up, e, TIME_MS, VEL_MIN, DEADBAND, false);
  move_profile_aim(down, -e, TIME_MS, VEL_MIN, DEADBAND, false);
  for (int i = 0; i < 100; i++) {
    int32_t vu = move_profile_step(up, e, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
    int32_t vd = move_profile_step(down, -e, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
    TEST_ASSERT_EQUAL_INT32(vu, -vd);
    TEST_ASSERT_TRUE(vd < 0);
  }
}

void test_brakes_to_the_floor_at_the_deadband() {
  // At the deadband's edge nothing is left to brake over: a reference at
  // speed comes down at its deceleration and sits on the floor, never below.
  MoveProfile p = {0, 0};
  move_profile_aim(p, (int32_t)800 << MOVE_POS_SHIFT, TIME_MS, VEL_MIN, DEADBAND, false);
  set_vref(p, 4000);
  uint32_t accel = accel_of(p);
  int32_t error_q8 = (int32_t)DEADBAND << MOVE_POS_SHIFT;
  int32_t last = 4000;
  for (int i = 0; i < 200; i++) {
    int32_t v = move_profile_step(p, error_q8, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
    TEST_ASSERT_TRUE_MESSAGE(v <= last, "reference must fall while braking");
    TEST_ASSERT_TRUE_MESSAGE(last - v <= max_step(accel), "reference dropped by more than accel*dt");
    TEST_ASSERT_TRUE_MESSAGE(v >= VEL_MIN, "reference went under the floor");
    last = v;
  }
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, last);
  // With error left it holds on the braking curve, the speed from which the
  // floor is reached at the deceleration over what is left: within a step.
  set_vref(p, 4000);
  int32_t v = 0;
  for (int i = 0; i < 200; i++) {
    v = move_profile_step(p, (int32_t)(DEADBAND + 2) << MOVE_POS_SHIFT, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  }
  int32_t curve = (int32_t)sqrt((double)VEL_MIN * VEL_MIN + 2.0 * accel * 2);
  TEST_ASSERT_INT32_WITHIN(max_step(accel), curve, v);
}

void test_wrong_way_reference_restarts_at_the_floor() {
  MoveProfile p = {0, 0};
  move_profile_aim(p, (int32_t)800 << MOVE_POS_SHIFT, TIME_MS, VEL_MIN, DEADBAND, false);
  set_vref(p, 3000);
  // overshot past the deadband: the error has changed sign
  int32_t v = move_profile_step(p, -((int32_t)30 << MOVE_POS_SHIFT), MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_INT32_WITHIN(max_step(accel_of(p)), -VEL_MIN, v);
  TEST_ASSERT_TRUE(v < 0);
}

void test_speed_cap_and_floor_bound_the_reference() {
  MoveProfile p = {0, 0};
  int32_t e = (int32_t)1000 << MOVE_POS_SHIFT;
  move_profile_aim(p, e, TIME_MS, VEL_MIN, DEADBAND, false);
  int32_t v = 0;
  for (int i = 0; i < 400; i++) {
    v = move_profile_step(p, e, 2000, VEL_MIN, DEADBAND);
    TEST_ASSERT_TRUE(v <= 2000);
    TEST_ASSERT_TRUE(v >= VEL_MIN);
  }
  TEST_ASSERT_EQUAL_INT32(2000, v);
  // a cap under the floor: the floor wins, as in move_vref
  MoveProfile q = {0, 0};
  move_profile_aim(q, e, TIME_MS, VEL_MIN, DEADBAND, false);
  TEST_ASSERT_EQUAL_INT32(VEL_MIN, move_profile_step(q, e, 100, VEL_MIN, DEADBAND));
  // no cap: the reference is still bounded
  MoveProfile r = {0, 0};
  move_profile_aim(r, e, MOVE_PROFILE_TIME_MIN_MS, VEL_MIN, DEADBAND, false);
  for (int i = 0; i < 2000; i++) v = move_profile_step(r, e, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_EQUAL_INT32(MOVE_PROFILE_VEL_CAP, v);
}

void test_aim_continuing_keeps_the_reference() {
  MoveProfile p = {0, 0};
  int32_t e = (int32_t)900 << MOVE_POS_SHIFT;
  move_profile_aim(p, e, TIME_MS, VEL_MIN, DEADBAND, false);
  int32_t v = 0;
  for (int i = 0; i < 100; i++) v = move_profile_step(p, e, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_TRUE(v > 3000);
  uint32_t accel_far = accel_of(p);
  // nearer target, same direction: the reference carries on, the
  // deceleration is not relaxed
  move_profile_aim(p, (int32_t)300 << MOVE_POS_SHIFT, TIME_MS, VEL_MIN, DEADBAND, true);
  TEST_ASSERT_EQUAL_UINT32(accel_far, accel_of(p));
  int32_t v2 = move_profile_step(p, (int32_t)300 << MOVE_POS_SHIFT, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_INT32_WITHIN(max_step(accel_of(p)), v, v2);
  // farther target: the larger acceleration is taken, the reference carries on
  move_profile_aim(p, (int32_t)1000 << MOVE_POS_SHIFT, MOVE_PROFILE_TIME_MIN_MS, VEL_MIN, DEADBAND, true);
  TEST_ASSERT_TRUE(accel_of(p) > accel_far);
  int32_t v3 = move_profile_step(p, (int32_t)1000 << MOVE_POS_SHIFT, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_INT32_WITHIN(max_step(accel_of(p)), v2, v3);
  // a reversal starts over at the floor
  move_profile_aim(p, -e, TIME_MS, VEL_MIN, DEADBAND, false);
  int32_t v4 = move_profile_step(p, -e, MOVE_VEL_UNLIMITED, VEL_MIN, DEADBAND);
  TEST_ASSERT_INT32_WITHIN(max_step(accel_of(p)), -VEL_MIN, v4);
  // and move_profile_end forgets the plan
  move_profile_end(p);
  TEST_ASSERT_EQUAL_UINT16(0, p.v_q2);
}

void test_simulated_full_move_is_soft_and_on_time() {
  Sim up = simulate(10, 1013, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim down = simulate(1013, 10, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim legacy = simulate(10, 1013, TIME_MS, MOVE_VEL_UNLIMITED, true, no_retarget());
  printf("\nfull travel, T=%d ms:\n", TIME_MS);
  print_sim("smooth up", up);
  print_sim("smooth down", down);
  print_sim("fast (legacy) up", legacy);
  TEST_ASSERT_TRUE(up.arrived);
  TEST_ASSERT_TRUE(down.arrived);
  TEST_ASSERT_TRUE(legacy.arrived);
  // soft start: the first tick's drive is the floor's feedforward, not full duty
  TEST_ASSERT_TRUE_MESSAGE(up.first_drive < TAKEUP_DUTY, "smooth move started at the take-up ceiling");
  TEST_ASSERT_TRUE_MESSAGE(legacy.first_drive >= TAKEUP_DUTY, "legacy move no longer starts hard");
  // on time: about T, allowing for the plant's top speed on a full move
  TEST_ASSERT_TRUE(up.arrival_us >= 150000UL && up.arrival_us <= 400000UL);
  TEST_ASSERT_TRUE(down.arrival_us >= 150000UL && down.arrival_us <= 400000UL);
  // symmetric: the feedforward absorbs the plant's direction asymmetry
  TEST_ASSERT_UINT32_WITHIN(up.arrival_us / 10, up.arrival_us, down.arrival_us);
  // the 10-90% time of an S-curve is roughly half the move (Automatone: 112-129 ms of 220-260)
  uint32_t t1090 = (up.t90_us - up.t10_us) / 1000;
  TEST_ASSERT_TRUE(t1090 >= 80 && t1090 <= 220);
  // no overshoot beyond the deadband
  TEST_ASSERT_TRUE(up.overshoot <= DEADBAND);
  TEST_ASSERT_TRUE(down.overshoot <= DEADBAND);
  // the reference never dips under the floor and never jumps
  TEST_ASSERT_TRUE(up.vref_floor >= VEL_MIN);
  TEST_ASSERT_TRUE(up.vref_jump <= max_step(move_profile_accel(1003, TIME_MS)));
}

void test_simulated_durations_are_about_independent_of_distance() {
  Sim quarter = simulate(100, 356, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim half = simulate(100, 612, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim full = simulate(10, 1013, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim small = simulate(500, 520, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim tiny = simulate(500, 510, TIME_MS, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim legacy_quarter = simulate(100, 356, TIME_MS, MOVE_VEL_UNLIMITED, true, no_retarget());
  Sim legacy_half = simulate(100, 612, TIME_MS, MOVE_VEL_UNLIMITED, true, no_retarget());
  printf("\ndistance, T=%d ms:\n", TIME_MS);
  print_sim("quarter (256)", quarter);
  print_sim("half (512)", half);
  print_sim("full (1003)", full);
  print_sim("small (20)", small);
  print_sim("tiny (10)", tiny);
  print_sim("fast (legacy) quarter", legacy_quarter);
  print_sim("fast (legacy) half", legacy_half);
  TEST_ASSERT_TRUE(quarter.arrived && half.arrived && full.arrived && small.arrived && tiny.arrived);
  // quarter and half are both under the plant's top speed: the same time
  TEST_ASSERT_UINT32_WITHIN(half.arrival_us / 4, half.arrival_us, quarter.arrival_us);
  // the full move runs into the plant's top speed; it is longer, but not twice
  TEST_ASSERT_TRUE(full.arrival_us <= half.arrival_us * 16 / 10);
  // a short move is on the floor the whole way, and quick
  TEST_ASSERT_TRUE(small.arrival_us < 100000UL);
  TEST_ASSERT_TRUE(small.overshoot <= DEADBAND && tiny.overshoot <= DEADBAND);
  TEST_ASSERT_TRUE(quarter.overshoot <= DEADBAND && half.overshoot <= DEADBAND);
}

void test_simulated_retarget_keeps_the_reference_continuous() {
  // retargeted mid-move, nearer and farther, same direction; and reversed
  Retarget nearer = {60000, 500};
  Retarget farther = {60000, 1000};
  Retarget back = {60000, 10};
  Sim n = simulate(10, 900, TIME_MS, MOVE_VEL_UNLIMITED, false, nearer);
  Sim f = simulate(10, 600, TIME_MS, MOVE_VEL_UNLIMITED, false, farther);
  Sim b = simulate(10, 900, TIME_MS, MOVE_VEL_UNLIMITED, false, back);
  printf("\nretarget at 60 ms:\n");
  print_sim("900 -> 500 (nearer)", n);
  print_sim("600 -> 1000 (farther)", f);
  print_sim("900 -> 10 (reversal)", b);
  TEST_ASSERT_TRUE(n.arrived && f.arrived && b.arrived);
  uint32_t accel_max_seen = move_profile_accel(1000, TIME_MS);
  TEST_ASSERT_TRUE(n.vref_jump <= max_step(accel_max_seen));
  TEST_ASSERT_TRUE(f.vref_jump <= max_step(accel_max_seen));
  TEST_ASSERT_TRUE(n.overshoot <= DEADBAND);
  TEST_ASSERT_TRUE(f.overshoot <= DEADBAND);
  TEST_ASSERT_TRUE(n.vref_floor >= VEL_MIN && f.vref_floor >= VEL_MIN);
  // the reversal restarts at the floor: its one jump is the reference flip
  TEST_ASSERT_TRUE(b.overshoot <= DEADBAND);
  TEST_ASSERT_TRUE(b.arrival_us <= 500000UL);
}

void test_simulated_speed_byte_still_caps_a_smooth_move() {
  Sim capped = simulate(10, 1013, TIME_MS, 2000, false, no_retarget());
  Sim legacy = simulate(10, 1013, TIME_MS, 2000, true, no_retarget());
  printf("\nspeed cap 2000 counts/s:\n");
  print_sim("smooth", capped);
  print_sim("fast (legacy)", legacy);
  TEST_ASSERT_TRUE(capped.arrived);
  TEST_ASSERT_TRUE(capped.peak_vref <= 2000);
  // 1003 counts at 2000 counts/s is 500 ms before the ramps
  TEST_ASSERT_TRUE(capped.arrival_us >= 500000UL);
  TEST_ASSERT_TRUE(capped.overshoot <= DEADBAND);
  TEST_ASSERT_UINT32_WITHIN(legacy.arrival_us / 4, legacy.arrival_us, capped.arrival_us);
}

void test_simulated_move_time_scales_the_move() {
  Sim fast = simulate(100, 612, 120, MOVE_VEL_UNLIMITED, false, no_retarget());
  Sim slow = simulate(100, 612, 480, MOVE_VEL_UNLIMITED, false, no_retarget());
  printf("\nhalf travel, T=120 and T=480 ms:\n");
  print_sim("T=120", fast);
  print_sim("T=480", slow);
  TEST_ASSERT_TRUE(fast.arrived && slow.arrived);
  TEST_ASSERT_UINT32_WITHIN(140000UL, 480000UL, slow.arrival_us);
  TEST_ASSERT_TRUE(fast.arrival_us < slow.arrival_us / 2);
  TEST_ASSERT_TRUE(fast.overshoot <= DEADBAND && slow.overshoot <= DEADBAND);
}

void test_simulated_late_nearer_retarget_brakes_in_time() {
  // A nearer target in the same direction, set late, when the reference is
  // already past where it could brake to the new target at this move's
  // deceleration: the planner must come down to its braking curve at once
  // (the fader cannot stop softly from where it is; the brake is the host's
  // doing), not keep braking at the move's own rate and run past the target.
  Retarget r100 = {100000, 400};
  Retarget r120 = {120000, 500};
  Retarget r140 = {140000, 600};
  Sim a = simulate(10, 900, TIME_MS, MOVE_VEL_UNLIMITED, false, r100);
  Sim b = simulate(10, 900, TIME_MS, MOVE_VEL_UNLIMITED, false, r120);
  Sim c = simulate(10, 900, TIME_MS, MOVE_VEL_UNLIMITED, false, r140);
  printf("\nlate nearer retarget (10 -> 900):\n");
  print_sim("-> 400 at 100 ms", a);
  print_sim("-> 500 at 120 ms", b);
  print_sim("-> 600 at 140 ms", c);
  TEST_ASSERT_TRUE(a.arrived && b.arrived && c.arrived);
  TEST_ASSERT_TRUE_MESSAGE(a.overshoot <= DEADBAND, "late retarget to 400 overshot");
  TEST_ASSERT_TRUE_MESSAGE(b.overshoot <= DEADBAND, "late retarget to 500 overshot");
  TEST_ASSERT_TRUE_MESSAGE(c.overshoot <= DEADBAND, "late retarget to 600 overshot");
  TEST_ASSERT_TRUE(a.vref_floor >= VEL_MIN && b.vref_floor >= VEL_MIN && c.vref_floor >= VEL_MIN);
}

void test_accel_guards_the_shortest_time() {
  // T = 1 ms makes T^2 / 4 zero: the clamp, not a division by zero (the wire
  // clamps at MOVE_PROFILE_TIME_MIN_MS; this is for every other caller)
  TEST_ASSERT_EQUAL_UINT32(MOVE_PROFILE_ACCEL_MAX, move_profile_accel(1023, 1));
  TEST_ASSERT_EQUAL_UINT32(MOVE_PROFILE_ACCEL_MAX, move_profile_accel(1, 1));
  TEST_ASSERT_EQUAL_UINT32(MOVE_PROFILE_ACCEL_MAX, move_profile_accel(1023, 0));
}

void test_wire_time_is_clamped_and_mode_checked() {
  TEST_ASSERT_EQUAL_UINT16(MOVE_PROFILE_TIME_MIN_MS, move_profile_time_from_wire(0, 0));
  TEST_ASSERT_EQUAL_UINT16(MOVE_PROFILE_TIME_MIN_MS, move_profile_time_from_wire(0, 49));
  TEST_ASSERT_EQUAL_UINT16(240, move_profile_time_from_wire(0, 240));
  TEST_ASSERT_EQUAL_UINT16(2000, move_profile_time_from_wire(0x07, 0xD0));
  TEST_ASSERT_EQUAL_UINT16(MOVE_PROFILE_TIME_MAX_MS, move_profile_time_from_wire(0xFF, 0xFF));
  TEST_ASSERT_TRUE(move_profile_mode_valid(MOVE_PROFILE_FAST));
  TEST_ASSERT_TRUE(move_profile_mode_valid(MOVE_PROFILE_SMOOTH));
  TEST_ASSERT_FALSE(move_profile_mode_valid(2));
  TEST_ASSERT_FALSE(move_profile_mode_valid(0xFF));
  TEST_ASSERT_EQUAL_HEX16(0x0109, FW_VERSION_MOVE_PROFILE);
  TEST_ASSERT_TRUE(FW_VERSION >= FW_VERSION_MOVE_PROFILE);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_accel_is_four_d_over_t_squared);
  RUN_TEST(test_soft_start_ramps_from_the_floor);
  RUN_TEST(test_falling_move_mirrors_a_rising_one);
  RUN_TEST(test_brakes_to_the_floor_at_the_deadband);
  RUN_TEST(test_wrong_way_reference_restarts_at_the_floor);
  RUN_TEST(test_speed_cap_and_floor_bound_the_reference);
  RUN_TEST(test_aim_continuing_keeps_the_reference);
  RUN_TEST(test_simulated_full_move_is_soft_and_on_time);
  RUN_TEST(test_simulated_durations_are_about_independent_of_distance);
  RUN_TEST(test_simulated_retarget_keeps_the_reference_continuous);
  RUN_TEST(test_simulated_speed_byte_still_caps_a_smooth_move);
  RUN_TEST(test_simulated_move_time_scales_the_move);
  RUN_TEST(test_wire_time_is_clamped_and_mode_checked);
  RUN_TEST(test_simulated_late_nearer_retarget_brakes_in_time);
  RUN_TEST(test_accel_guards_the_shortest_time);
  return UNITY_END();
}
