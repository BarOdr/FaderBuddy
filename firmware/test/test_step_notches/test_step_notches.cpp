// Host tests of the step notches (HAPTIC_STEP_NOTCHES, firmware/src/haptic_notch.h):
// the window a stepped fader pulls within, touched and untouched, and the
// validation of its config word.
//
//   pio test -e native_test

#include <unity.h>

#include "haptic_engage.h"
#include "haptic_notch.h"

static const int16_t DEADBAND = 8;
static const int16_t KICK = 127;

static uint16_t step_notches(uint8_t count, uint8_t strength, uint8_t width) {
  return (uint16_t)((HAPTIC_STEP_NOTCHES << HAPTIC_MODE_bp) | (count << HAPTIC_DETENT_COUNT_bp) |
                    (strength << HAPTIC_DETENT_STRENGTH_bp) | (width << HAPTIC_NOTCH_WIDTH_bp));
}

static uint16_t config(uint8_t mode, uint8_t count, uint8_t strength) {
  return (uint16_t)((mode << HAPTIC_MODE_bp) | (count << HAPTIC_DETENT_COUNT_bp) |
                    (strength << HAPTIC_DETENT_STRENGTH_bp));
}

// The drive haptic_pull() applies for an error off the nearest step.
static int16_t pull(uint16_t cfg, bool touched, int16_t error, int16_t drive) {
  return haptic_shape(error, DEADBAND, haptic_step_notch_half_width(cfg, touched), drive);
}

void setUp() {}
void tearDown() {}

void test_mode_is_four_and_the_width_field_sits_above_the_strength() {
  TEST_ASSERT_EQUAL_UINT8(4, HAPTIC_STEP_NOTCHES);
  TEST_ASSERT_EQUAL_UINT16(0x3C00, HAPTIC_NOTCH_WIDTH_bm);
  TEST_ASSERT_EQUAL_UINT16(0, HAPTIC_NOTCH_WIDTH_bm & (HAPTIC_MODE_bm | HAPTIC_DETENT_COUNT_bm | HAPTIC_DETENT_STRENGTH_bm));
}

void test_touched_window_is_the_width_field_in_eight_count_units() {
  TEST_ASSERT_EQUAL_INT16(6 * 8, haptic_step_notch_half_width(step_notches(3, 3, 6), true));
  TEST_ASSERT_EQUAL_INT16(1 * 8, haptic_step_notch_half_width(step_notches(10, 7, 1), true));
  TEST_ASSERT_EQUAL_INT16(15 * 8, haptic_step_notch_half_width(step_notches(2, 0, 15), true));
}

void test_untouched_pull_reaches_the_whole_travel() {
  // A released fader settles on the nearest step from anywhere, as detents.
  TEST_ASSERT_EQUAL_INT16(INT16_MAX, haptic_step_notch_half_width(step_notches(3, 3, 6), false));
}

void test_touched_fader_is_free_between_steps() {
  const uint16_t cfg = step_notches(3, 3, 4);  // 32 counts either side
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, true, 32, KICK));
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, true, -200, -KICK));
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, true, 400, 254));
}

void test_touched_fader_feels_a_pull_inside_the_notch() {
  const uint16_t cfg = step_notches(3, 3, 6);  // full from 24 to 32 counts off the step
  TEST_ASSERT_EQUAL_INT16(KICK, pull(cfg, true, 28, KICK));
  TEST_ASSERT_EQUAL_INT16(-KICK, pull(cfg, true, -28, -KICK));
}

void test_released_fader_between_steps_is_pulled_to_the_step() {
  // Untouched, the same far error that is free under the hand pulls fully.
  const uint16_t cfg = step_notches(3, 3, 4);
  TEST_ASSERT_EQUAL_INT16(KICK, pull(cfg, false, 200, KICK));
  TEST_ASSERT_EQUAL_INT16(-KICK, pull(cfg, false, -200, -KICK));
  // ...and it still coasts at rest on the step (the deadband).
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, false, 0, KICK));
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, false, DEADBAND, KICK));
}

void test_touched_window_has_no_step_anywhere() {
  const uint16_t cfg = step_notches(5, 3, 6);
  for (int side = -1; side <= 1; side += 2) {
    int16_t previous = 0;
    for (int e = 6 * 8 + 10; e >= 0; --e) {
      int16_t d = pull(cfg, true, (int16_t)(side * e), (int16_t)(side * KICK));
      int16_t step = (int16_t)(d > previous ? d - previous : previous - d);
      TEST_ASSERT_TRUE(step <= KICK / 16 + 1);
      previous = d;
    }
  }
}

void test_a_touch_glitch_shorter_than_the_settle_time_keeps_the_notch_window() {
  // The hand rests between two steps and the PTC drops the touch bit for a
  // few ticks: the window stays the notch, so the hand feels no kick.
  TEST_ASSERT_TRUE(haptic_step_notch_held(false, 0));
  TEST_ASSERT_TRUE(haptic_step_notch_held(false, 10));
  TEST_ASSERT_TRUE(haptic_step_notch_held(false, HAPTIC_STEP_RELEASE_SETTLE_MS));
  const uint16_t cfg = step_notches(3, 3, 4);
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, haptic_step_notch_held(false, 20), 200, KICK));
}

void test_a_release_held_past_the_settle_time_pulls_to_the_step() {
  TEST_ASSERT_EQUAL_UINT32(50, HAPTIC_STEP_RELEASE_SETTLE_MS);
  TEST_ASSERT_FALSE(haptic_step_notch_held(false, HAPTIC_STEP_RELEASE_SETTLE_MS + 1));
  TEST_ASSERT_FALSE(haptic_step_notch_held(false, 1000));
  const uint16_t cfg = step_notches(3, 3, 4);
  TEST_ASSERT_EQUAL_INT16(KICK, pull(cfg, haptic_step_notch_held(false, 51), 200, KICK));
}

void test_a_held_touch_keeps_the_notch_window_however_long() {
  TEST_ASSERT_TRUE(haptic_step_notch_held(true, 0));
  TEST_ASSERT_TRUE(haptic_step_notch_held(true, 100000));
}

void test_step_notches_config_needs_two_to_ten_steps_and_a_width() {
  TEST_ASSERT_TRUE(haptic_config_valid(step_notches(2, 3, 6)));
  TEST_ASSERT_TRUE(haptic_config_valid(step_notches(10, 7, 15)));
  TEST_ASSERT_TRUE(haptic_config_valid(step_notches(5, 0, 1)));
  TEST_ASSERT_FALSE(haptic_config_valid(step_notches(0, 3, 6)));
  TEST_ASSERT_FALSE(haptic_config_valid(step_notches(1, 3, 6)));   // one step: the centre notch's job
  TEST_ASSERT_FALSE(haptic_config_valid(step_notches(11, 3, 6)));
  TEST_ASSERT_FALSE(haptic_config_valid(step_notches(3, 3, 0)));   // no window
}

void test_other_modes_still_validate_as_before() {
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_DETENTS, 4, 3)));
  TEST_ASSERT_FALSE(haptic_config_valid(config(HAPTIC_DETENTS, 0, 3)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_CENTRE_NOTCH, 6, 1)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_NO_HAPTICS, 0, 0)));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_mode_is_four_and_the_width_field_sits_above_the_strength);
  RUN_TEST(test_touched_window_is_the_width_field_in_eight_count_units);
  RUN_TEST(test_untouched_pull_reaches_the_whole_travel);
  RUN_TEST(test_touched_fader_is_free_between_steps);
  RUN_TEST(test_touched_fader_feels_a_pull_inside_the_notch);
  RUN_TEST(test_released_fader_between_steps_is_pulled_to_the_step);
  RUN_TEST(test_touched_window_has_no_step_anywhere);
  RUN_TEST(test_a_touch_glitch_shorter_than_the_settle_time_keeps_the_notch_window);
  RUN_TEST(test_a_release_held_past_the_settle_time_pulls_to_the_step);
  RUN_TEST(test_a_held_touch_keeps_the_notch_window_however_long);
  RUN_TEST(test_step_notches_config_needs_two_to_ten_steps_and_a_width);
  RUN_TEST(test_other_modes_still_validate_as_before);
  return UNITY_END();
}
