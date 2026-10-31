// Host tests of the placed notches (HAPTIC_PLACED_NOTCHES,
// firmware/src/haptic_notch.h): the notch a fader is pulled to, the window
// that exists only under the hand, and the validation of the config word.
//
//   pio test -e native_test

#include <unity.h>

#include "haptic_engage.h"
#include "haptic_notch.h"

static const int16_t DEADBAND = 8;
static const int16_t KICK = 127;

static uint16_t placed(uint8_t count, uint8_t strength, uint8_t width) {
  return (uint16_t)((HAPTIC_PLACED_NOTCHES << HAPTIC_MODE_bp) | (count << HAPTIC_DETENT_COUNT_bp) |
                    (strength << HAPTIC_DETENT_STRENGTH_bp) | (width << HAPTIC_NOTCH_WIDTH_bp));
}

// The drive haptic_pull() would apply for an error off the nearest notch.
static int16_t pull(uint16_t cfg, bool touched, int16_t error, int16_t drive) {
  return haptic_shape(error, DEADBAND, haptic_placed_notch_half_width(cfg, touched), drive);
}

static uint8_t nearest(const uint8_t *positions, uint8_t count, uint8_t position) {
  return haptic_placed_notch_nearest(positions, count, position);
}

void setUp() {}
void tearDown() {}

void test_mode_is_five_and_the_register_follows_idle_duration() {
  TEST_ASSERT_EQUAL_UINT8(5, HAPTIC_PLACED_NOTCHES);
  TEST_ASSERT_EQUAL_UINT8(0x14, REG_LAYER_NOTCHES);
  TEST_ASSERT_EQUAL_UINT8(4, HAPTIC_PLACED_NOTCHES_MAX);
}

void test_config_word_fits_four_notches_and_every_width() {
  TEST_ASSERT_EQUAL_UINT16(0, placed(4, 7, 15) & ~(HAPTIC_MODE_bm | HAPTIC_DETENT_COUNT_bm |
                                                  HAPTIC_DETENT_STRENGTH_bm | HAPTIC_NOTCH_WIDTH_bm));
}

void test_valid_with_one_to_four_notches_and_a_width() {
  TEST_ASSERT_TRUE(haptic_config_valid(placed(1, 1, 6)));
  TEST_ASSERT_TRUE(haptic_config_valid(placed(4, 7, 15)));
  TEST_ASSERT_TRUE(haptic_config_valid(placed(2, 0, 1)));
}

void test_rejects_no_notch_too_many_notches_and_no_width() {
  TEST_ASSERT_FALSE(haptic_config_valid(placed(0, 1, 6)));
  TEST_ASSERT_FALSE(haptic_config_valid(placed(5, 1, 6)));
  TEST_ASSERT_FALSE(haptic_config_valid(placed(15, 1, 6)));
  TEST_ASSERT_FALSE(haptic_config_valid(placed(1, 1, 0)));
}

void test_touched_window_is_the_width_field_in_eight_count_units() {
  TEST_ASSERT_EQUAL_INT16(6 * 8, haptic_placed_notch_half_width(placed(1, 1, 6), true));
  TEST_ASSERT_EQUAL_INT16(1 * 8, haptic_placed_notch_half_width(placed(4, 7, 1), true));
  TEST_ASSERT_EQUAL_INT16(15 * 8, haptic_placed_notch_half_width(placed(2, 0, 15), true));
}

void test_released_fader_has_no_window_at_all() {
  // The point of the mode: 0 tells motor_update() to coast. A step notch's
  // whole-travel window here would pull a released fader and change the value.
  TEST_ASSERT_EQUAL_INT16(0, haptic_placed_notch_half_width(placed(1, 7, 15), false));
  TEST_ASSERT_EQUAL_INT16(0, haptic_placed_notch_half_width(placed(4, 1, 1), false));
}

void test_the_pull_ends_at_the_first_low_reading_not_at_the_filtered_release() {
  // STATE_TOUCH stays set for two more acquisitions (40 ms and more) after the
  // reading falls below the release threshold: a fader held on a notch's slope
  // would slide under the motor with no hand on it.
  TEST_ASSERT_FALSE(haptic_placed_notch_under_hand(true, 49, 50));
  TEST_ASSERT_FALSE(haptic_placed_notch_under_hand(true, -3, 50));
}

void test_a_reading_at_the_threshold_still_pulls_under_the_hand() {
  TEST_ASSERT_TRUE(haptic_placed_notch_under_hand(true, 50, 50));
  TEST_ASSERT_TRUE(haptic_placed_notch_under_hand(true, 400, 50));
}

void test_no_reading_pulls_without_the_touch_bit() {
  TEST_ASSERT_FALSE(haptic_placed_notch_under_hand(false, 400, 50));
}

void test_released_fader_feels_nothing_at_any_distance() {
  const uint16_t cfg = placed(2, 7, 15);
  for (int16_t error = -1000; error <= 1000; error += 7) {
    TEST_ASSERT_EQUAL_INT16(0, pull(cfg, false, error, error < 0 ? -254 : 254));
  }
}

void test_touched_fader_feels_a_pull_inside_the_notch() {
  const uint16_t cfg = placed(1, 1, 6);  // full from 24 to 32 counts off the notch
  TEST_ASSERT_EQUAL_INT16(KICK, pull(cfg, true, 28, KICK));
  TEST_ASSERT_EQUAL_INT16(-KICK, pull(cfg, true, -28, -KICK));
}

void test_touched_fader_is_free_outside_the_notch() {
  const uint16_t cfg = placed(1, 1, 6);  // 48 counts either side
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, true, 48, KICK));
  TEST_ASSERT_EQUAL_INT16(0, pull(cfg, true, -300, -KICK));
}

void test_single_notch_is_the_target_from_anywhere() {
  // The window, not the target, limits where it pulls.
  const uint8_t one[] = {64};
  TEST_ASSERT_EQUAL_UINT8(64, nearest(one, 1, 0));
  TEST_ASSERT_EQUAL_UINT8(64, nearest(one, 1, 255));
}

void test_nearest_of_several_wins_in_any_order() {
  const uint8_t unordered[] = {200, 20, 120, 60};
  TEST_ASSERT_EQUAL_UINT8(20, nearest(unordered, 4, 25));
  TEST_ASSERT_EQUAL_UINT8(60, nearest(unordered, 4, 55));
  TEST_ASSERT_EQUAL_UINT8(120, nearest(unordered, 4, 125));
  TEST_ASSERT_EQUAL_UINT8(200, nearest(unordered, 4, 255));
}

void test_only_the_counted_positions_are_used() {
  const uint8_t positions[] = {20, 200, 200, 200};
  TEST_ASSERT_EQUAL_UINT8(20, nearest(positions, 1, 250));
}

void test_the_ends_of_the_scale_do_not_wrap() {
  const uint8_t both[] = {0, 255};
  TEST_ASSERT_EQUAL_UINT8(0, nearest(both, 2, 0));
  TEST_ASSERT_EQUAL_UINT8(255, nearest(both, 2, 255));
  const uint8_t top[] = {0};
  TEST_ASSERT_EQUAL_UINT8(0, nearest(top, 1, 255));  // the largest distance there is
  const uint8_t bottom[] = {255};
  TEST_ASSERT_EQUAL_UINT8(255, nearest(bottom, 1, 0));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_mode_is_five_and_the_register_follows_idle_duration);
  RUN_TEST(test_config_word_fits_four_notches_and_every_width);
  RUN_TEST(test_valid_with_one_to_four_notches_and_a_width);
  RUN_TEST(test_rejects_no_notch_too_many_notches_and_no_width);
  RUN_TEST(test_touched_window_is_the_width_field_in_eight_count_units);
  RUN_TEST(test_released_fader_has_no_window_at_all);
  RUN_TEST(test_the_pull_ends_at_the_first_low_reading_not_at_the_filtered_release);
  RUN_TEST(test_a_reading_at_the_threshold_still_pulls_under_the_hand);
  RUN_TEST(test_no_reading_pulls_without_the_touch_bit);
  RUN_TEST(test_released_fader_feels_nothing_at_any_distance);
  RUN_TEST(test_touched_fader_feels_a_pull_inside_the_notch);
  RUN_TEST(test_touched_fader_is_free_outside_the_notch);
  RUN_TEST(test_single_notch_is_the_target_from_anywhere);
  RUN_TEST(test_nearest_of_several_wins_in_any_order);
  RUN_TEST(test_only_the_counted_positions_are_used);
  RUN_TEST(test_the_ends_of_the_scale_do_not_wrap);
  return UNITY_END();
}
