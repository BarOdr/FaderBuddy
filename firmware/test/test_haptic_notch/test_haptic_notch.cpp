// Host tests of the centre notch window and the haptic config validation
// (firmware/src/haptic_notch.h, combined with firmware/src/haptic_engage.h as
// haptic_pull() combines them).
//
//   pio test -e native_test

#include <unity.h>

#include "haptic_engage.h"
#include "haptic_notch.h"

static const int16_t DEADBAND = 8;
static const int16_t KICK = 127;     // about what the law gives at the deadband edge
static const int16_t HALF = 6 * 8;   // a 6-unit notch: 48 counts either side of centre

// The drive haptic_pull() applies in the notch: the inner soft engage, then
// the window's fade out toward its outer edge.
static int16_t notch(int16_t error, int16_t drive) {
  return haptic_window(error, HALF, haptic_engage(error, DEADBAND, drive));
}

static uint16_t config(uint8_t mode, uint8_t count, uint8_t strength) {
  return (uint16_t)((mode << HAPTIC_MODE_bp) | (count << HAPTIC_DETENT_COUNT_bp) |
                    (strength << HAPTIC_DETENT_STRENGTH_bp));
}

void setUp() {}
void tearDown() {}

// --- the window alone ---

void test_window_is_zero_at_and_outside_its_edge() {
  // Outside the window the fader is free, as between magnet ends.
  TEST_ASSERT_EQUAL_INT16(0, haptic_window(HALF, HALF, KICK));
  TEST_ASSERT_EQUAL_INT16(0, haptic_window(-HALF, HALF, -KICK));
  TEST_ASSERT_EQUAL_INT16(0, haptic_window(HALF + 1, HALF, KICK));
  TEST_ASSERT_EQUAL_INT16(0, haptic_window(400, HALF, 254));
  TEST_ASSERT_EQUAL_INT16(0, haptic_window(-400, HALF, -254));
}

void test_window_passes_the_drive_unchanged_inside_its_band() {
  // Further in than the fade, the window does not touch the drive.
  TEST_ASSERT_EQUAL_INT16(KICK, haptic_window(0, HALF, KICK));
  TEST_ASSERT_EQUAL_INT16(KICK, haptic_window(HALF - HAPTIC_NOTCH_FADE, HALF, KICK));
  TEST_ASSERT_EQUAL_INT16(-KICK, haptic_window(-(HALF - HAPTIC_NOTCH_FADE), HALF, -KICK));
}

void test_window_edge_is_faded_not_a_relay() {
  // Just inside the edge the drive is small: entering the window must not
  // step from coast to the capped drive.
  int16_t d = haptic_window(HALF - 1, HALF, KICK);
  TEST_ASSERT_TRUE(d >= 0);
  TEST_ASSERT_TRUE(d <= KICK / 8);
  d = haptic_window(-(HALF - 1), HALF, -KICK);
  TEST_ASSERT_TRUE(d <= 0);
  TEST_ASSERT_TRUE(d >= -KICK / 8);
  TEST_ASSERT_INT16_WITHIN(1, KICK / 2, haptic_window(HALF - HAPTIC_NOTCH_FADE / 2, HALF, KICK));
}

void test_window_fade_is_monotonic_toward_the_edge() {
  int16_t previous = KICK;
  for (int e = HALF - HAPTIC_NOTCH_FADE; e <= HALF + 2; ++e) {
    int16_t d = haptic_window((int16_t)e, HALF, KICK);
    TEST_ASSERT_TRUE(d <= previous);
    TEST_ASSERT_TRUE(previous - d <= KICK / HAPTIC_NOTCH_FADE + 1);
    previous = d;
  }
}

void test_window_keeps_the_sign_of_the_drive() {
  // The law's damping can oppose the error; the window scales, never flips.
  TEST_ASSERT_TRUE(haptic_window(HALF - 4, HALF, -40) < 0);
  TEST_ASSERT_TRUE(haptic_window(-(HALF - 4), HALF, 40) > 0);
  TEST_ASSERT_EQUAL_INT16(-40, haptic_window(10, HALF, -40));
}

void test_window_handles_the_widest_notch_and_full_drive() {
  // 15 units = 120 counts at drive 254: no overflow in the scaling.
  const int16_t wide = 15 * 8;
  TEST_ASSERT_EQUAL_INT16(254, haptic_window(0, wide, 254));
  int16_t d = haptic_window(wide - 8, wide, 254);
  TEST_ASSERT_INT16_WITHIN(1, 127, d);
  TEST_ASSERT_INT16_WITHIN(1, -127, haptic_window(-(wide - 8), wide, -254));
}

// --- the notch as haptic_pull applies it ---

void test_notch_is_free_at_centre_and_outside() {
  TEST_ASSERT_EQUAL_INT16(0, notch(0, KICK));
  TEST_ASSERT_EQUAL_INT16(0, notch(DEADBAND, KICK));
  TEST_ASSERT_EQUAL_INT16(0, notch(HALF, KICK));
  TEST_ASSERT_EQUAL_INT16(0, notch(-200, -KICK));
}

void test_notch_has_no_step_anywhere_across_the_window() {
  // Sweep from far outside to the centre on both sides: the drive rises and
  // falls in small steps only, so neither edge kicks.
  for (int side = -1; side <= 1; side += 2) {
    int16_t previous = 0;
    for (int e = HALF + 10; e >= 0; --e) {
      int16_t d = notch((int16_t)(side * e), (int16_t)(side * KICK));
      int16_t step = (int16_t)(d > previous ? d - previous : previous - d);
      TEST_ASSERT_TRUE(step <= KICK / 16 + 1);
      TEST_ASSERT_TRUE(side > 0 ? d >= 0 : d <= 0);
      previous = d;
    }
  }
}

void test_notch_reaches_full_drive_between_the_fades() {
  // A 6-unit notch: engaged by deadband + 16 = 24, fade from 48 - 16 = 32.
  TEST_ASSERT_EQUAL_INT16(KICK, notch(28, KICK));
  TEST_ASSERT_EQUAL_INT16(-KICK, notch(-28, -KICK));
}

// --- haptic_shape: the single scaling step haptic_pull() applies ---

void test_shape_equals_engage_then_window_for_a_six_unit_notch() {
  // The bands do not overlap (engaged by 24, fade from 32), so the one-step
  // shape gives exactly the two fades applied one after the other.
  for (int e = -HALF - 10; e <= HALF + 10; ++e) {
    TEST_ASSERT_EQUAL_INT16(notch((int16_t)e, KICK), haptic_shape((int16_t)e, DEADBAND, HALF, KICK));
    TEST_ASSERT_EQUAL_INT16(notch((int16_t)e, -KICK), haptic_shape((int16_t)e, DEADBAND, HALF, -KICK));
  }
}

void test_shape_of_a_narrow_notch_has_no_step_and_is_zero_at_both_ends() {
  // A 3-unit notch (24 counts): the engage and the window fades overlap.
  const int16_t narrow = 3 * 8;
  TEST_ASSERT_EQUAL_INT16(0, haptic_shape(DEADBAND, DEADBAND, narrow, KICK));
  TEST_ASSERT_EQUAL_INT16(0, haptic_shape(narrow, DEADBAND, narrow, KICK));
  int16_t previous = 0;
  int16_t peak = 0;
  for (int e = narrow + 4; e >= 0; --e) {
    int16_t d = haptic_shape((int16_t)e, DEADBAND, narrow, KICK);
    int16_t step = (int16_t)(d > previous ? d - previous : previous - d);
    TEST_ASSERT_TRUE(step <= KICK / 16 + 1);
    TEST_ASSERT_TRUE(d >= 0);
    TEST_ASSERT_TRUE(d < KICK);
    if (d > peak) peak = d;
    previous = d;
  }
  TEST_ASSERT_TRUE(peak > 0);
  TEST_ASSERT_TRUE(haptic_shape(-16, DEADBAND, narrow, -KICK) < 0);
}

// --- config ---

void test_notch_half_width_is_the_count_field_in_units_of_eight() {
  TEST_ASSERT_EQUAL_INT16(8, haptic_notch_half_width(config(HAPTIC_CENTRE_NOTCH, 1, 0)));
  TEST_ASSERT_EQUAL_INT16(48, haptic_notch_half_width(config(HAPTIC_CENTRE_NOTCH, 6, 3)));
  TEST_ASSERT_EQUAL_INT16(120, haptic_notch_half_width(config(HAPTIC_CENTRE_NOTCH, 15, 7)));
}

void test_notch_width_zero_is_rejected() {
  TEST_ASSERT_FALSE(haptic_config_valid(config(HAPTIC_CENTRE_NOTCH, 0, 0)));
  TEST_ASSERT_FALSE(haptic_config_valid(config(HAPTIC_CENTRE_NOTCH, 0, 7)));
}

void test_notch_widths_one_to_fifteen_are_accepted() {
  for (uint8_t w = 1; w <= 15; ++w) {
    TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_CENTRE_NOTCH, w, 2)));
  }
}

void test_other_modes_validate_as_before() {
  // Detents: count 1-10 only. Other modes: the count field is not checked.
  TEST_ASSERT_FALSE(haptic_config_valid(config(HAPTIC_DETENTS, 0, 0)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_DETENTS, 1, 0)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_DETENTS, 10, 0)));
  TEST_ASSERT_FALSE(haptic_config_valid(config(HAPTIC_DETENTS, 11, 0)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_NO_HAPTICS, 0, 0)));
  TEST_ASSERT_TRUE(haptic_config_valid(config(HAPTIC_SMOOTH_WITH_MAGNET_ENDS, 0, 5)));
}

void test_mode_value_and_fade_length() {
  // The wire value and the tuning the tests above assume.
  TEST_ASSERT_EQUAL_INT(3, HAPTIC_CENTRE_NOTCH);
  TEST_ASSERT_EQUAL_INT(16, HAPTIC_NOTCH_FADE);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_window_is_zero_at_and_outside_its_edge);
  RUN_TEST(test_window_passes_the_drive_unchanged_inside_its_band);
  RUN_TEST(test_window_edge_is_faded_not_a_relay);
  RUN_TEST(test_window_fade_is_monotonic_toward_the_edge);
  RUN_TEST(test_window_keeps_the_sign_of_the_drive);
  RUN_TEST(test_window_handles_the_widest_notch_and_full_drive);
  RUN_TEST(test_notch_is_free_at_centre_and_outside);
  RUN_TEST(test_notch_has_no_step_anywhere_across_the_window);
  RUN_TEST(test_notch_reaches_full_drive_between_the_fades);
  RUN_TEST(test_shape_equals_engage_then_window_for_a_six_unit_notch);
  RUN_TEST(test_shape_of_a_narrow_notch_has_no_step_and_is_zero_at_both_ends);
  RUN_TEST(test_notch_half_width_is_the_count_field_in_units_of_eight);
  RUN_TEST(test_notch_width_zero_is_rejected);
  RUN_TEST(test_notch_widths_one_to_fifteen_are_accepted);
  RUN_TEST(test_other_modes_validate_as_before);
  RUN_TEST(test_mode_value_and_fade_length);
  return UNITY_END();
}
