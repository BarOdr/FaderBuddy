// Host tests of the haptic engage shaping (firmware/src/haptic_engage.h).
//
//   pio test -e native_test

#include <unity.h>

#include "haptic_engage.h"

static const float DEADBAND = 8.0f;
static const float BAND = 16.0f;
static const int16_t KICK = 127;  // about what the law gives at the deadband edge

static int16_t engage(float error, int16_t drive) { return haptic_engage(error, DEADBAND, BAND, drive); }

void setUp() {}
void tearDown() {}

void test_no_drive_inside_the_deadband() {
  TEST_ASSERT_EQUAL_INT16(0, engage(0.0f, KICK));
  TEST_ASSERT_EQUAL_INT16(0, engage(8.0f, KICK));
  TEST_ASSERT_EQUAL_INT16(0, engage(-8.0f, -KICK));
}

void test_just_past_the_deadband_the_drive_is_small_not_a_kick() {
  // The tremble: a finger nudging the fader across the deadband edge must
  // not switch the motor between nothing and half duty.
  int16_t d = engage(9.0f, KICK);
  TEST_ASSERT_TRUE(d >= 0);
  TEST_ASSERT_TRUE(d <= KICK / 8);
  d = engage(-9.0f, -KICK);
  TEST_ASSERT_TRUE(d <= 0);
  TEST_ASSERT_TRUE(d >= -KICK / 8);
}

void test_drive_grows_with_the_error_across_the_band() {
  int16_t previous = 0;
  for (int e = 9; e <= 24; ++e) {
    int16_t d = engage((float)e, KICK);
    TEST_ASSERT_TRUE(d >= previous);
    // No step bigger than an even share of the band.
    TEST_ASSERT_TRUE(d - previous <= KICK / 16 + 1);
    previous = d;
  }
}

void test_halfway_through_the_band_gives_half_the_drive() {
  TEST_ASSERT_INT16_WITHIN(1, KICK / 2, engage(16.0f, KICK));
  TEST_ASSERT_INT16_WITHIN(1, -KICK / 2, engage(-16.0f, -KICK));
}

void test_full_drive_from_the_end_of_the_band() {
  // The felt detent force (the strength cap) is unchanged further out.
  TEST_ASSERT_EQUAL_INT16(KICK, engage(24.0f, KICK));
  TEST_ASSERT_EQUAL_INT16(171, engage(60.0f, 171));
  TEST_ASSERT_EQUAL_INT16(-171, engage(-60.0f, -171));
}

void test_a_braking_drive_keeps_its_sign() {
  // The law's damping can oppose the error when the carriage arrives fast;
  // the shaping scales it but never flips it.
  TEST_ASSERT_TRUE(engage(16.0f, -40) < 0);
  TEST_ASSERT_TRUE(engage(-16.0f, 40) > 0);
  TEST_ASSERT_EQUAL_INT16(-40, engage(30.0f, -40));
}

void test_zero_band_falls_back_to_the_plain_deadband() {
  TEST_ASSERT_EQUAL_INT16(0, haptic_engage(8.0f, DEADBAND, 0.0f, KICK));
  TEST_ASSERT_EQUAL_INT16(KICK, haptic_engage(8.5f, DEADBAND, 0.0f, KICK));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_no_drive_inside_the_deadband);
  RUN_TEST(test_just_past_the_deadband_the_drive_is_small_not_a_kick);
  RUN_TEST(test_drive_grows_with_the_error_across_the_band);
  RUN_TEST(test_halfway_through_the_band_gives_half_the_drive);
  RUN_TEST(test_full_drive_from_the_end_of_the_band);
  RUN_TEST(test_a_braking_drive_keeps_its_sign);
  RUN_TEST(test_zero_band_falls_back_to_the_plain_deadband);
  return UNITY_END();
}
