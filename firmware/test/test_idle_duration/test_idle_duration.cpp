// Host tests of the REG_IDLE_DURATION write decoding (firmware/src/shared/i2c_data.h).
//
//   pio test -e native_test

#include <unity.h>

#include "shared/i2c_data.h"

static uint16_t write(uint16_t ms) { return idle_duration_from_wire((uint8_t)(ms >> 8), (uint8_t)ms); }

void setUp() {}
void tearDown() {}

void test_value_in_range_is_used_as_written() {
  TEST_ASSERT_EQUAL_UINT16(300, write(300));
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_DEFAULT_MS, write(IDLE_DURATION_DEFAULT_MS));
}

void test_bytes_are_big_endian() {
  TEST_ASSERT_EQUAL_UINT16(0x0258, idle_duration_from_wire(0x02, 0x58));  // 600 ms
}

void test_range_ends_are_kept() {
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MIN_MS, write(IDLE_DURATION_MIN_MS));
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MAX_MS, write(IDLE_DURATION_MAX_MS));
}

void test_too_short_is_clamped_to_minimum() {
  // Zero would let the fader drop to idle under a moving hand: the safety floor.
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MIN_MS, write(0));
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MIN_MS, write(IDLE_DURATION_MIN_MS - 1));
}

void test_too_long_is_clamped_to_maximum() {
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MAX_MS, write(IDLE_DURATION_MAX_MS + 1));
  TEST_ASSERT_EQUAL_UINT16(IDLE_DURATION_MAX_MS, write(0xFFFF));
}

void test_default_is_inside_the_range() {
  TEST_ASSERT_TRUE(IDLE_DURATION_DEFAULT_MS >= IDLE_DURATION_MIN_MS);
  TEST_ASSERT_TRUE(IDLE_DURATION_DEFAULT_MS <= IDLE_DURATION_MAX_MS);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_value_in_range_is_used_as_written);
  RUN_TEST(test_bytes_are_big_endian);
  RUN_TEST(test_range_ends_are_kept);
  RUN_TEST(test_too_short_is_clamped_to_minimum);
  RUN_TEST(test_too_long_is_clamped_to_maximum);
  RUN_TEST(test_default_is_inside_the_range);
  return UNITY_END();
}
