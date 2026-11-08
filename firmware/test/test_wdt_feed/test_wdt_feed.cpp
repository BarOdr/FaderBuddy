// Host tests of the watchdog feed rule (firmware/src/wdt_feed.h).
//
//   pio test -e native_test

#include <unity.h>

#include "wdt_feed.h"

void setUp() {}
void tearDown() {}

void test_an_idle_bus_feeds() {
  uint16_t idle = 0;
  TEST_ASSERT_TRUE(wdt_may_feed(5000, idle, true, true));
  TEST_ASSERT_EQUAL_UINT16(5000, idle);
}

void test_a_bus_held_99_ms_still_feeds() {
  uint16_t idle = 0;
  wdt_may_feed(1000, idle, true, true);
  TEST_ASSERT_TRUE(wdt_may_feed(1099, idle, false, true));
  TEST_ASSERT_EQUAL_UINT16(1000, idle);
}

void test_a_bus_held_100_ms_stops_feeding() {
  uint16_t idle = 0;
  wdt_may_feed(1000, idle, true, true);
  TEST_ASSERT_FALSE(wdt_may_feed(1100, idle, false, true));
  TEST_ASSERT_FALSE(wdt_may_feed(1500, idle, false, true));
}

void test_a_released_bus_feeds_again() {
  uint16_t idle = 0;
  wdt_may_feed(1000, idle, true, true);
  TEST_ASSERT_FALSE(wdt_may_feed(1200, idle, false, true));
  TEST_ASSERT_TRUE(wdt_may_feed(1210, idle, true, true));
  TEST_ASSERT_TRUE(wdt_may_feed(1300, idle, false, true));
}

void test_the_16_bit_clock_wraps() {
  // millis() low 16 bits wrap every 65.5 s.
  uint16_t idle = 0;
  wdt_may_feed(65500, idle, true, true);
  TEST_ASSERT_TRUE(wdt_may_feed(63, idle, false, true));    // 99 ms later
  TEST_ASSERT_FALSE(wdt_may_feed(64, idle, false, true));   // 100 ms later
}

void test_an_idle_sample_across_the_wrap_is_recent() {
  uint16_t idle = 0;
  wdt_may_feed(65530, idle, true, true);
  wdt_may_feed(20, idle, true, true);
  TEST_ASSERT_TRUE(wdt_may_feed(119, idle, false, true));
  TEST_ASSERT_FALSE(wdt_may_feed(120, idle, false, true));
}

void test_interrupts_off_never_feeds_even_on_an_idle_bus() {
  uint16_t idle = 0;
  TEST_ASSERT_FALSE(wdt_may_feed(2000, idle, true, false));
}

void test_a_frozen_clock_with_interrupts_off_never_feeds() {
  // I cleared: millis() stops, so the held time stays 0 for ever, and the
  // TWI interrupt is never served. Only the I flag stops the feed.
  uint16_t idle = 0;
  wdt_may_feed(3000, idle, true, true);
  for (int pass = 0; pass < 1000; pass++) {
    TEST_ASSERT_FALSE(wdt_may_feed(3000, idle, pass & 1, false));
  }
}

void test_a_frozen_clock_with_interrupts_on_and_idle_bus_feeds() {
  // Many passes within one millisecond: a fast healthy loop.
  uint16_t idle = 0;
  for (int pass = 0; pass < 100; pass++) {
    TEST_ASSERT_TRUE(wdt_may_feed(4000, idle, true, true));
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_an_idle_bus_feeds);
  RUN_TEST(test_a_bus_held_99_ms_still_feeds);
  RUN_TEST(test_a_bus_held_100_ms_stops_feeding);
  RUN_TEST(test_a_released_bus_feeds_again);
  RUN_TEST(test_the_16_bit_clock_wraps);
  RUN_TEST(test_an_idle_sample_across_the_wrap_is_recent);
  RUN_TEST(test_interrupts_off_never_feeds_even_on_an_idle_bus);
  RUN_TEST(test_a_frozen_clock_with_interrupts_off_never_feeds);
  RUN_TEST(test_a_frozen_clock_with_interrupts_on_and_idle_bus_feeds);
  return UNITY_END();
}
