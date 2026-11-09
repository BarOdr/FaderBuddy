// Host tests of the watchdog feed rule and the held-bus release rule
// (firmware/src/wdt_feed.h).
//
//   pio test -e native_test

#include <unity.h>

#include "wdt_feed.h"

static BusWatch fresh() {
  BusWatch w;
  w.idle_ms = 0;
  w.released = false;
  return w;
}

void setUp() {}
void tearDown() {}

// --- the feed rule -----------------------------------------------------------

void test_interrupts_on_feeds() {
  TEST_ASSERT_TRUE(wdt_may_feed(true));
}

void test_interrupts_off_never_feeds() {
  // I cleared: millis() stops and the TWI interrupt is never served, so the
  // board is not healthy whatever loop() still does.
  TEST_ASSERT_FALSE(wdt_may_feed(false));
}

void test_a_held_bus_does_not_withhold_the_feed() {
  // The change from firmware 1.10: a healthy CPU keeps the watchdog fed
  // whatever the bus does. The bus is freed by the release rule instead, so a
  // held bus no longer resets the chip (and no longer recalibrates touch
  // under a resting finger).
  BusWatch w = fresh();
  bus_release_due(1000, w, true);
  for (uint16_t t = 1001; t < 5000; t++) {
    bus_release_due(t, w, false);
    TEST_ASSERT_TRUE(wdt_may_feed(true));
  }
}

// --- the release rule --------------------------------------------------------
//
// Time is the low 8 bits of millis(): the rule needs only "100 ms since the
// last idle sample", decided once per hold, and loop() passes far more often
// than every 156 ms, so a wider clock would buy nothing on a part with no
// flash to spare.

void test_an_idle_bus_is_never_released() {
  BusWatch w = fresh();
  for (uint16_t t = 0; t < 2000; t += 7) TEST_ASSERT_FALSE(bus_release_due((uint8_t)t, w, true));
}

void test_an_idle_sample_is_remembered() {
  BusWatch w = fresh();
  bus_release_due(200, w, true);
  TEST_ASSERT_EQUAL_UINT8(200, w.idle_ms);
}

void test_a_bus_held_99_ms_is_not_released_yet() {
  BusWatch w = fresh();
  bus_release_due(10, w, true);
  TEST_ASSERT_FALSE(bus_release_due(109, w, false));
  TEST_ASSERT_EQUAL_UINT8(10, w.idle_ms);
}

void test_a_bus_held_100_ms_is_released_once() {
  // One release per hold: repeating it while the line stays low would only
  // count the same hold again, and an outside holder is not ours to free.
  BusWatch w = fresh();
  bus_release_due(10, w, true);
  TEST_ASSERT_TRUE(bus_release_due(110, w, false));
  TEST_ASSERT_FALSE(bus_release_due(111, w, false));
  TEST_ASSERT_FALSE(bus_release_due(200, w, false));
  TEST_ASSERT_FALSE(bus_release_due((uint8_t)9000, w, false));
}

void test_a_bus_idle_again_re_arms_the_release() {
  BusWatch w = fresh();
  bus_release_due(10, w, true);
  TEST_ASSERT_TRUE(bus_release_due(110, w, false));
  TEST_ASSERT_FALSE(bus_release_due(120, w, true));   // freed: both lines high
  TEST_ASSERT_FALSE(bus_release_due(210, w, false));  // held again, 90 ms
  TEST_ASSERT_TRUE(bus_release_due(220, w, false));   // 100 ms since the last idle sample
}

void test_the_8_bit_clock_wraps() {
  // The low 8 bits of millis() wrap every 256 ms.
  BusWatch w = fresh();
  bus_release_due(250, w, true);
  TEST_ASSERT_FALSE(bus_release_due(93, w, false));  // 99 ms later
  TEST_ASSERT_TRUE(bus_release_due(94, w, false));   // 100 ms later
}

void test_an_idle_sample_across_the_wrap_is_recent() {
  BusWatch w = fresh();
  bus_release_due(250, w, true);
  bus_release_due(20, w, true);
  TEST_ASSERT_FALSE(bus_release_due(119, w, false));
  TEST_ASSERT_TRUE(bus_release_due(120, w, false));
}

void test_a_bus_held_longer_than_the_clock_period_is_released_once() {
  // Held for seconds: now - idle_ms wraps back under 100 ms every 256 ms,
  // which must not look like a fresh hold.
  BusWatch w = fresh();
  bus_release_due(10, w, true);
  TEST_ASSERT_TRUE(bus_release_due(110, w, false));
  for (uint32_t t = 111; t < 10 + 40 * 256UL; t += 13) {
    TEST_ASSERT_FALSE(bus_release_due((uint8_t)t, w, false));
  }
}

void test_a_pass_that_skips_the_window_releases_at_the_next_one() {
  // A hypothetical pass slower than 156 ms (none exists today: the longest,
  // the calibration EEPROM save, is under 100 ms) can step over the 100..255
  // ms window of the 8-bit difference; the release is then late by up to one
  // clock period, never lost.
  BusWatch w = fresh();
  bus_release_due(0, w, true);
  TEST_ASSERT_FALSE(bus_release_due((uint8_t)260, w, false));  // reads as 4 ms
  TEST_ASSERT_TRUE(bus_release_due((uint8_t)356, w, false));   // reads as 100 ms
}

void test_a_fast_loop_within_one_millisecond_decides_the_same() {
  // Many passes per millisecond: the release is decided by time, not passes.
  BusWatch w = fresh();
  bus_release_due(40, w, true);
  for (int pass = 0; pass < 100; pass++) TEST_ASSERT_FALSE(bus_release_due(139, w, false));
  TEST_ASSERT_TRUE(bus_release_due(140, w, false));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_interrupts_on_feeds);
  RUN_TEST(test_interrupts_off_never_feeds);
  RUN_TEST(test_a_held_bus_does_not_withhold_the_feed);
  RUN_TEST(test_an_idle_bus_is_never_released);
  RUN_TEST(test_an_idle_sample_is_remembered);
  RUN_TEST(test_a_bus_held_99_ms_is_not_released_yet);
  RUN_TEST(test_a_bus_held_100_ms_is_released_once);
  RUN_TEST(test_a_bus_idle_again_re_arms_the_release);
  RUN_TEST(test_the_8_bit_clock_wraps);
  RUN_TEST(test_an_idle_sample_across_the_wrap_is_recent);
  RUN_TEST(test_a_bus_held_longer_than_the_clock_period_is_released_once);
  RUN_TEST(test_a_pass_that_skips_the_window_releases_at_the_next_one);
  RUN_TEST(test_a_fast_loop_within_one_millisecond_decides_the_same);
  return UNITY_END();
}
