// Host tests of touch injection (REG_DEBUG_TOUCH, firmware/src/touch_inject.h
// and the wire decoding in firmware/src/shared/i2c_data.h).
//
//   pio test -e native_test

#include <unity.h>

#include "shared/i2c_data.h"
#include "touch_inject.h"

static TouchSources t;

void setUp() { t = TOUCH_SOURCES_INIT; }
void tearDown() {}

static uint16_t wire(uint16_t ms) { return debug_touch_ms_from_wire((uint8_t)(ms >> 8), (uint8_t)ms); }

// --- Wire decoding ---------------------------------------------------------

void test_wire_value_in_range_is_used_as_written() {
  TEST_ASSERT_EQUAL_UINT16(1, wire(1));
  TEST_ASSERT_EQUAL_UINT16(300, wire(300));
  TEST_ASSERT_EQUAL_UINT16(DEBUG_TOUCH_MAX_MS, wire(DEBUG_TOUCH_MAX_MS));
  TEST_ASSERT_EQUAL_UINT16(0x01F4, debug_touch_ms_from_wire(0x01, 0xF4));  // big-endian, 500 ms
}

void test_wire_zero_means_release() { TEST_ASSERT_EQUAL_UINT16(0, wire(0)); }

void test_wire_too_long_is_clamped_to_the_bound() {
  // A dead host must not leave a fader "touched" for longer than the bound.
  TEST_ASSERT_EQUAL_UINT16(DEBUG_TOUCH_MAX_MS, wire(DEBUG_TOUCH_MAX_MS + 1));
  TEST_ASSERT_EQUAL_UINT16(DEBUG_TOUCH_MAX_MS, wire(0xFFFF));
}

void test_bound_is_two_seconds_and_below_the_absent_register_value() {
  TEST_ASSERT_EQUAL_UINT16(2000, DEBUG_TOUCH_MAX_MS);
  // Firmware without the register reads back 0xFFFF; a real remaining time never does.
  TEST_ASSERT_TRUE(DEBUG_TOUCH_MAX_MS < FW_VERSION_NONE);
}

// --- Hold and expiry -------------------------------------------------------

void test_nothing_touched_at_start() {
  TEST_ASSERT_FALSE(touch_effective(t));
  TEST_ASSERT_EQUAL_UINT16(0, touch_inject_remaining(t, 1000));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, 1000));
}

void test_write_starts_a_touch_with_a_detect_edge() {
  TEST_ASSERT_EQUAL(TOUCH_EDGE_DETECT, touch_inject_write(t, 1000, 500));
  TEST_ASSERT_TRUE(touch_effective(t));
  TEST_ASSERT_EQUAL_UINT16(500, touch_inject_remaining(t, 1000));
  TEST_ASSERT_EQUAL_UINT16(200, touch_inject_remaining(t, 1300));
}

void test_hold_expires_by_itself_without_refresh() {
  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, 1499));
  TEST_ASSERT_TRUE(touch_effective(t));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 1500));
  TEST_ASSERT_FALSE(touch_effective(t));
  TEST_ASSERT_EQUAL_UINT16(0, touch_inject_remaining(t, 1500));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, 1501));  // released once
}

void test_late_tick_still_ends_the_hold() {
  // A slow loop pass must not keep the fader touched: the first tick after the
  // time is up ends it, however late, and the remaining time never goes negative.
  touch_inject_write(t, 1000, 100);
  TEST_ASSERT_EQUAL_UINT16(0, touch_inject_remaining(t, 5000));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 5000));
}

void test_hold_survives_a_millis_wrap() {
  uint32_t start = 0xFFFFFF00UL;
  touch_inject_write(t, start, 1000);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, start + 999));  // wrapped past 0
  TEST_ASSERT_EQUAL_UINT16(1, touch_inject_remaining(t, start + 999));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, start + 1000));
}

void test_rewrite_restarts_the_time_without_a_new_edge() {
  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_write(t, 1400, 500));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, 1500));
  TEST_ASSERT_EQUAL_UINT16(400, touch_inject_remaining(t, 1500));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 1900));
}

void test_rewrite_can_shorten_the_hold() {
  touch_inject_write(t, 1000, 2000);
  touch_inject_write(t, 1100, 10);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 1110));
}

void test_write_of_zero_releases_now() {
  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_write(t, 1100, 0));
  TEST_ASSERT_FALSE(touch_effective(t));
  TEST_ASSERT_EQUAL_UINT16(0, touch_inject_remaining(t, 1100));
}

void test_write_of_zero_without_a_hold_does_nothing() {
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_write(t, 1000, 0));
  TEST_ASSERT_FALSE(touch_effective(t));
}

void test_hold_longer_than_the_bound_is_cut_to_the_bound() {
  // Defence in depth: even a caller that skipped the wire decoding cannot hold longer.
  touch_inject_write(t, 1000, 0xFFFF);
  TEST_ASSERT_EQUAL_UINT16(DEBUG_TOUCH_MAX_MS, touch_inject_remaining(t, 1000));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 1000 + DEBUG_TOUCH_MAX_MS));
}

// --- OR with the sensed touch ----------------------------------------------

void test_sensor_alone_behaves_as_before() {
  TEST_ASSERT_EQUAL(TOUCH_EDGE_DETECT, touch_sensor_event(t, true));
  TEST_ASSERT_TRUE(touch_effective(t));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_sensor_event(t, false));
  TEST_ASSERT_FALSE(touch_effective(t));
}

void test_injected_release_never_masks_a_real_touch() {
  touch_sensor_event(t, true);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_write(t, 1000, 100));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_tick(t, 1100));  // hold ends, hand still there
  TEST_ASSERT_TRUE(touch_effective(t));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_inject_write(t, 1200, 0));
  TEST_ASSERT_TRUE(touch_effective(t));
}

void test_sensor_release_under_a_hold_keeps_the_touch() {
  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_sensor_event(t, true));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_sensor_event(t, false));
  TEST_ASSERT_TRUE(touch_effective(t));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_RELEASE, touch_inject_tick(t, 1500));
}

void test_touch_already_held_keeps_its_start_time() {
  // The 50 ms touch override counts from the first edge. A second source
  // joining an existing touch must not produce another DETECT, which would
  // restart that count and delay the override.
  TEST_ASSERT_EQUAL(TOUCH_EDGE_DETECT, touch_inject_write(t, 1000, 500));
  TEST_ASSERT_EQUAL(TOUCH_EDGE_NONE, touch_sensor_event(t, true));
}

// --- Haptics and STATE -----------------------------------------------------

void test_haptics_never_driven_by_an_injection_alone() {
  TEST_ASSERT_TRUE(touch_haptics_allowed(t));  // no touch: unchanged behaviour (input movement)
  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_FALSE(touch_haptics_allowed(t));
  touch_sensor_event(t, true);
  TEST_ASSERT_TRUE(touch_haptics_allowed(t));  // a real hand is there too
  touch_sensor_event(t, false);
  TEST_ASSERT_FALSE(touch_haptics_allowed(t));
  touch_inject_tick(t, 1500);
  TEST_ASSERT_TRUE(touch_haptics_allowed(t));
}

void test_state_bits_follow_the_sources() {
  const uint32_t other = 0x0FFFFFFEUL & ~STATE_TOUCH_INJECTED_bm;  // every other used bit set
  TEST_ASSERT_EQUAL_HEX32(other, touch_state_bits(other | STATE_TOUCH_bm | STATE_TOUCH_INJECTED_bm, t));

  touch_inject_write(t, 1000, 500);
  TEST_ASSERT_EQUAL_HEX32(other | STATE_TOUCH_bm | STATE_TOUCH_INJECTED_bm, touch_state_bits(other, t));

  touch_inject_tick(t, 1500);
  touch_sensor_event(t, true);
  TEST_ASSERT_EQUAL_HEX32(other | STATE_TOUCH_bm, touch_state_bits(other, t));
}

void test_injected_bit_is_bit_30_and_clear_of_the_other_fields() {
  TEST_ASSERT_EQUAL_HEX32(0x40000000UL, STATE_TOUCH_INJECTED_bm);
  const uint32_t used = STATE_TOUCH_bm | STATE_MODE_bm | STATE_ACTIVE_LAYER_bm | STATE_POSITION_bm |
                        STATE_POSITION_NONCE_bm | STATE_RAW_ADC_bm | STATE_DOUBLE_TAP_NONCE_bm;
  TEST_ASSERT_EQUAL_HEX32(0, used & STATE_TOUCH_INJECTED_bm);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_wire_value_in_range_is_used_as_written);
  RUN_TEST(test_wire_zero_means_release);
  RUN_TEST(test_wire_too_long_is_clamped_to_the_bound);
  RUN_TEST(test_bound_is_two_seconds_and_below_the_absent_register_value);
  RUN_TEST(test_nothing_touched_at_start);
  RUN_TEST(test_write_starts_a_touch_with_a_detect_edge);
  RUN_TEST(test_hold_expires_by_itself_without_refresh);
  RUN_TEST(test_late_tick_still_ends_the_hold);
  RUN_TEST(test_hold_survives_a_millis_wrap);
  RUN_TEST(test_rewrite_restarts_the_time_without_a_new_edge);
  RUN_TEST(test_rewrite_can_shorten_the_hold);
  RUN_TEST(test_write_of_zero_releases_now);
  RUN_TEST(test_write_of_zero_without_a_hold_does_nothing);
  RUN_TEST(test_hold_longer_than_the_bound_is_cut_to_the_bound);
  RUN_TEST(test_sensor_alone_behaves_as_before);
  RUN_TEST(test_injected_release_never_masks_a_real_touch);
  RUN_TEST(test_sensor_release_under_a_hold_keeps_the_touch);
  RUN_TEST(test_touch_already_held_keeps_its_start_time);
  RUN_TEST(test_haptics_never_driven_by_an_injection_alone);
  RUN_TEST(test_state_bits_follow_the_sources);
  RUN_TEST(test_injected_bit_is_bit_30_and_clear_of_the_other_fields);
  return UNITY_END();
}
