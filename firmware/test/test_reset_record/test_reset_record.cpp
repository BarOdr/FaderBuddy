// Host tests of the reset record behind REG_RESET_CAUSE
// (firmware/src/reset_record.h).
//
//   pio test -e native_test

#include <unity.h>

#include "reset_record.h"

static ResetRecord noise(uint16_t marker, uint8_t restarts) {
  ResetRecord r;
  r.marker = marker;
  r.cause = 0xEE;
  r.restarts = restarts;
  return r;
}

void setUp() {}
void tearDown() {}

void test_power_up_noise_is_a_cold_start_with_no_restarts() {
  ResetRecord r = noise(0x1234, 77);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_COLD, r.cause);
  TEST_ASSERT_EQUAL_UINT8(0, r.restarts);
}

void test_every_start_leaves_the_record_running() {
  ResetRecord r = noise(0x1234, 0);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_HEX16(RESET_RECORD_RUNNING, r.marker);
}

void test_a_start_while_running_is_an_unexpected_restart() {
  // The watchdog: the previous run was still running when the board reset.
  ResetRecord r = noise(0x1234, 0);
  reset_record_boot(r);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_UNEXPECTED, r.cause);
  TEST_ASSERT_EQUAL_UINT8(1, r.restarts);
  TEST_ASSERT_EQUAL_HEX16(RESET_RECORD_RUNNING, r.marker);
}

void test_unexpected_restarts_are_counted() {
  ResetRecord r = noise(0x1234, 0);
  reset_record_boot(r);
  for (uint8_t i = 0; i < 5; i++) reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(5, r.restarts);
}

void test_restart_count_saturates() {
  ResetRecord r = noise(RESET_RECORD_RUNNING, 254);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(255, r.restarts);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(255, r.restarts);
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_UNEXPECTED, r.cause);
}

void test_a_handoff_is_not_an_unexpected_restart_and_keeps_the_count() {
  // REG_ENTER_BOOTLOADER, then the bootloader starts the same image again.
  ResetRecord r = noise(RESET_RECORD_RUNNING, 3);
  reset_record_handoff(r);
  TEST_ASSERT_EQUAL_HEX16(RESET_RECORD_HANDOFF, r.marker);
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_HANDOFF, r.cause);
  TEST_ASSERT_EQUAL_UINT8(3, r.restarts);
  TEST_ASSERT_EQUAL_HEX16(RESET_RECORD_RUNNING, r.marker);
}

void test_a_cold_start_clears_an_earlier_count() {
  ResetRecord r = noise(RESET_RECORD_RUNNING, 4);
  reset_record_boot(r);
  r.marker = 0x0000;  // power-up: RAM is noise again
  reset_record_boot(r);
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_COLD, r.cause);
  TEST_ASSERT_EQUAL_UINT8(0, r.restarts);
}

void test_cause_and_count_are_the_two_wire_bytes_in_order() {
  // main.cpp sends REG_RESET_CAUSE straight from the record.
  ResetRecord r = noise(RESET_RECORD_RUNNING, 6);
  reset_record_boot(r);
  const uint8_t *wire = &r.cause;
  TEST_ASSERT_EQUAL_UINT8(RESET_CAUSE_UNEXPECTED, wire[0]);
  TEST_ASSERT_EQUAL_UINT8(7, wire[1]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_power_up_noise_is_a_cold_start_with_no_restarts);
  RUN_TEST(test_every_start_leaves_the_record_running);
  RUN_TEST(test_a_start_while_running_is_an_unexpected_restart);
  RUN_TEST(test_unexpected_restarts_are_counted);
  RUN_TEST(test_restart_count_saturates);
  RUN_TEST(test_a_handoff_is_not_an_unexpected_restart_and_keeps_the_count);
  RUN_TEST(test_a_cold_start_clears_an_earlier_count);
  RUN_TEST(test_cause_and_count_are_the_two_wire_bytes_in_order);
  return UNITY_END();
}
