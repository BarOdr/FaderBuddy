/*
 * Reset record: why the application last started, for REG_RESET_CAUSE.
 *
 * Pure logic with no hardware access, so it runs in the native_test host
 * tests. main.cpp owns one ResetRecord in .noinit RAM, which the C runtime
 * does not clear and a warm reset (watchdog, software reset, brown-out with
 * the supply held) keeps, and which a power-up leaves as noise.
 *
 * Why not RSTCTRL.RSTFR: the I2C bootloader owns the reset vector, reads the
 * flags and writes them back to clear them before it jumps to the
 * application, so the application never sees which reset it was. The
 * bootloader is not field-updatable, so the application keeps its own record:
 *
 *   - while it runs, the marker says RESET_RECORD_RUNNING;
 *   - when it leaves on purpose (REG_ENTER_BOOTLOADER), it writes
 *     RESET_RECORD_HANDOFF just before the software reset;
 *   - at the next start, RUNNING means it stopped without a handoff, which
 *     with the watchdog on is almost always the watchdog.
 *
 * Since firmware 1.11 the record also counts the held-bus releases of this
 * board's TWI (wdt_feed.h): a held bus frees the TWI instead of resetting
 * the chip, so the host reads how often that happened next to the restarts.
 *
 * A power-up matches a 16-bit marker by chance about once in 65536 starts and
 * is then reported as one unexpected restart; the restart count never
 * survives a power-up otherwise.
 */

#pragma once

#include <stdint.h>

#include "shared/i2c_data.h"

#define RESET_RECORD_RUNNING (0x5A17)  // the application is running
#define RESET_RECORD_HANDOFF (0xB00F)  // it left on purpose, for the bootloader

struct ResetRecord {
  uint16_t marker;       // RESET_RECORD_*, or noise after a power-up
  uint8_t cause;         // RESET_CAUSE_* of this start; REG_RESET_CAUSE byte 0
  uint8_t restarts;      // unexpected starts since the last cold one; byte 1
  uint8_t bus_releases;  // held-bus releases of this board's TWI since the last cold start; byte 2
};

// Called once at start, before anything else uses the record: classifies
// this start from what the previous run left and marks the record running.
static inline void reset_record_boot(ResetRecord &r) {
  if (r.marker == RESET_RECORD_RUNNING) {
    r.cause = RESET_CAUSE_UNEXPECTED;
    uint8_t n = r.restarts + 1;
    if (n != 0) r.restarts = n;  // saturates at 255; this form is 6 bytes smaller
  } else if (r.marker == RESET_RECORD_HANDOFF) {
    r.cause = RESET_CAUSE_HANDOFF;
  } else {
    r.cause = RESET_CAUSE_COLD;
    r.restarts = 0;
    r.bus_releases = 0;
  }
  r.marker = RESET_RECORD_RUNNING;
}

// Called just before the application resets itself on purpose.
static inline void reset_record_handoff(ResetRecord &r) {
  r.marker = RESET_RECORD_HANDOFF;
}

// Called when loop() has freed this board's TWI from a held bus
// (wdt_feed.h): counts it, saturating at 255, so a host sees the bus was
// held without any board having reset.
static inline void reset_record_bus_released(ResetRecord &r) {
  uint8_t n = r.bus_releases + 1;
  if (n != 0) r.bus_releases = n;  // saturates at 255; this form is 6 bytes smaller
}
