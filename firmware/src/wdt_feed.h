/*
 * When loop() may feed the hardware watchdog (main.cpp), as pure logic so it
 * runs in the native_test host tests.
 *
 * A pass feeds only when both hold:
 *
 *   - interrupts are globally on. millis() and the TWI interrupt both need
 *     them: a loop left running with I cleared would see a frozen clock (so
 *     the held-bus time below never grows) while its TWI sits in CLKHOLD
 *     holding SCL low, and must not keep the board alive;
 *   - the shared fader bus has been idle (SDA and SCL both high) at some pass
 *     within the last WDT_BUS_STUCK_MS. Traffic leaves both lines high between
 *     bits and transfers far more often than that, so a bus with neither line
 *     released for that long is held: by this board's TWI stretching the
 *     clock, or by a slave transmitter left driving SDA low when the host gave
 *     up mid-read, with the CPU running normally.
 *
 * Time is the low 16 bits of millis(); differences wrap correctly.
 */

#pragma once

#include <stdint.h>

#define WDT_BUS_STUCK_MS (100)

// `idle_ms` keeps when the bus was last seen idle; `lines_idle` is this
// pass's sample of both lines high; `irq_on` is SREG's I flag.
static inline bool wdt_may_feed(uint16_t now, uint16_t &idle_ms, bool lines_idle, bool irq_on) {
  // RED: the rule main.cpp runs in 1.10's first cut, without the I flag.
  (void)irq_on;
  if (lines_idle) idle_ms = now;
  return (uint16_t)(now - idle_ms) < WDT_BUS_STUCK_MS;
}
