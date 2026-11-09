/*
 * Two rules loop() applies at the end of every pass (main.cpp), as pure logic
 * so they run in the native_test host tests:
 *
 *   - wdt_may_feed: whether this pass feeds the hardware watchdog. Only a
 *     CPU that is not healthy withholds the feed, and the one sign of that
 *     loop() can test is interrupts globally off: millis() and the TWI
 *     interrupt both need them, so a loop left running with I cleared sits
 *     with a frozen clock and a TWI nobody serves, and must not keep the
 *     board alive. The bus state plays no part since firmware 1.11: in 1.10
 *     a held bus also withheld the feed, so every board on the bus reset
 *     about 1 s later, recalibrated touch to whatever rested on its cap and
 *     lost its RAM state, for a condition a healthy CPU can clear itself.
 *
 *   - bus_release_due: whether this pass frees this board's TWI (main.cpp
 *     twi_release(): the slave is disabled and enabled again, which drops
 *     its hold on both lines and restarts its state machine). Due when the
 *     shared fader bus has not been idle (SDA and SCL both high) at any pass
 *     for WDT_BUS_STUCK_MS. Traffic leaves both lines high between bits and
 *     transfers far more often than that, so a bus with neither line released
 *     for that long is held: by this board's TWI stretching the clock, or by
 *     a slave transmitter left driving SDA low when the host gave up
 *     mid-read. The host must have given up before the release: its
 *     transaction timeout has to stay under WDT_BUS_STUCK_MS (the Surface's
 *     fader bus uses about 50 ms), or a CLKHOLD release would let it finish
 *     clocking a read with SDA floating and take 0xFF bytes as data.
 *     Once per hold: after a release the bus must be seen idle
 *     before another is due, so a line held by someone else (a shorted line,
 *     a stuck host) is not "released" ten times a second and counted as ten
 *     holds, and a hold longer than the 16-bit clock period cannot look like
 *     a fresh one.
 *
 * Time is the low 8 bits of millis(): the rule only has to notice 100 ms
 * since the last idle sample, once per hold, and loop() passes every few
 * milliseconds, so the 8-bit difference cannot step over its 100..255 window
 * unnoticed except on a pass slower than 156 ms, which only delays the
 * release by up to one 256 ms period. A wider clock costs flash the part
 * does not have.
 */

#pragma once

#include <stdint.h>

#define WDT_BUS_STUCK_MS (100)

// `irq_on` is SREG's I flag at the end of the pass.
static inline bool wdt_may_feed(bool irq_on) {
  return irq_on;
}

struct BusWatch {
  uint8_t idle_ms;  // low 8 bits of millis() when the bus was last seen idle
  bool released;    // the current hold has had its release
};

// `now` is the low 8 bits of millis(); `lines_idle` is this pass's sample of
// both lines high.
static inline bool bus_release_due(uint8_t now, BusWatch &w, bool lines_idle) {
  if (lines_idle) {
    w.idle_ms = now;
    w.released = false;
    return false;
  }
  if (w.released || (uint8_t)(now - w.idle_ms) < WDT_BUS_STUCK_MS) return false;
  w.released = true;
  return true;
}
