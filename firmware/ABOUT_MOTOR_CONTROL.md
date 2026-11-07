# About: motor control and closed-loop tuning

Notes for whoever next touches the movement code. Covers the plant model, why
the control law is shaped the way it is, what each constant does, and how to
re-measure everything on hardware.

Three files: `src/motor_control.h` holds the drive primitives and every `MOVE_*`
constant; `src/main.cpp` runs the control law and owns the mode state machine;
`src/motor_cal.cpp` measures the plant during self-calibration and re-derives
the gains from it.

## Units

Two different position scales appear in the code and in measurements:

- **ADC counts** (0-1023). What the firmware works in: `input_calib_min/max`,
  `target_adc`, all `MOVE_*` gains. The filtered position `input_pos_q8` and
  the error the law sees are ADC counts in Q8 (counts x 256), see "Fixed
  point" below.
- **raw** (0-2046). What the `STATE` register reports, because ADC1 runs with
  2x accumulation. `raw = 2 * ADC`.

Full travel is about 1956 raw over 60 mm, so **1 mm is about 33 raw / 16 ADC counts**.

"Duty" always means the 0-254 value handed to `motor_set()`, matching the TCA0
split-mode period.

## The plant

Measured on a jig fader. Numbers vary between units; the shapes do not.

**It is a velocity source with a friction deadband, not a force source.**
Below a breakaway duty nothing moves at all. Above it, speed is roughly
proportional to duty. There is no useful "push gently" region: at breakaway the
speed jumps discontinuously to ~900 raw/s (Stribeck effect). This single fact
drives almost every design decision below.

**Breakaway duty depends strongly on PWM decay mode, not on frequency:**

| drive scheme            | dir A (rising ADC) | dir B (falling) |
|-------------------------|--------------------|-----------------|
| fast decay @ 306 Hz     | ~55                | ~75             |
| fast decay @ 19.6 kHz   | ~190               | ~200            |
| slow decay @ 19.6 kHz   | ~70                | ~90             |
| slow decay @ 39.2 kHz   | ~68                | ~88             |

Direction asymmetry (~20 duty counts) is consistent and real; the two
directions get separate feedforward constants.

**Velocity above breakaway** rises ~58 raw/s per duty count going up, ~94 going
down. Full duty is ~16000 raw/s (~490 mm/s).

**Coast-down is first order with tau ~5 ms.** Stopping distance after drive is
cut is about `v * 6 ms`: 7 raw from 2300 raw/s, 80 raw from 12000 raw/s.
Braking (both low sides on) only shortens this by ~25% - the mechanism is
already heavily friction-damped, so **braking is not the anti-overshoot lever**.
Cutting drive early is.

## Why slow decay

The DRV8837 takes IN1/IN2 as 1/0 forward, 0/1 reverse, 0/0 coast, 1/1 brake.

- **Fast decay** (what the code used to do): PWM one pin, hold the other low, so
  each cycle alternates drive and coast. Winding current decays through the body
  diodes every cycle. At a high carrier the average current ends up far below
  what the duty implies, so low-duty torque collapses - hence the 190 breakaway
  in the table above.
- **Slow decay** (what `motor_set(..., slow_decay=true, ...)` does): hold the
  leading pin high from PORT, PWM the trailing pin between drive and brake, so
  its high fraction is the inverse duty. Current keeps circulating through the
  bridge, average current tracks duty, and low-duty torque survives.

This is why movement no longer drops the carrier to 306 Hz. That switch existed
purely to buy torque back, and it was the audible buzz. There is now one
permanently inaudible carrier (19.6 kHz) for everything.

Haptics use slow decay too, because they run through the same control law
(see "Haptics" below) and the plant model it inverts was measured in slow decay.

## The control law

In `main.cpp`'s `motor_update()`, `MODE_REMOTE_MOVEMENT_IN_PROGRESS`:

```
error = target_adc - input_ewma
if |error| <= move_deadband:  brake, and start/continue the settle timer
else:
    v_ref = MOVE_VREF_SLOPE * error               # position loop
    |v_ref| clamped to the requested speed limit  # if one was requested
    |v_ref| floored at move_vel_min               # Stribeck floor
    u  = sign * (breakaway + |v_ref| / k)         # feedforward: plant inverted
    u += move_kv * (v_ref - velocity_ewma)        # velocity loop
    u += sign(error) * stiction_ramp              # stall escape
    drive_ceiling += MOVE_TAKEUP_RAMP_RATE * dt   # opens up from move start
    clamp |u| to (drive_ceiling + stiction_ramp), capped at MOVE_MAX_DUTY
```

Since firmware 1.9 the first line, the position loop, is the **fast** profile
only. By default the velocity reference is the cap until a brake planned
on the measured speed and error takes over (`move_brake.h`, "Smooth motion"
below); everything from the speed clamp down is the same in both, except
the velocity gain, which the brake doubles.

**This is cascade control**: an outer position loop that asks for a velocity,
and an inner velocity loop that delivers it. Worth seeing in those terms,
because the previous flat form hid it - `u = KP*e - KD*v + FF` factors exactly
as `FF + KD*((KP/KD)*e - v)`, which is the same loop with `v_ref = (KP/KD)*e`
and gain `KD`. `MOVE_VREF_SLOPE` is that `KP/KD` ratio and keeps its old value
of 50/s; `MOVE_KV` is that `KD`.

Two things follow from writing it out, and both were real bugs in the flat form:

- **The inner loop's open-loop gain is `k * KV`**, not `KV`. `k` is the plant's
  ADC counts/sec per duty, so a higher-torque motor closes a proportionally
  hotter loop. Through the lag of the velocity estimate it goes underdamped and
  the fader oscillates *on velocity* - overshoot, brake, fall under,
  re-accelerate - which is not a position overshoot and does not look like one
  on the bench; it looks like the movement is chugging. `KV` is therefore
  scaled by `1/k` on a characterised unit, holding `k*KV` at the 1.16 the
  reference fader was tuned at.
- **The feedforward has to depend on the requested speed.** Inverting the plant
  gives `u_ff = breakaway + v_ref/k` exactly, so at steady state the feedback
  term is zero and the carriage simply travels at `v_ref`. A *fixed*
  feedforward instead commands whatever speed it commands - ~1100 ADC/s on the
  reference unit - and nothing slower is reachable without subtracting drive
  back off, which is what the old one-sided governor did. See "Optional speed
  limiting" below for why that misbehaved.

Five ideas, each solving a specific measured problem:

**Feedforward** is the plant model inverted: `breakaway + v_ref/k`. It supplies
both the breakaway duty and the duty that produces the wanted speed, so the
feedback term only has to correct the error in that model. Both halves are
measured per unit (see "Per-unit motor characterisation"); the compiled-in
defaults are the reference unit's values, so an uncharacterised fader behaves
as it always did.

**The velocity loop** is what prevents overshoot. Because `v_ref` is
`MOVE_VREF_SLOPE * error`, the controller is asking the carriage to be slower
the closer it gets, and drive falls away when it is travelling faster than
that. At 50/s against a coast limit of `1/tau ~ 200/s` it cuts about four
stopping distances short and coasts in. The old code had no such term and a
constant `+80` offset, so it drove at full authority right up to the deadband
and arrived at ~12000 raw/s - precisely the 58-80 raw overshoot it used to
show.

**Stiction ramp** prevents stalls near the target. Because FF sits below
breakaway, small errors produce `FF + KP*error` below breakaway: the controller
commands motion it cannot produce, sits stuck outside the deadband, and after
8 s falls into `MODE_ERROR`. The ramp adds drive at `MOVE_RAMP_RATE` only while
commanded-but-stalled, capped at `MOVE_RAMP_MAX`. The cap also means a jammed
fader cannot wind up to full drive.

It is shed at `MOVE_RAMP_DECAY_RATE` once the carriage breaks free - fast
(~20 ms from full, far shorter than the coast time, so it still cannot
contribute to overshoot) but **not instantly**. Dropping up to 70 duty in one
tick is itself a relay, and sitting near `MOVE_STALL_VELOCITY` it chatters:
wind up, break free, dump the drive, stall again. With the feedforward now
derived to clear breakaway this should rarely engage at all during a normal
move; it covers measurement error, a cold or stiff unit, and the
uncharacterised case.

**Take-up ceiling** addresses belt backlash. A direction-reversed move otherwise
starts at full duty into slack: the rotor free-runs unloaded, then snaps taut
with an audible click and a jerk. The drive ceiling therefore starts at
`MOVE_TAKEUP_DUTY` and opens up to full over time at `MOVE_TAKEUP_RAMP_RATE`.

It is re-armed when a move is commanded in the opposite direction to the one in
progress, not on every write to `LAYER_TARGET`. Slack is only taken up on a
reversal, and a host that streams position updates - an LVGL slider being
dragged, say - retargets many times a second; re-arming on each of those would
hold the ceiling at the take-up duty for the whole gesture and cap the fader at
roughly half speed.

Two constraints prevent problems found during development:

- **It must be time-based, not gated on "are we moving yet".** While crossing
  the slack the rotor *is* moving, it just isn't loaded, so a velocity-gated
  ceiling lifts partway through the slack and full duty still lands on
  engagement. Time is the only available signal that doesn't depend on whether
  the belt has taken up.
- **The ceiling must stay above the feedforward.** It exists to stop drive
  stepping to full duty, not to suppress the feedforward. Set below it, it
  starves the term that gets the carriage moving: the fader stalls, waits for
  the stiction ramp, and breaks free abruptly - which hunts *and* makes the
  click worse. This regressed once already, when the feedforward was re-centred
  upward and the ceiling was left at its old value, which is why the derivation
  now computes the ceiling from the feedforward rather than storing it loose.

**Optional speed limit** (see below) is off by default and changes nothing when
unset.

## Smooth motion

Firmware up to 1.8 moved the way the control law above reads: `v_ref =
MOVE_VREF_SLOPE * error`, so a move starts at the largest reference it will
ever see, runs flat out until the speed limit or the plant caps it, and
approaches the target on an exponential tail. It is the quickest the mechanism
moves (full travel in about 250 ms) and it looks like what it is: a jolt at
the start, a coast at the end. The faders on a Chase Bliss Automatone, the
reference Entity's faders are measured against, ease into the target: full
travel in 0.22-0.26 s with 10-90 % of the distance in 112-129 ms, overshoot
under 1 %, the same up and down (slow-motion video at 240 fps, Preamp MKII
fader).

The host cannot produce that from outside. Streaming intermediate targets
over I2C was tried (2026-10-21) and rejected: at 100 Hz the fader sees a
staircase of short moves, each one a soft start and a brake, and the result is
a shudder, not a curve. The shape has to come from inside the control tick,
which is where `move_brake.h` puts it.

### The first design, and why it was replaced

The first smooth profile (`move_profile.h`, still in the tree with its tests
until it is retired) was a velocity triangle: the reference started at the
Stribeck floor, rose at `a = 4d/T^2` and fell at the same rate so that it was
back at the floor at the deadband's edge, with the braking decision taken
every tick from the measured error. In simulation it was everything the
Automatone does. On the bench (2026-11-07, board 0, `T` = 500 and 1000 ms)
it was not: the carriage followed the falling reference with a large,
unit-dependent lag, reached the deadband at 1500-4000 counts/s, overshot by
16 counts and bounced. The reason is in the velocity loop: its feedforward
`breakaway + v_ref/k` keeps driving forward while only `kv (v_ref - v)`,
at 0.04 duty per count/s, pulls the carriage down toward a falling
reference, and in the drive range between zero and the duty that sustains
the speed the plant decelerates at only 10-15k counts/s^2 (it coasts; it
does not brake). The fast profile is clean for the opposite reason: its
collapsing reference makes a velocity error of thousands of counts/s, the
drive goes negative, and the motor brakes at 300-500k counts/s^2. The bridge
brake in the deadband stops an arrival under about 1400 counts/s inside the
8-count window; above that the carriage runs out and comes back from the
floor. And the move was uniformly slow: a 1000 ms `T` was needed for a clean
end, and the operator wants a fast move that decelerates only at the end.

### Cruise and brake

The profile in force keeps the cascade from "The control law" and replaces
only its first line, the position loop:

```
aim (move start, retarget, reversal):
    cruising
each tick, outside the deadband:
    distance = |error| - deadband - RUN_IN              # counts left to the run-in
    cruising:
        v_ref = cap (the speed byte, else VEL_CAP)      # the fast profile's take-off
        if distance * 2000 <= (max(v, v_min) + v_min) * T_b:
            braking, a = (v^2 - v_min^2) / (2 distance)  # from the measured speed v
    braking:
        v_ref = sqrt(v_min^2 + 2 a distance)             # on the measured error
        kv doubled
    |v_ref| clamped to the speed limit, floored at v_min, toward the target
```

- **The take-off is the fast profile's.** While cruising the reference is the
  cap, so the velocity loop is saturated and the drive is whatever the
  take-up ceiling and `MOVE_MAX_DUTY` allow, exactly as in firmware 1.8;
  the carriage reaches its top speed (about 8000 counts/s on the bench) as
  quickly as it can. The profile plans nothing here, which is why it costs
  no flash to speak of and why a fast and a smooth move start the same.
- **The brake is decided by time and distance from the measured speed.**
  `T_b`, the brake time (`REG_MOVE_PROFILE`'s ms, 120 by default), is how
  long the reference takes from the speed the carriage has down to the
  floor. A reference falling at a constant rate from `v` to `v_min` in
  `T_b` covers `(v + v_min)/2 * T_b` counts, so the brake starts when that
  is what is left to the run-in. From 8000 counts/s at 120 ms that is 513
  counts, about half of full travel; a carriage under the floor (at rest,
  moving away) is braked for as one at the floor, so a move shorter than
  about 90 counts never leaves the floor and simply crawls.
- **The curve is on the error, not on a clock.** From the entry on, the
  reference each tick is `sqrt(v_min^2 + 2 a (|error| - deadband - RUN_IN))`
  for the error measured that tick, with `a` fixed at entry from the speed
  and the distance. A plant that lags the reference finds the curve lower
  than its speed and is pulled down harder; a hand that slows the carriage
  finds it higher. The curve reaches `v_min` at the run-in
  (`MOVE_BRAKE_RUN_IN`, 24 counts before the deadband) and stays there, so
  the carriage settles on its floor equilibrium (the speed at which the
  floor's feedforward over-drive balances friction, about 1000-1400 counts/s
  on board 0) before the deadband, where the bridge brake takes over as
  before.
- **The velocity gain is doubled while braking** (`MOVE_BRAKE_KV_SHIFT`,
  smooth moves only; `move_drive()` takes the gain as an argument). Against
  the floor's over-drive this halves the lag the carriage keeps behind a
  falling reference, so it arrives under what the bridge brake can absorb.
  The fast profile and the haptics run the law's gain.
- **Retargets re-plan.** Any new aim starts by cruising and lets the next
  tick decide: a farther target while braking cruises again, a nearer one
  set late brakes at once from the speed the carriage has, on a curve
  continuous with that speed (the deceleration is clamped at
  `MOVE_BRAKE_ACCEL_MAX`; one it cannot meet leaves the curve under the
  speed, and the loop brakes as hard as it can). A reversal is a fresh
  cruise with the take-up ceiling re-armed, as before.
- **Wrong-way references are the floor back.** If the error changes sign
  without a new aim (an overshoot past the deadband), the reference is the
  floor toward the target, braking, never a new cruise.

Everything else is as before: the velocity loop, the feedforward, the
stiction ramp, the take-up ceiling, the brake in the deadband, the settle
timer, the movement timeout and `MODE_ERROR`, the stall escape. The speed byte
of `LAYER_TARGET` still caps the reference, in the cruise and on the curve.
Haptics use `move_vref` directly and are not touched.

### Fast and smooth, and the brake time

`REG_MOVE_PROFILE` (0x15, `i2c_data.h`) selects the profile and the brake time
at runtime, per fader, with no per-flash configuration:

- `MOVE_PROFILE_SMOOTH` (1, the default from reset) with a brake time `T_b`
  of `MOVE_PROFILE_TIME_MIN_MS` (50) to `MOVE_PROFILE_TIME_MAX_MS` (2000)
  ms, `MOVE_BRAKE_TIME_DEFAULT_MS` (120) by default;
- `MOVE_PROFILE_FAST` (0): the move of firmware up to 1.8, the ms ignored.

The ms was the whole move's time in the first design; in this one it is the
brake's, and a longer value starts the brake earlier and makes it gentler
without slowing the cruise. (`MOVE_PROFILE_TIME_DEFAULT_MS`, 240, is the
first design's default and stays in `i2c_data.h` for `test_move_profile`;
the firmware starts at `MOVE_BRAKE_TIME_DEFAULT_MS`.)

A write takes effect at the next move: a write during a move is kept until
that move ends (a later write replaces it), so a move finishes under the
profile it started with, and a read until then shows the profile in force.
The setting is not stored, so a host that wants fast moves, or another brake
time, writes the register after every fader reset, gated on
`FW_VERSION_MOVE_PROFILE`. The host needs nothing for the default.

### What the bench and the simulation say

Bench, 2026-11-07, board 0 at the default (the same build on all eight
boards since): full travel about 0.5 s including the 300 ms settle, so about
0.2 s of motion; overshoot 2-5 counts, one down move 18; no bounce; the
operator's verdict was that it is right. Measured again against the
Automatone's numbers it is a fast move with a soft end, not an S-curve: the
take-off is firmware 1.8's.

`test_move_brake` runs the whole loop (the filters, the plan, the drive with
the raised gain, the take-up ceiling, the brake) against the first-order
model of the reference plant from "The plant" (`k` 29, breakaway 68/86,
coast tau 5 ms) at the 2 kHz tick. The model tops out at 5400 counts/s where
the bench reaches 8000, and it brakes as readily as it accelerates, so its
times are longer and its ends cleaner than a fader's:

| move, T_b = 120 ms | arrival | 10-90 % | brake from | overshoot | arrival speed |
|---|---|---|---|---|---|
| full travel up (1023 counts) | 325 ms | 185 ms | 158 ms | 0 | 555 |
| full travel down | 347 ms | 198 ms | 181 ms | 0 | 557 |
| half (512) | 240 ms | 144 ms | 74 ms | 0 | 557 |
| quarter (256) | 194 ms | 143 ms | 28 ms | 0 | 554 |
| 80 counts (brake only) | 129 ms | 116 ms | 0 | 0 | 555 |
| 40 counts (floor crawl) | 57 ms | - | 0 | 0 | 555 |
| full travel, T_b = 50 / 400 ms | 294 / 511 ms | 167 / 337 ms | 197 / 62 ms | 0 | 559 / 555 |
| speed byte at 2000 counts/s | 591 ms | 416 ms | 424 ms | 0 | 552 |
| retarget 500 -> 900 at 150 ms (farther, while braking) | 339 ms | - | 72 ms | 0 | 556 |
| retarget 1000 -> 450 at 120 ms (nearer, late) | 124 ms | - | 120 ms | 0 | 448 |
| reversal 1000 -> 0 at 150 ms | 419 ms | - | 253 ms | 0 | 553 |

### Bench tuning

What to try on the fader, in this order:

1. **The brake time** is the one knob. Write `0x15` as `[1, ms >> 8, ms & 0xFF]`
   (240 is `[1, 0x00, 0xF0]`). Longer starts the brake earlier and lands
   slower; shorter arrives faster and, past the point where the carriage
   cannot shed its speed in the distance, overshoots. 120 ms was accepted
   on board 0.
2. **Fast for comparison**: `[0, 0, 0]` on the same fader, same move. That is
   exactly firmware 1.8's move; the start should look the same.
3. **Watch the end** for the three shapes: a bounce (the carriage runs past
   the deadband and comes back) is an arrival faster than the bridge brake
   absorbs, so a longer brake time or a larger `MOVE_BRAKE_RUN_IN`; a ring
   in the tail (the carriage hunting around the curve) is the doubled gain,
   `MOVE_BRAKE_KV_SHIFT` 0; a creep (the last millimetre at the floor) is
   the run-in or the brake time too long for that unit.
4. **Watch the start** only if it differs from fast mode's: it is the same
   code path, and a hesitation there is breakaway and the stiction ramp, not
   the profile.
5. **The speed byte** still works and is still unitless: it caps the cruise
   and the curve, and a limited move has the same end.
6. **Retargets**: a farther target during the brake cruises again (a second
   take-off, by design); a nearer one set late brakes at once. **A profile
   or brake time written mid-move** changes nothing until that move ends.

The per-unit part is the floor equilibrium: the speed a unit settles at
under the floor's feedforward depends on its friction, and the bridge
brake's reach on its mass. Characterisation (`motor_cal.cpp`) sets the
floor and the feedforward; the brake time is the host's to set per unit if
one needs it. Nothing is stored.

### Implementation notes

The plan is integer and written for the ATtiny's flash (`move_brake.h`,
"Fixed point" below for the rest of the loop): the brake's deceleration is
16 bits in sixteenths of a count/s^2, the curve is worked out in units of 16
counts/s (the floor, 560, is 35 such units exactly) so that its root is
16-bit and one Newton step a tick, descending from the previous tick's value,
finds it (two 32-bit divisions per move, at the entry and on each tick of
the curve; `avr-libc`'s is already linked for the ramps). The entry test and
the curve's radicand are one 32-bit product each. The whole profile,
including the gain argument of `move_drive()`, costs the same flash as the
first design it replaced: `fb_app_only` is at 14808 B of 14846 with either.

| constant | value | effect |
|---|---|---|
| `MOVE_BRAKE_TIME_DEFAULT_MS` | 120 | The brake time from reset. Full travel lands near 0.25 s on the bench. |
| `MOVE_PROFILE_TIME_MIN/MAX_MS` | 50 / 2000 | What a host may write; clamped. |
| `MOVE_BRAKE_RUN_IN` | 24 | Counts before the deadband at which the curve reaches the floor; where the carriage settles to its floor equilibrium. |
| `MOVE_BRAKE_KV_SHIFT` | 1 | The velocity gain is shifted up by this while braking (within `MOVE_BRAKE_KV_MAX_Q16`, 16383, what the loop keeps in 32 bits). 0 turns it off. |
| `MOVE_BRAKE_ACCEL_MAX` | 524288 | Clamp on the deceleration; a nearer retarget set too late to meet reaches it and brakes as hard as the loop can. Keeps the curve's product in 32 bits. |
| `MOVE_BRAKE_VEL_CAP` | 16000 | The cruising reference when the speed byte sets none, twice the plant's reach, so the loop saturates as the fast profile's does. |
| `MOVE_PROFILE_TIME_DEFAULT_MS` | 240 | The first design's default; unused by the firmware, kept for `test_move_profile`. |

## Haptics

Haptics are not a second controller. `haptic_pull()` picks a target - the
nearest detent, the calibrated end within `HAPTIC_MAGNET_RANGE` of it, the
calibrated midpoint for the centre notch, or the nearest placed notch -
and hands the error to the same `control_law()` a remote move uses, with no
speed limit, no stall ramp and no take-up ceiling. Only two things differ:

- **The drive is capped by haptic strength** (`get_strength_max_pwm()`,
  `HAPTIC_MIN_PWM` to full duty). While the user holds the fader off a detent
  the velocity reference is unmet, so the output saturates at the cap: that cap
  *is* the felt force. Strength 0 is kept well above breakaway so even the
  weakest setting returns the carriage to the detent.
- **The deadband coasts rather than brakes**, so the fader is free at rest.
- **The drive fades in past the deadband** (`haptic_engage()`,
  `HAPTIC_SOFT_BAND`): the law's feedforward floor is breakaway, about half
  duty, so switching it on at the deadband edge is a relay. A finger resting
  on the fader nudges it across that edge again and again and the motor kicks
  each time, which the hand feels as trembling. Scaling the drive from 0 at the
  edge to full over 16 counts turns the edge into a spring. The price is that a
  released fader can stop short of the detent's centre once the scaled drive
  falls below breakaway: up to about 20 counts at strength 3, with that weak
  drive left on until the idle timeout. (The error is truncated to whole
  counts, so the deadband is effectively 9.)
- **The centre notch acts only within a window** (`HAPTIC_CENTRE_NOTCH`,
  `haptic_notch.h`). A single detent pulls to the centre from anywhere on the
  travel, a spring; the notch pulls only within its half-width of the
  midpoint (the config's count field, in units of 8 counts) and coasts
  outside, so a continuous control gets a clear centre without fighting the
  hand elsewhere. Switching the capped drive on at the window edge would be a
  relay again, so the drive also fades out over the window's outer 16 counts.
  Both fades share one scaling step (`haptic_shape()`: the smaller of the two
  wins where they overlap), which saved the flash a second multiply would
  cost. At half-width 6 (48 counts, the recommended value) the pull is full
  from about 25 to 32 counts off centre and fades on both sides; at 4 or less
  it never reaches full strength, and 5 reaches it at exactly one count (24).
  A fader released near the window's outer edge may stay there, its faded
  drive below breakaway, by design.
  - The window starts after each fader's calibrated deadband (8-20 counts),
    not at the default 8. At deadband 20, width 6 peaks at about 14/16 of
    full strength, so faders can feel slightly different.
  - A fader released inside the window is pulled to the centre, so a resting
    position within about the deadband to 32 counts of centre is not kept
    once the hand lifts (a UX choice for Bartek: "clear but not sticky" vs
    "snaps to centre").
  - Stall exposure is the same as for detents: a fader held off centre at
    strength 7 stalls the motor until it is released, and the drive in the
    outer fade stays below breakaway until idle (1 s).
- **Step notches change the window with the touch** (`HAPTIC_STEP_NOTCHES`,
  `haptic_step_notch_half_width()`). The steps are the detents' targets
  (`get_nearest_detent_position()`, 2-10 evenly spaced including the ends).
  While the fader is touched (`STATE_TOUCH`) each step is a notch: the pull
  acts only within the `HAPTIC_NOTCH_WIDTH` half-width of the nearest step,
  shaped as the centre notch (soft engage past the deadband, fade-out over
  the outer 16 counts), and the hand moves freely between steps. Untouched,
  the window is the whole travel, as for detents, so a fader released between
  two steps is pulled to the nearer one: the release snap needs no host
  traffic. The window widens once the touch bit has stayed clear for 50 ms
  (`HAPTIC_STEP_RELEASE_SETTLE_MS`, `haptic_step_notch_held()`, the same gate
  as entering INPUT_ACTIVE), so a PTC release glitch with the hand resting
  between steps does not kick toward the nearest step. The soft engage fades
  the drive in only over the 16 counts past the deadband: a fader released
  within that band of a step starts gently, but one released further away
  (40 counts or more) gets the full strength cap in one tick and the control
  law decelerates it onto the step. That is the same as a detent's pull on a
  fader a remote move left off a detent, so not a new risk, but the snap is
  firmer than the notch under the hand. A host moving the fader (remote move)
  is unaffected: haptics run only in INPUT_ACTIVE.
  - Width: neighbouring notches overlap when the half-width is more than half
    the step spacing (about 1000/(count-1) counts of calibrated travel); the
    nearest step then wins everywhere and the mode feels like detents under
    the hand. Hosts should keep the width well below that.
  - Narrow widths: the notch acts only beyond the move deadband, so a width
    of 1-2 units (8-16 counts) is at or under the deadband of faders
    calibrated to 16-20 counts and never pulls under the hand; only the
    release snap remains, as for the centre notch. Keep the width at least
    deadband/8 + 2 units (4-6 is the planned range).
  - Stall exposure is as for detents, but only within a notch while held.
- **Placed notches exist only under the hand** (`HAPTIC_PLACED_NOTCHES`,
  `haptic_placed_notch_half_width()`, `haptic_placed_notch_nearest()`). Up to
  four positions from `REG_LAYER_NOTCHES`, on the scale of `REG_LAYER_TARGET`,
  for a continuous control with places worth finding by feel. While the fader
  is touched each is a notch, shaped as the centre notch, with the same
  half-width field as the step notches. Untouched there is no window at all
  and the motor coasts: not a settle time, as the step notches have, and not a
  pull to the nearest notch, because a released fader must not move and the
  value must not change. So a fader let go inside a notch stays off its centre
  where the hand left it.
  - The test is `STATE_TOUCH` and the latest reading still at or above the
    release threshold (`haptic_placed_notch_under_hand()`), with no delay on
    purpose. `STATE_TOUCH` alone clears only after two more acquisitions (40 ms
    and more at the library's 20 ms period), long enough to slide a fader held
    on a notch's slope; the reading ends the pull at the first low
    acquisition, so up to about 20 ms after the capacitance falls, plus how long
    that takes as the finger lifts. A dip in the reading drops the pull for
    that acquisition, which is a weaker pull under a hand, not a kick. Whether
    what is left moves a fader lifted from a slope is to be checked on the mule
    (decision 0143's HIL); a fader that moved then moved honestly, and the
    value follows it. A touch bit stuck on with no hand (decision 0089) is the
    known limit, as for modes 3 and 4.
  - Spacing: two notches overlap when they are closer than twice the
    half-width (96 counts at width 6, about a tenth of the travel), and the
    nearest wins between them with a sign flip at the midpoint. Hosts keep
    them further apart.
  - Stall exposure is as for the step notches under the hand; none released.

The previous haptics were `150 + 3 * error` with no velocity term - far above
breakaway right up to the dead zone, so a released fader arrived fast, sailed
through the detent and was pulled back, oscillating. The velocity loop cuts
drive (and reverses it) when the carriage is moving faster than the error
warrants, exactly as it does at the end of a move.

A side effect of the velocity loop: dragging the fader *away* from a detent
feels slightly stiffer the faster you go, and flinging it *toward* one is
damped. Both saturate at the strength cap.

## Fixed-rate loop and the velocity estimate

The control law runs on a `CONTROL_TICK_US` tick, not once per `loop()`.
Previously it ran every iteration, so gains and filter constants were per
*iteration* and drifted with however long touch processing took. `adc_drain()`
still runs every pass, folding every completed ADC conversion into `input_ewma`
on the `RESRDY` flag, so the position filter runs at the ADC's own constant rate
and never re-uses or tears a result.

Velocity is `d(input_pos_q8)/dt`, smoothed with a fixed time constant
`MOVE_VEL_TAU_US` (4 ms). The position is kept in Q8 and fed by a dithered
ADC, so its differences stay meaningful well below one ADC count - no need for
a longer baseline. **dt is measured from the previous execution, not the
scheduled tick time**; using the schedule silently skews the estimate whenever
the loop cannot keep up.

At 2 kHz the main loop still runs 6-8 kHz, leaving headroom for `ptc_process`.
At 250 us the loop saturates (loop rate == tick rate) with no margin; don't.

## Fixed point

The law, the filters and the ramps run in integer arithmetic (`move_law.h`;
the float versions cost about 1.1 KB of avr-libc soft float, see "Flash
budget"). The scales:

| Quantity | Representation | Why |
|---|---|---|
| position, error | ADC counts in Q8 (`x 256`), `int32_t` | the ADC dither makes sub-count position meaningful; a full-scale difference times the Q16 filter coefficient still fits 32 bits |
| velocity | whole ADC counts/s, `int32_t`, clamped to +-32767 | one count/s is far below anything the law resolves; the clamp keeps every product below in 32 bits |
| duty | whole 0-254; the stall ramp and the take-up ceiling accumulate in Q8 | the ramps advance by fractions of a duty per tick |
| `1/k`, `KV` | Q16, `uint16_t` (`MoveLaw::inv_k_q16`, `kv_q16`) | the only fractional gains; `MOVE_KV_Q16` 2621 is 0.04 |
| slope, floor, deadband, breakaway | whole | they always were |

What the representation costs, measured by the host tests in
`firmware/test/test_move_law/` against the float law copied into them as the
reference (`pio test -e native_test`, the numbers are printed):

- position filter: within 0.013 ADC counts of the float EWMA over a dithered
  full-scale sweep; a constant input settles within 10/256 count of itself
  (the Q16 coefficient rounds a smaller difference to no step);
- velocity estimate: within 4.5 counts/s of the float estimate through an
  8000 counts/s move (one Q8 position step per 500 us tick is 7.8 counts/s);
  the tick's alpha is kept in Q12, because at Q8 the 500 us tick's 0.125 was
  quantised by 0.85% and drifted the estimate by tens of counts/s through a
  transient;
- control law: within 1 duty of the float law inside +-254 duty over 4.5 M
  samples (four plants, five speed caps, full-scale error, +-8000 counts/s);
  the deviation is the rounding of 1/k and KV to Q16 crossing an integer
  boundary;
- stall ramp and take-up ceiling: within 1 duty over a 450 ms episode; the
  per-tick step is rounded so truncation cannot accumulate into a lag.

`error == 0` takes the falling branch, as the float sign did. The lab build's
`REG_DEBUG_GAINS` keeps its x1000 wire scaling, but the slope and the deadband
now land on whole numbers and KV on Q16.

## Constants and how to change them

| constant | value | effect |
|---|---|---|
| `CONTROL_TICK_US` | 500 | 2 kHz. Higher rates gain little; the limit is friction, not sample rate. |
| `MOVE_VREF_SLOPE` | 50 | ADC/s of velocity reference per ADC count of error. The old `KP/KD`. Higher approaches faster and coasts in from further out. |
| `MOVE_KV_Q16` | 2621 (0.04 in Q16) | Velocity loop gain. **Default only** - scaled by `1/k` on a characterised unit, because `k*KV` is what sets damping. |
| `MOVE_BD_RISING/FALLING` | 68 / 86 | Assumed per-direction breakaway duty. **Default only** - measured on a characterised unit. |
| `MOVE_K_DEFAULT` | 29 | Assumed ADC counts/sec per duty. **Default only** - measured. |
| `MOVE_VEL_MIN` | 560 | Floor on the velocity reference, and so on the feedforward's clearance above breakaway. |
| `MOVE_DEADBAND` | 8.0 | On-target window - the final error at which a move is declared arrived. Has a hard floor set by the plant - see below. Don't lower it. Derived per unit when characterised. |
| `MOVE_STALL_VELOCITY` | 60 | "Not moving" threshold, for the stiction ramp. |
| `MOVE_RAMP_RATE/MAX` | 250 / 70 | Stall escape strength. |
| `MOVE_RAMP_DECAY_RATE` | 3500 | How fast the stall escape sheds once moving. Finite, not instant - see below. |
| `MOVE_TAKEUP_DUTY` | 130 | Drive ceiling at move start. Must stay above FF, which is why it is derived from FF on a characterised unit. |
| `MOVE_TAKEUP_RAMP_RATE` | 1200 | Duty/sec the ceiling opens up. Lower = gentler start. |
| `MOVE_SPEED_SLOWEST_MS` | 700 | Full-travel time at host speed 0. The slow end of the validated window. |
| `MOVE_SPEED_FASTEST_MS` | 250 | Full-travel time at host speed 254. Faster than this the limiter stops biting. |
| `MOVE_TIMEOUT_TOLERANCE` | 20 | On timeout, error below this goes idle instead of `MODE_ERROR`. |
| `MOVE_BRAKE_*`, `MOVE_PROFILE_*` | see "Smooth motion" | The smooth profile's brake time, run-in, gain shift and clamps. `MOVE_VREF_SLOPE` governs the fast profile only. |

**The mechanism sets the minimum deadband.** Nothing moves
below breakaway, and at breakaway speed jumps straight to `v_jump`, so the
smallest correction the fader can make is that speed times the time it takes to
actually stop. A deadband below that floor cannot be satisfied: the controller
steps past the window in both directions forever, which looks like dithering at
the end of a move and eventually trips the movement timeout.

**That stopping time is not the coast time.** Coast-down is first order with
tau ~5 ms, so the carriage travels ~`v * 6 ms` after drive is cut - but the
controller does not cut drive the instant the carriage reaches the window. The
position filter and the control tick both have to notice first, and the
carriage keeps moving throughout. The reference unit's measured minimum
correction of 5-8 ADC counts at 450-560 ADC/s implies **9-18 ms end to end**,
two to three times the coast alone. `MOTORCAL_STOP_TIME_MS` is 15 ms for that
reason; sizing it at 6 produced a window a looser fader could not land in.

Prefer a slightly larger deadband: too small causes dithering and timeouts;
too large leaves a position error of 2-5 LSB in the host's 8-bit position.

**`k*KV` is the speed/robustness tradeoff**, and it is the product that
matters, not `KV` alone. Lower is faster but rings sooner; higher is smoother
but slower to settle. Lowering it is mathematically equivalent to adding
carriage mass, since both make the carriage coast further, so a sweep of it
doubles as an inertia sweep. Measured on the jig at `k*KV` 1.16:

| effective inertia | 1.0x | 1.5x | 2.0x | 3.0x |
|---|---|---|---|---|
| reversals over the suite | 0 | 6 | 60 | 145 |

So the tuning tolerates roughly 1.5-2x the *total* moving inertia (carriage +
belt + reflected rotor inertia) before ringing returns. A plastic knob cap is a
few grams against that and is not a concern.

**This is why `k` has to be measured.** Leave `KV` at a fixed 0.04 and a fader
with `k` 60 instead of 29 runs `k*KV` of 2.4 - past the 2.0x column above, into
the regime where the loop rings. It shows up as velocity chugging during the
move rather than as position overshoot at the end, which is why it is easy to
misread as a speed-limiter problem.

## Tuning for hardware variance

Tune the gains to settle reliably across faders with different friction and
age. Avoid slow settling and `MODE_ERROR` timeouts before improving position
accuracy on any one fader.

The parameter that matters is how the feedforward compares to a given unit's
breakaway duty. Sweeping FF on the reference fader (breakaway 68 rising) shows
where moves stop settling:

| FF (rising) | 46 | 66 | 86 | 106 | 126 | 146 | 166 |
|---|---|---|---|---|---|---|---|
| moves that settled | 3/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 4/6 |

The window is roughly **66 to 146**, and it is very lopsided: only about 2 counts
of room below the reference breakaway, but ~78 above. So FF is set to 106, the
centre, rather than just under breakaway.

**Why too-low FF is the dangerous direction.** If FF lands below a unit's
breakaway, small errors cannot produce motion at all. The carriage stalls just
outside the deadband, the stiction ramp winds up, and when it finally crosses
breakaway the speed jumps discontinuously to ~900 raw/s - far too fast to stop
inside the window. It overshoots, the error flips sign, and the cycle repeats
until the movement timeout. That is what "motor active, oscillating, never
settles, then error" looks like in the field, and no deadband value fully
rescues it (tested up to 10 ADC counts).

**Higher FF is limited by the take-up ceiling.** `MOVE_TAKEUP_DUTY` caps drive
while the carriage is stalled, so an over-high feedforward is simply clamped instead of producing a
violent escape. Once moving, the D term absorbs the extra.

Measured tolerance at the shipping values, emulated by offsetting FF:

| unit breakaway vs reference | -40 | -30 | -20 | -10 | 0 | +10 | +20 | +30 | +40 |
|---|---|---|---|---|---|---|---|---|---|
| moves that settled | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 6/7 |

So roughly **-40 to +30 duty counts** of breakaway spread is absorbed, with
worst-case steady-state error about 5.6 ADC counts (~0.35 mm, under 1.5 LSB of
the 8-bit position a host sees). The previous "just under breakaway" tuning
failed already at -10.

With FF centred, the design also stops being sensitive to the other constants:
every deadband from 3 to 9 and every take-up value from 70 to 150 settles 7/7.
That insensitivity is the point - if a change makes behaviour depend sharply on
one constant again, that is a regression even if the reference fader looks good.

### The other failure: a unit with LESS friction than the reference

Centring FF buys tolerance, but it does not make the design unit-independent,
and the failure at the low-friction end is the mirror image of the one above.
Near the target the slowest motion the controller can command is the
equilibrium of `u = KP*e + FF - KD*v` against the plant:

```
v_min = k * (KP*deadband + FF - breakaway) / (1 + k*KD)
```

and a move can only settle if `v_min * (stopping time)` fits inside the
deadband. On the reference unit (breakaway 68, k 29) that is
`29*(12 + 106 - 68)/2.16 = 670` ADC/s, about 4 counts of stopping distance
inside a 6 count window. On a unit whose breakaway is 40 and whose `k` is 40,
the same constants give `40*(12 + 106 - 40)/2.6 = 1200` ADC/s and 7.2 counts -
**larger than the deadband**. The fader cannot make a correction small enough
to land in the window, so it steps past, reverses, steps past again, and hunts
until the movement timeout.

That is what a brand-new, looser fader looks like on the bench: jerky, worst at
low commanded speeds, and never quite arriving. It is not fixable by raising the
deadband (the window would have to grow past what the host can see) and it is
not fixable by lowering FF globally, because that reintroduces the stall failure
on stiffer units. The gap between FF and breakaway has to be a *per-unit*
quantity.

## Per-unit motor characterisation

Self-calibration therefore measures the plant as well as the endpoints, and the
gains above become defaults that a characterised unit overwrites. Three numbers
per direction, in `MotorCalData`, persisted in the same EEPROM record as
`calib_min`/`calib_max` and reported at `REG_MOTOR_CAL` (0x12):

| measured | what it is |
|---|---|
| `bd` | breakaway duty - the duty at which the carriage first moves at all |
| `k` | ADC counts/sec of cruise gained per duty count above breakaway |
| `vjump` | the speed motion starts at once breakaway is crossed (Stribeck) |

**The measurement.** Each run parks at the end it will travel away from, brakes
for 200 ms (breakaway from a standstill is a different number from the duty
needed to *sustain* motion, so residual coast would corrupt it), then ramps duty
at ~80/sec until speed exceeds a motion threshold - that duty is `bd`. It holds
there for 40 ms taking the peak speed, which is `vjump`, then dwells at `bd+25`
and `bd+50` for 100 ms each, averaging the tail of each hold. The two dwell
speeds give a two-point fit for `k`. Runs alternate direction so each starts
where the last ended, two passes per direction, averaged. Anything implausible
(`bd` outside 10..180, `k` outside 8..90, a run reaching the calibrated ends)
discards the whole characterisation and keeps the compiled-in defaults - a
half-measured fader is worse than an un-measured one.

**What is derived** (`apply_motor_calibration`, in `motor_cal.cpp`):

```
breakaway, k  ->  straight into the feedforward, u_ff = bd + v_ref/k
KV            =  1.16 / k        (holds the inner loop's gain k*KV constant)
vel_min       =  clamp(1.25 * vjump, 200, 1500)
deadband      =  clamp(vel_min * 15 ms, 8, 20)
takeup        =  max(130, max_dir(bd + vel_min/k) + 20)
```

The compiled-in defaults are themselves a plant model - breakaway 68/86, `k`
29, `vjump` 448 - so there is one derivation rather than one per case, and an
uncharacterised fader provably runs the gains the constants were tuned to.

**On a unit that has never been characterised** - or whose measurement failed -
every derived value falls back to the constant in the table above, so the
behaviour is exactly the pre-1.2 firmware. `REG_MOTOR_CAL` byte 0 says which
case a fader is in.

## Flash budget

Worth knowing before adding anything here: **the ATtiny1616's 16 KB is the
binding constraint on this file**, and the I2C bootloader work reserves the top
2 KB, leaving **14336 bytes for the application**. The characterisation above
was written to fit that, which is why the measurement and the derivation are
integer throughout and the ramp is fixed point - the float versions of the same
logic cost roughly 1.3 KB more and did not fit.

Three unrelated savings were taken at the same time and are worth not undoing:

- `main.cpp` no longer includes `<megaTinyCore.h>`. Nothing used it, but its
  ADC-error helpers take `HardwareSerial&` default arguments, which under LTO
  pulls all of `UART0.cpp` - both USART ISRs and the `Print`/`HardwareSerial`
  machinery - into a build with `SERIAL_ENABLED` off. **800 bytes of flash and
  145 of RAM.**
- The heartbeat LED and `nSLEEP` go straight to `VPORTB` rather than through
  `digitalWrite()`, which is 160 bytes of pin lookup for two call sites.
- `onI2cRequest` builds multi-byte replies through `i2c_write_u16`/`u32` helpers
  instead of writing bytes out at each call site.

The control law itself was the last float on the chip: position EWMA,
velocity estimate, `control_law()`, the ramps, the priming and the gain
derivation (`1/k`, `1.16/k`) together pulled in `__addsf3`, `__mulsf3`,
`__divsf3`, `__fp_*`, `__fixunssfsi`, `__floatunsisf` and their wrappers,
about 1.1 KB. Moving the law to fixed point (`move_law.h`, "Fixed point"
above) took the `fb_app_only` application from 14822 to 13696 bytes (the
limit is 14846) with no soft-float symbol left in either build; the
self-calibration's 0.995 span buffer went to per-mille integer arithmetic in
the same step, as it was the one other `double`.

The smooth profile (firmware 1.9) spent most of that again: the plan,
`REG_MOVE_PROFILE` and the second reference path together are about 1000 B,
and `fb_app_only` is at 14808 B of 14846. The first design's first version,
in signed Q8 with 32-bit products and the measured tick, did not fit by
290 B; the cruise-and-brake design that replaced it was 114 B over on its
first build and fits at the same 14808 B after dropping redundant clamps
and a second Newton division. **38 B are left.** A planner re-seeded from
the measured velocity at a mid-move `REG_MOVE_PROFILE` write (about 300 B)
was the first thing the budget refused.

If a future change does not fit, measure before trimming: build, then
`avr-nm -S --size-sort -C .pio/build/fb_app_only/firmware.elf | tail -20`
(the toolchain's `avr-nm` lives under `~/.platformio/packages/toolchain-atmelavr*/bin/`).

## Optional speed limiting

`LAYER_TARGET` (0x0E) takes an optional 4th byte capping the speed of the move.
Three-byte writes behave exactly as before, so this is backwards compatible and
does not change the protocol version.

A host that uses the byte should send it on every move, 255 included. The byte
sets the layer's *stored* speed, and a 3-byte write leaves that alone - so
falling back to 3 bytes for a full-speed move silently re-applies whatever limit
the layer last held. Send 3 bytes only to firmware predating the byte, which
ignores a 4-byte write entirely rather than moving.

The byte is a **unitless 0-255 speed**: 255 (the default) is unlimited, and 0 is
the slowest the mechanism moves smoothly. Values below 255 map linearly in
*velocity* across `MOVE_SPEED_SLOWEST_MS` .. `MOVE_SPEED_FASTEST_MS`, which are
stated as full-travel times because that is how the measurements below were
taken; the calibrated span turns each into a velocity.

The scale is deliberately unitless rather than a move time in ms. The usable
window is narrow and bounded at both ends by the plant - past the fast end the
limiter stops having any effect, past the slow end the mechanism stick-slips -
so a physical-units parameter mostly offers values that do nothing or cannot be
honoured. Mapping the whole byte onto the validated window means every value a
host can send does something, and the endpoints can be re-measured and moved
without changing the host-facing meaning of the byte. The parameter does not
specify a move duration; timing accuracy was only about 15% with the previous
scheme.

**It is realised by clamping the velocity reference**, and nothing else - one
line in the control law. Because the feedforward is `breakaway + v_ref/k`, a
lower reference directly means less drive, so the requested speed is produced
rather than fought for.

The previous fixed feedforward commanded ~1100 ADC/s regardless of the request, so clamping the error could never slow
the fader below that floor. Two extra parts were bolted on to work around it:
an error clamp to stop the P term saturating, and a **one-sided governor** that
subtracted drive once measured velocity exceeded the limit.

A one-sided correction puts the operating point exactly on its own
discontinuity: below the limit it contributes nothing at all, above it a lagged
correction. That is a limit-cycle generator, and it behaved like one - overshoot
the limit, brake hard, fall under, accelerate again, most visible at low
requested speeds where the cycle is a large fraction of the commanded velocity.
Both parts are gone; the plant-inverting feedforward removed the floor that
made them necessary.

Measured on the reference fader, requesting a full-scale time and timing a move
of 67% of full scale:

| requested full-scale | 250 ms | 400 ms | 600 ms | 800 ms | 1200 ms | 2000 ms |
|---|---|---|---|---|---|---|
| measured | 193 ms | 261 ms | 356 ms | 444 ms | 584 ms | 759 ms |
| vs expected | +16% | -2% | -11% | -17% | -27% | -43% |

**Usable range is roughly 250 ms to 700 ms of full-scale time**, hence those two
endpoints for the speed scale. Those figures were measured against the old
governor, whose tracking error ran to -43% at the slow end; the reference-clamp
form should track considerably better, since the feedforward now produces the
requested speed directly instead of being pulled back down to it. **Re-measure
that table** - it is the clearest single check that this change did what it was
meant to.

Beyond the slow end it saturates: there is a hard floor below which the motor
cannot sustain continuous rotation and creeps in stick-slip steps rather than
moving smoothly. Slower smooth motion is not reachable by tuning - it would need
a different drive scheme.

`move_vel_min` (1.25x the measured Stribeck jump, 560 ADC counts/sec by default)
is the clamp for that floor. It is applied to the velocity reference in the
control law, every tick, and nowhere else - not where the speed byte is decoded,
which would freeze it against a `move_vel_min` that calibration can change
afterwards. It is also the floor the reference is held at right next to the
target, which is what keeps the feedforward clear of breakaway for small errors.

Direction asymmetry is within about 7%.

Treat this as a *limit*, not a precise speed control - it is realised through
the friction-dependent plant, so exact velocity varies with the fader.

The lab jig's `sstep <from> <to> <speed> <log_ms>` command takes the same
unitless byte, so re-measuring the table above means picking speeds rather than
times.

## Reproducing the measurements

Two extra build environments exist for this; neither ships.

```bash
source ~/.platformio/penv/bin/activate

# ATtiny with REG_DEBUG_DRIVE / STATUS / GAINS (repo root)
pio run -e fader_buddy_lab -t upload

# ESP32 jig as a serial-driven measurement harness, replacing the
# production test state machine
cd production_tools/programAndTest && pio run -e lab -t upload
```

`env:lab` (`src/lab_main.cpp`) gives a line-oriented serial interface at
921600 baud: `step <from> <to> <ms>` captures a step response at ~3.2 kHz,
`openloop` and `stopdist` drive the bridge directly for system ID, `dbg` reads
control-loop internals, and `gain <index> <value>` overrides any tuning constant
at runtime so a gain sweep does not need a reflash per trial. Gain indices are
in `i2c_data.h` (`DEBUG_GAIN_*`); the reference slope, KV and deadband are
scaled by 1000.

Restore production firmware with `pio run -e fader_buddy -t upload` and
`pio run -e lilygo-t-display -t upload`.

The debug registers (0xF0-0xF2) are additive and compiled out unless
`DEBUG_DRIVE` is defined, so the protocol version is unchanged.

`REG_MOTOR_CAL` (0x12) is *not* debug-only and is present in production builds,
so a unit's measured breakaway, slope and jump speed can be read on a bench with
nothing but an I2C host attached - no jig, no lab firmware, no programmer. The
ESPHome component logs all of it at startup.

## Measurement gotchas

- **Derive the target from `calib_min`/`calib_max`, not from where the fader
  settles.** Read them via `REG_DEBUG_STATUS`. Inferring the mapping from
  settled endpoints bakes in the controller's own error - this produced a
  phantom 22 raw bias early in tuning.
- **The `STATE` raw ADC field is noisy while the motor drives.** Excursions of
  40+ raw appear there that the filtered `pos` field does not show; they are ADC
  noise, not motion. Median-filter before computing overshoot, and cross-check
  against `pos`.
- **Clear `MODE_ERROR` between automated trials.** One latched error otherwise
  makes every later trial report a frozen fader.
- Reported `err` from `REG_DEBUG_STATUS` is the controller's own Q8 error,
  reported in eighths of a count, and is the ground truth for steady-state
  accuracy.
