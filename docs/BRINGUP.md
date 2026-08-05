# First power-on

None of this firmware has ever run on hardware. It compiles and its logic
is unit-tested, but compiling is not running. Expect the first session to
find real problems — that is what this order is designed to catch, cheaply
and in isolation, before anything can strip a gear.

**Do these in order. Do not skip ahead to walking.**

---

## 0. Before power

- [ ] **Servos NOT plugged in.** Every step up to §4 is done with the servo
      connectors off.
- [ ] Battery disconnected. Bench power or USB only.
- [ ] If an HC-SR04 is fitted: **confirm the echo line is level-shifted**
      to 3.3 V. The sensor drives 5 V and the ESP32 is not 5 V tolerant —
      wiring it direct damages the pin. A 1 kΩ / 2 kΩ divider is enough.
- [ ] Buck converter output measured at **5.1 V** *before* it is connected
      to anything.

---

## 1. Flash it

```bash
make ports     # confirm the board is seen
make flash
make monitor   # 115200
```

**If upload fails or the board boot-loops:** unplug the servos on wire
channels **0 and 1** and retry. They sit on GPIO 15 and GPIO 2, which are
ESP32 boot strapping pins latched before any code runs. Firmware cannot
fix this. Check it before suspecting cables or drivers.

Expected on the serial console:

```
sesame firmware
servo self-test ok
no stored calibration -- using defaults
control surface at http://192.168.4.1
tick 20 ms. type 'help'.
```

**`FATAL: servo self-test failed on wire channel N`** means the wrong
ESP32Servo got picked up — almost always a Library Manager copy shadowing
the vendored 3.0.9. Remove it; see `firmware/sesame/src/vendor/ESP32Servo/VENDORING.md`.

---

## 2. Check the things that cost nothing

Still no servos attached.

- [ ] `status` responds
- [ ] OLED shows a face and blinks. Blank display → SDA/SCL swapped, or the
      panel answers at 0x3D rather than 0x3C.
- [ ] Wi-Fi network `Sesame` appears; `http://192.168.4.1` loads the page
- [ ] `estop` then any motion command → refused. Then `stand` clears it.

---

## 3. One servo at a time

Plug in **channel 0 only**. Nothing else.

```
setjoint 0 0
setjoint 0 20
setjoint 0 -20
```

For each joint, before moving on, confirm:

- [ ] **The correct leg moves.** If a different leg moves, the wiring does
      not match `kChannelMap` — fix the wiring, not the code. Channels 4
      and 5 are Rear-right knee and Front-right knee respectively; that
      ordering looks wrong and is correct.
- [ ] **It moves the expected direction.** Hip: `+` swings forward. Knee:
      `+` puts the foot down. If reversed, set `invert` for that joint:
      ```
      setcal <j> 732 2929 0 1 -60 90
      ```
- [ ] **It is mechanically centred at 0.** If not, adjust `trimDeg` (the
      4th argument) until it is.

Repeat for channels 1 through 7. **This is the slow part and it is worth
doing properly** — every pose and gait downstream assumes it.

Then:

```
savecal
```

Without this the calibration is lost on the next power cycle.

---

## 4. Standing

All servos attached, robot **held in your hands, not on the ground**.

- [ ] `stand` — all four legs should reach a symmetric stance
- [ ] `rest` — legs fold flat
- [ ] Alternate a few times. Any judder or fighting means a joint is still
      mis-trimmed. Go back to §3.

Then set it down.

- [ ] It supports its own weight without collapsing
- [ ] `status` shows `throttle=1.00`. If it is below 1.0 while merely
      standing, the power budget is already saturated — see §6.

---

## 5. Measure the robot

Two numbers in `firmware/sesame/src/robot_config.h` are **placeholders**
and every gait calculation depends on them:

| | measure with calipers, in mm |
|---|---|
| `kPlaceholderCoxaMm` | hip yaw axis → knee pitch axis |
| `kPlaceholderLegMm` | knee pitch axis → foot contact point |

Also `kHalfBodyLenMm` / `kHalfBodyWidMm` — body centre to hip axis.

Edit that one file, reflash. Nothing else hardcodes a dimension, and the
host test suite already runs every gait invariant across a sweep of link
lengths, so a correction needs no other change.

Until these are real, `bodyHeightMm` and `stepHeightMm` are self-consistent
but do not match the physical robot.

---

## 6. Walking

Robot on the floor, **clear space**, ready to catch it.

```
drive 20 0 27 8      # slow, forward
```

Remember `drive` is a **stream**: the robot stops itself after 500 ms
without a new command. That is the watchdog, and it is deliberate. Use the
web UI or the companion for continuous walking.

- [ ] It walks forward rather than shuffling in place
- [ ] `drive 0 20 27 8` turns on the spot
- [ ] `stop` halts smoothly rather than snapping

**If it browns out or resets mid-stride** — the most likely failure on a
600 mAh pack:

1. Add a **1000 µF+ capacitor across the 5 V servo rail**, close to the
   servo headers. Cheapest fix available and it is hardware, not firmware.
2. Reduce `maxTotalDegPerSec` in `MotionLimits`.
3. Walk slower: lower `vx`.

`throttle` in `status` tells you when the power governor is active. Below
1.0 means it is already limiting motion to protect the rail.

---

## 7. Sensors, if fitted

- [ ] IMU: `status` shows a `tilt=` field. Tip the robot by hand while
      standing — the legs should adjust to keep the body level. If it
      over-corrects or oscillates, reduce `kLevelGain` (currently **0.35**,
      an untuned guess) in `sesame.ino`.
- [ ] Rangefinder: `status` shows `range=`. Wave a hand in front of it
      while walking — it should stop. `range=none` means UNKNOWN, not
      clear; check the level shifter and wiring.

---

## 8. Voice

```bash
make voice --robot            # typed, real robot
make voice-listen             # spoken
```

The companion talks to the robot over Wi-Fi. Either join the robot's
`Sesame` network, or put the robot on yours by filling in
`firmware/sesame/secrets.h` (copy `secrets_example.h`).

---

## Known-unverified list

Everything below was reasoned but never measured. If something behaves
oddly, suspect these first:

- 20 ms tick timing under Wi-Fi load
- The chunked OLED flush (~2.9 ms/page assumption)
- Ultrasonic echo ISR timing
- IMU I²C burst read and filter constants
- `kLevelGain = 0.35` — a guess
- Link lengths — placeholders until §5
