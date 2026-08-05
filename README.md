# sesame-robot

Custom firmware for a [Sesame](https://github.com/dorianborian/sesame-robot) quadruped robot, written from scratch for the **ESP32-WROOM-32**.

Sesame is an open-source ESP32 mini walking robot by [Dorian Todd](https://www.doriantodd.com/) — 8× MG90S servos, an SSD1306 OLED face, and WiFi control. This repo is an independent firmware implementation for that hardware platform, not a fork.

## Hardware target

| | |
|---|---|
| Controller | ESP32-WROOM-32 (DevKitC-32E), hand-wired to protoboard |
| Actuators | 8× MG90S metal-gear micro servos |
| Display | 0.96" SSD1306 128×64 I²C OLED |
| Power | 2S 14500 Li-ion (7.4 V) → 5 V buck converter |

### Pin map

| Motor idx | GPIO | Joint |
|---|---|---|
| 0 | 15 | R1 |
| 1 | 2 | R2 |
| 2 | 23 | L1 |
| 3 | 19 | L2 |
| 4 | 4 | R4 |
| 5 | 16 | R3 |
| 6 | 17 | L3 |
| 7 | 18 | L4 |
| — | 21 | I²C SDA |
| — | 22 | I²C SCL |

> GPIO 2 and 15 are ESP32 boot strapping pins. If an upload fails, unplug motors 0 and 1 and retry.

> **Got the board?** Read **[docs/BRINGUP.md](docs/BRINGUP.md)** before you
> plug in servos. This firmware has never run on hardware; that document is
> the order that finds the problems cheaply instead of expensively.

## Quick start

```bash
make test      # host unit tests for the motion core -- no board needed
make verify    # compile the firmware for ESP32 -- no board needed
make ports     # is the robot plugged in?
make flash     # compile + upload
make monitor   # serial console at 115200
```

`make test` needs only `clang++` and `make`. The ESP32 targets need
`arduino-cli`; see [`firmware/TOOLCHAIN.md`](firmware/TOOLCHAIN.md) for
that and for the Arduino IDE route, which uses the same files.

To use the IDE instead, open `firmware/sesame/sesame.ino`, select
**ESP32 Dev Module**, and set **Tools → Arduino Runs On → Core 1**.

## Layout

```
firmware/sesame/          the Arduino sketch
  sesame.ino              setup() + loop() + command dispatch, nothing else
  config.h                pin map and timing
  src/core/               PURE C++11 -- no Arduino, unit-tested on the host
  src/board/              hardware layer (servos; OLED and sensors to come)
  src/vendor/ESP32Servo/  pinned 3.0.9, vendored deliberately
firmware/host/            unit tests + an offline gait simulator
```

`src/core/` has no Arduino dependency by construction, which is what makes
the IK, gait, and slew math testable with no hardware attached. That
contract is written down in
[`src/core/README.md`](firmware/sesame/src/core/README.md) and enforced by
`-Werror` on the host build.

## The voice assistant

```bash
make voice          # type commands, simulated robot -- works right now
make voice-listen   # say "hey sesame, walk forward" (macOS app bundle)
make voice --robot  # drive a real robot over WiFi
```

Speech recognition runs **on-device** via Apple's Speech framework: free,
offline, no API key, and your voice never leaves the machine. The brain is
a rule-based intent parser — deterministic, sub-millisecond, and its
failure mode is "I didn't understand", not confidently doing the wrong
thing to a physical machine.

It understands walking, turning, postures, faces, and context-dependent
follow-ups ("faster", "again", "the other way").

For phrasings outside that vocabulary there's an **optional local LLM
fallback** — free, offline, via [Ollama](https://ollama.com):

```bash
python3 tools/companion/run.py --brain cascade
```

Rules always run first; the model is asked only when they come up empty.
Crucially, **a model can never emit motion** — it picks an intent from
the same closed vocabulary the rule parser uses, and the intent→command
translation is identical either way. A model can only choose differently
among moves that already existed and were already clamped. Hostile
outputs (a smuggled velocity, a command string, an injection in a slot)
all validate to "unknown" and emit nothing.

It still isn't a chatbot. It maps what you say onto robot actions.

## Status

Verified: ESP32 compile clean (arduino-esp32 2.0.17), **66% flash / 15%
RAM**. All host and companion tests pass.

**Nothing has run on hardware.** No board exists yet, so servo directions,
tick timing, the OLED flush, and the boot self-test are all reasoned but
unmeasured. Expect a real calibration session when the robot arrives.

- [x] Motion core: IK, gait scheduler, slew limiting, calibration, commands
- [x] Host test harness + offline gait simulator
- [x] Servo bank, serial CLI, boot self-test, latching e-stop
- [x] Walking — crawl gait, forward/back/turn/arc, with a drive watchdog
- [x] Safety envelope with derived (not hardcoded) motion caps
- [x] OLED face — nine expressions, chunked non-blocking flush
- [x] WiFi control surface, HTTP API, built-in web UI, OTA
- [x] NVS-persisted calibration
- [x] Voice assistant: wake word, on-device speech, intent parsing, TTS
- [x] IMU body levelling and ultrasonic obstacle stop — both optional, auto-detected
- [x] Optional local-LLM fallback for out-of-vocabulary phrasings

### What the sensors do and don't do

The IMU **levels the body on uneven ground**. It does not provide balance
recovery and cannot: with 2 DOF per leg there is no foot-position-preserving
attitude control, and the crawl gait is already statically stable. Past 35°
of tilt it stops correcting, because the robot is falling or has been picked
up and flailing makes it worse.

The rangefinder **stops the robot before it walks into something**. An
invalid reading means *unknown*, never *clear* — a sensor that has stopped
answering is not evidence of open space.

> **Wiring warning:** the HC-SR04 echo line drives 5 V and the ESP32 is not
> 5 V tolerant. It **must** be level-shifted (a 1k/2k divider is enough) or
> you will damage the pin.

## Known limitation: 2 DOF per leg

Each leg has two servos, so a foot's reachable set is a **torus** — a 2D
surface in 3D space. You control two of {fore-aft, lateral, height}; the
third follows. That is a property of the mechanism, not something firmware
can fix.

Two consequences, both real:

**There is no strafe.** At neutral stance a leg's achievable foot motion is
purely tangential to its hip circle, so lateral authority is exactly zero.
It is refused with an explanation, never silently turned into forward
motion.

**A foot cannot stay planted while the body translates over it.** The gait
plans directly in (yaw, elevation) so every foot target is exactly
reachable, but the stance path is a shallow arc rather than a straight
line — about 3 mm of lateral bow over a stride, reported as
`maxLateralDeviationMm`.

What you do get: step length, step height, body height, gait frequency and
duty factor as **runtime knobs that mean what they say**, and
forward/backward/turn/arc from one formula instead of four hand-written
poses.

## Credits

Hardware design, CAD, and the original reference firmware are the work of [Dorian Todd](https://github.com/dorianborian/sesame-robot), released under Apache-2.0. Any vendored assets from that project retain their original license and attribution.

## License

Apache-2.0 — see [LICENSE](LICENSE).
