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

## Status

Verified: compiles for ESP32 (arduino-esp32 2.0.17), 22% flash / 6% RAM.
Host tests pass. **Nothing has run on hardware yet** — no board exists, so
servo directions, timing, and the boot self-test are all unverified.

- [x] Motion core: IK, gait scheduler, slew limiting, calibration, command parsing
- [x] Host test harness + offline gait simulator
- [x] Servo bank, serial CLI, boot self-test — `stand`, `rest`, `setjoint`, `estop`
- [ ] Gait: `drive` is withheld pending a parameterization fix (see below)
- [ ] OLED faces, WiFi control surface, NVS-persisted calibration
- [ ] Closed-loop sensing (IMU balance, ultrasonic)
- [ ] Voice / LLM-driven behavior

### Known limitation: 2 DOF per leg

Each leg has two servos, so a foot's reachable set is a **torus** — a 2D
surface in 3D space. You get to control two of {fore-aft, lateral, height};
the third follows. In particular a foot cannot stay planted while the body
translates over it, so some scuffing or body bob is unavoidable. That is a
property of the mechanism, not something firmware can fix.

The gait planner currently emits full 3D foot targets, which are therefore
not exactly reachable. `drive` reports unimplemented rather than running it
— a `stepHeightMm` knob that silently does something else is worse than no
knob. Being fixed by replanning in the two coordinates the mechanism
actually has.

## Credits

Hardware design, CAD, and the original reference firmware are the work of [Dorian Todd](https://github.com/dorianborian/sesame-robot), released under Apache-2.0. Any vendored assets from that project retain their original license and attribution.

## License

Apache-2.0 — see [LICENSE](LICENSE).
