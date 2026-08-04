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

## Status

🚧 Early development. Building toward:

- [ ] Baseline: servo control, OLED faces, WiFi control surface
- [ ] Real inverse kinematics with smooth, non-blocking gait interpolation
- [ ] Closed-loop sensing (IMU balance, ultrasonic obstacle avoidance)
- [ ] Voice / LLM-driven behavior

## Credits

Hardware design, CAD, and the original reference firmware are the work of [Dorian Todd](https://github.com/dorianborian/sesame-robot), released under Apache-2.0. Any vendored assets from that project retain their original license and attribution.

## License

Apache-2.0 — see [LICENSE](LICENSE).
