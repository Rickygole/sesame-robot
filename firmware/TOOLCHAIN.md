# Toolchain

Arduino IDE cannot pin dependency versions inside a repository, so this
file is the source of truth instead. If something behaves strangely, check
your versions against this list first.

## Open the project

Open **`firmware/sesame/sesame.ino`** in Arduino IDE.

The sketch folder must stay named `sesame`, matching `sesame.ino`. Arduino
compiles `.h`/`.cpp` beside the `.ino` flat, and everything under
`sesame/src/` **recursively** — which is what lets the module tree survive
without flattening. Do not add a second `.ino`; multiple `.ino` files are
concatenated alphabetically with auto-injected prototypes, which produces
baffling errors.

## 1. ESP32 board support

Arduino IDE → **Settings** → *Additional boards manager URLs*:

```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Then **Boards Manager** → search `esp32` → **"esp32 by Espressif Systems"**
→ version dropdown → **`2.0.17`**.

> **Install 2.0.17, not the latest 3.x.** On 2.0.17 ESP32Servo 3.0.9 takes
> the older `ledcSetup` code path. arduino-esp32 3.x reworked LEDC
> attachment, which is where ESP32Servo issue #103 lives (writing one servo
> disturbs others). Revisit after the gait work is stable.

If the dropdown offers only 3.x, use the legacy index instead:
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

## 2. Libraries

| Library | Version | Needed from | Note |
|---|---|---|---|
| **ESP32Servo** | **3.0.9** | stage 1a | **DO NOT INSTALL** — vendored at `sesame/src/vendor/ESP32Servo/` |
| Adafruit SSD1306 | 2.5.13 | stage 1b | accept the dependency prompt |
| Adafruit GFX Library | 1.11.11 | stage 1b | pulled in as a dependency |
| Adafruit BusIO | 1.16.1 | stage 1b | pulled in as a dependency |
| ArduinoJson | 7.2.1 | stage 1b | Benoit Blanchon |

Stage 1a needs **no** Library Manager installs at all.

> Installing ESP32Servo via Library Manager alongside the vendored copy
> risks duplicate symbols and ambiguous resolution. `src/vendor/version_guard.h`
> fails the build loudly if the wrong version is picked up. See
> `sesame/src/vendor/ESP32Servo/VENDORING.md`.

## 3. Tools menu

Board: **ESP32 Dev Module**.

| Setting | Value |
|---|---|
| Upload Speed | 921600 *(drop to 115200 if uploads fail)* |
| CPU Frequency | 240 MHz (WiFi/BT) |
| Flash Frequency | 80 MHz |
| Flash Mode | QIO |
| Flash Size | 4 MB (32 Mb) |
| Partition Scheme | Default 4 MB with spiffs |
| Core Debug Level | Info |
| PSRAM | Disabled *(WROOM-32 has none)* |
| **Arduino Runs On** | **Core 1** — enforced by an `#error` in `sesame.ino` |
| **Events Run On** | **Core 0** |
| Erase All Flash Before Sketch Upload | Disabled *(Enabled for the first upload only, to clear stale NVS)* |

Also set **Settings → Compiler warnings → All**. Arduino gives no
per-project flags, so this partially recovers the `-Wall -Wextra` that the
host build enforces.

## 4. Upload troubleshooting

**If upload fails or the board boot-loops, unplug the servos on wire
channels 0 and 1 first.** Those sit on GPIO 15 and GPIO 2, which are ESP32
boot strapping pins. Firmware cannot fix this — straps are latched before
any code runs. Check this before suspecting cables or drivers.

Other cases:
- **No port** — the DevKitC-32E uses a CP210x bridge; install Silicon Labs' VCP driver.
- **Upload still fails** — hold BOOT while uploading, or lower upload speed.
- OTA is planned for stage 1b specifically to sidestep the strapping-pin dance on every iteration.

## 5. Host-side tests (no board required)

The pure motion core under `sesame/src/core/` has no Arduino dependency
and compiles on macOS/Linux directly:

```
make test     # build and run all unit tests
make sim      # dump a simulated gait to build/gait.csv
make clean
```

Needs only `clang++` (or `g++`) and `make` — no Arduino toolchain, no
PlatformIO. `-Werror` is deliberate: it turns the purity rules in
`sesame/src/core/README.md` into hard build failures rather than
conventions.
