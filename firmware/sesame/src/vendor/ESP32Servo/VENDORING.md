# Vendored: ESP32Servo 3.0.9

## Provenance

| | |
|---|---|
| Upstream | https://github.com/madhephaestus/ESP32Servo |
| Version | **3.0.9** (`library.properties`: `version=3.0.9`) |
| Source | `https://github.com/madhephaestus/ESP32Servo/archive/refs/tags/3.0.9.tar.gz` |
| License | LGPL-2.1-or-later (see the header of each source file) |
| Authors | Kevin Harrington, John K. Bennett. Original `Servo.h` by Michael Margolis, 2009. |

Only the four files under upstream `src/` are vendored. Examples, docs, and
build scripts are not included.

## Why this is vendored rather than installed

The firmware **requires** exactly 3.0.9. Version 3.1.0 reworked LEDC/MCPWM
channel allocation and introduced a bug where writing to one servo can
disturb others ([issue #103](https://github.com/madhephaestus/ESP32Servo/issues/103)).
On a quadruped that presents as the wrong leg moving — a failure that looks
exactly like a wiring fault and is miserable to diagnose.

Arduino IDE's Library Manager **cannot pin a library version in the repo**.
There is also no usable version macro to test against: `ESP32Servo.h` defines
`ESP32_Servo_VERSION 1`, a stale artifact unchanged since 2017, and the real
version lives only in `library.properties`, which the preprocessor never sees.

Vendoring changes the problem from *detecting* the wrong version to making it
*impossible*: the version is now a git-tracked artifact of this repository.

> **Do not install ESP32Servo via Library Manager.** A Library Manager copy
> alongside this one risks duplicate symbols and ambiguous resolution.

## Modifications from upstream

LGPL-2.1 requires that changes be stated. There are exactly two, both
mechanical:

| File | Line | Before | After |
|---|---|---|---|
| `ESP32PWM.cpp` | 8 | `#include <ESP32PWM.h>` | `#include "ESP32PWM.h"` |
| `ESP32Servo.cpp` | 53 | `#include <ESP32Servo.h>` | `#include "ESP32Servo.h"` |

Angle-bracket includes resolve against the library search path, which does not
include a sketch subfolder. Quoted includes resolve relative to the including
file. No functional code was altered.

## Licensing note

This repository is Apache-2.0. These four files remain LGPL-2.1-or-later and
are not relicensed. For a personally-built robot this raises no practical
issue. If binaries are ever *distributed*, LGPL's relink provision applies to
a statically linked build — at that point either ship the object files or move
this back out to an external library.

## Verifying the vendored copy

`../version_guard.h` carries a compile-time fingerprint (`MAX_SERVOS == 16`,
which is 16 across all 3.0.x and 20 from 3.1.0 onward). Additionally, the
firmware runs a boot-time self-test that writes a distinct pulse width to each
channel and reads all eight back — that tests the actual symptom of issue #103
rather than trusting a macro, and survives any future library change.
