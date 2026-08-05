# src/core -- the purity contract

Everything in this directory is pure C++11: motion math with zero
knowledge of Arduino, WiFi, servos-as-hardware, or wall-clock time. It
must compile and run identically on macOS (`clang++`, the host test
harness in `firmware/host/`) and on the ESP32 target. That portability
is enforced by `-Werror` in the root `Makefile`, not by good intentions,
so treat any build warning in here as a bug.

Rules, no exceptions:

1. No `#include <Arduino.h>` or any transitively-Arduino header. This
   directory does not know an Arduino exists.
2. No `String`. Use `const char*` and fixed `char[N]` buffers only.
3. No `PROGMEM`, `pgm_read_*`, or `F()`.
4. No `millis()`, `micros()`, `delay()`. Time never enters core -- every
   API that needs it takes an explicit `float dt` from the caller.
5. No `Serial`. Functions return values and diagnostic structs; the
   caller (host test, or the ESP32 app layer in a later stage) is
   responsible for printing/logging.
6. **C++11 only.** No inline variables, no `if constexpr`, no
   `std::optional`, no structured bindings, no multi-statement
   `constexpr` function bodies. `enum class`, `static_assert`,
   single-return `constexpr` functions, and `constexpr` arrays are all
   fine and used throughout.
7. Fixed-size types (`int32_t`, `uint8_t`, `float`, ...) in anything
   that gets serialized or crosses a module boundary. No bare `int` in
   wire/NVS-facing structs. Avoid `double` -- it's software-emulated
   and slow on the ESP32's FPU.
8. No dynamic allocation. No `new`, no `std::vector`, no heap-backed
   containers. Fixed arrays sized by `constexpr` only.

If a change to this directory needs any of the above, it belongs in a
higher layer (the future `animator`/`behavior`/ESP32 app code), not
here.

## The testing rule

> For any module that produces commands for a constrained system, at
> least one test must assert those commands satisfy the system's
> constraints.

This is not a style preference. It was written down after the gait
planner shipped a trajectory that was **never once reachable** — every
foot target it produced lay off the leg's reachable surface, for all
2000 simulated ticks — and the entire test suite passed anyway.

It passed because every test stayed *inside* the planner. "Phase wraps
cleanly", "at least three legs in stance", "the trajectory is C⁰
continuous" — all of those are true of a completely infeasible
trajectory. No test ever crossed the boundary between the planner and
the mechanism it was commanding.

The test that would have caught it, and now does, is a round trip:
take the joint angles the planner actually emits, run them forward
through the kinematics, and feed the resulting point back into the IK
solver. Assert the residual is zero. Under a correct design that
assertion is tautological, which is exactly what makes it a good
regression guard — it fires the instant someone reintroduces planning
in coordinates the mechanism does not have.

`test_gait.cpp` and `test_envelope.cpp` both follow this.

## Test across a geometry sweep

Nobody has measured the physical robot. Every link length in
`geometry.h` is a placeholder marked `kPlaceholder*`.

So no test may hardcode those numbers. Run each invariant over a range
of link lengths, neutral yaws, body heights and duty factors instead. A
fix that only works at the made-up dimensions is worthless, and you
would not find out until the robot exists and walks wrong.

Note also that test *inputs* often have to scale with geometry, not just
the assertions: a fixed 40 mm/s command needs a 30.5° half-sweep on a
35 mm leg but only 16.9° on a 55 mm leg, so a hardcoded velocity
silently exceeds the mechanical limit on small robots and the test fails
for a reason that has nothing to do with the bug you were hunting.
