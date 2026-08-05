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
