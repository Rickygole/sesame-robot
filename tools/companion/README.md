# Sesame companion

A voice/text assistant for a simulated Sesame robot -- no hardware, no
network, and (for the typed path) no dependencies at all. It parses what
you say into a small fixed vocabulary of commands, plans robot motion
against the mechanism's real envelope, and drives a mock robot that
enforces the same 500ms drive watchdog the firmware does.

```
sesame_voice/
  core/            PURE stdlib -- intent.py, parser.py, dialogue.py,
                    plan.py, expression.py, wake.py. No pyobjc, no mic,
                    no wall-clock. Fully covered by tests/, including an
                    AST-walking test that fails the build if anything in
                    here ever imports something non-stdlib.
  io/               Not pure: mock_robot.py (simulated robot + ASCII
                    face), tts.py (`/usr/bin/say`), mic_speech.py
                    (AVAudioEngine + on-device Speech recognition).
  session.py         Shared driver: text in -> parse/resolve/plan/act ->
                     reply out. Used by both run.py and run_voice.py.
  run_voice.py       The voice entrypoint.
run.py                The typed console entrypoint.
app/
  Info.plist          Template consumed by build_app.sh.
  build_app.sh         Builds build/Sesame Voice.app.
tests/                 Headless stdlib unittest suite.
```

## Quick start

```bash
make voice        # type commands at a simulated robot -- no setup at all
make voice-test    # the headless test suite (29 core + 10 wake tests)
make voice-listen  # actual voice input -- see "Voice input" below first
```

`make voice` and `make voice-test` need nothing beyond system Python 3
and work exactly as before voice input was added -- this whole
directory's pure `core/` and the console front end are untouched by any
of it.

## Voice input

Say "hey sesame" and then a command ("hey sesame, walk forward"). It
listens on-device, offline -- audio is recognized locally by
`SFSpeechRecognizer` with `requiresOnDeviceRecognition = True` and never
leaves the machine, and replies are spoken with `/usr/bin/say`.

### Setup, once

```bash
make voice-listen
```

The first run:

1. Fetches a Python 3.12 via [`uv`](https://astral.sh/uv) (system Python
   here is 3.9, too old for the Speech/AVFoundation bindings) into
   `tools/companion/.venv`, and installs the pinned `pyobjc` packages
   from `requirements.txt` into it. Nothing is installed into system
   Python.
2. Builds `build/Sesame Voice.app` -- a real, minimal macOS app bundle,
   ad-hoc code-signed. This is required, not cosmetic: see "Why a real
   .app bundle" below.
3. Opens the app. **macOS will show two permission prompts** the first
   time -- Speech Recognition and Microphone. Both must be allowed.
   `tccutil reset Microphone com.sesame.voice` /
   `tccutil reset SpeechRecognition com.sesame.voice` reset them if you
   need to redo this.

After that, `make voice-listen` just rebuilds the bundle (fast, and a
no-op if nothing changed) and reopens it -- the permission grant sticks
because the bundle is always built at the same fixed path and signed
with the same identifier (`com.sesame.voice`).

### Why a real .app bundle, and why `open`

Two separate, both-required pieces of macOS privacy machinery are
involved, and skipping either one fails in a way that gives you almost
no information:

- **An Info.plist is mandatory.** Calling a privacy-sensitive API
  (`SFSpeechRecognizer`, `AVAudioEngine`'s mic tap) from a process with
  no enclosing `.app` bundle -- i.e. `python3 run_voice.py` from a
  shell -- crashes the process instantly with an **uncatchable SIGABRT
  and no Python traceback**. The only trace is in Console/`log show`:
  `NSSpeechRecognitionUsageDescription` and
  `NSMicrophoneUsageDescription` must be present in an Info.plist for
  TCC to show the permission prompt at all instead of just killing you.
  `run_voice.py` checks for this itself (`NSBundle.mainBundle()
  .bundleIdentifier()`) *before* importing anything Speech-related, and
  exits with a plain message instead of letting that crash happen.

- **It must be launched with `open`, not a direct path.** Even once the
  bundle exists, running its binary directly from a shell
  (`build/Sesame\ Voice.app/Contents/MacOS/SesameVoice`) still gets
  killed: TCC attributes the permission grant to the *responsible
  process*, which for anything spawned from a shell is the shell, not
  the app. `open -W "Sesame Voice.app"` makes the app responsible for
  itself, which is the actual unlock. This is exactly what `make
  voice-listen` does; there is no supported direct-invocation path.

`app/build_app.sh` (idempotent, rerun freely) does the rest: it copies
the venv's Python 3.12 binary into `Contents/MacOS/` (a *copy*, because
`codesign` refuses to sign a bundle whose main executable is a symlink),
which loses the interpreter's ability to find its own standard library
relative to its new location -- fixed by an `LSEnvironment` block in
Info.plist (`PYTHONHOME` / `PYTHONPATH`) that `open` applies when it
spawns the process. It then re-signs the whole bundle ad hoc with a
fixed identifier (`codesign -s - --force --identifier com.sesame.voice`)
so the signature is actually bound to *this* Info.plist -- an unsigned
copy's original signature (if any) predates being placed in a bundle at
all, and `codesign -dv` reports that mismatch as `Info.plist=not bound`,
which is a second, independent way to get silently killed at launch even
with the right Info.plist keys present.

(If a different Python distribution ends up installed here -- e.g. a
python.org framework build instead of the statically-linked
`python-build-standalone` that `uv python install` currently resolves to
on this machine -- `build_app.sh` also patches
`@executable_path/../../../../Python3`-style framework load paths with
`install_name_tool`. That branch is untested here because it isn't
needed for the Python this machine's `uv` actually installed; it's
defensive.)

### How wake-word detection works

`core/wake.py` (pure stdlib, `difflib`) is fed the recognizer's
transcript stream and matches it against `"hey sesame"` **and** a
handful of real on-device misrecognitions of it ("hasi sammy", "hey
sam", "he sesame", "hey sesamy", ...) collected from testing, using each
as a fuzzy-match anchor rather than trusting a single canonical phrase.
It's headlessly tested with plain strings -- no mic required -- in
`tests/test_wake.py`, including the cumulative-partial problem: Speech
delivers each partial as the *whole* utterance recognized so far, not a
diff, so a naive re-scan fires once per partial that happens to contain
the wake phrase. `WakeDetector` tracks a consumed cursor and latches
after firing until `reset()`.

`io/tts.py` exposes a `speaking` flag for exactly one reason: `say`'s
audio comes out of the speaker and straight back into the mic. Without
muting recognition while speaking, the robot's own voice would get
transcribed and could re-trigger the wake word -- talking to itself.

## Honest limits

- **Near-field mic only.** Tested against a MacBook Air's built-in mic
  at conversational distance (roughly 1-2m, quiet room). No noise
  suppression, no beamforming, no external mic support. Across a room,
  or with background noise, expect the wake word to be missed far more
  than the fuzzy-match false-positive testing in `test_wake.py` would
  suggest -- that suite proves the *text* matching is sound, not that
  the acoustic recognition upstream of it will hear you.
- **Command vocabulary only, not conversation.** There is no language
  model anywhere in this pipeline. `core/parser.py`'s fixed grammar is
  the whole vocabulary; `core/dialogue.py`'s replies are canned. Ask it
  something outside that vocabulary and it says so ("I didn't get
  that"), which is a deliberate design choice (see `core/parser.py`'s
  docstring), not a gap to be filled in later without also rethinking
  the safety argument that keeps a model out of the motion path.
- **What's actually been verified vs. what hasn't.** Every claim above
  about the bundle, code signing, and the guard's crash-avoidance was
  reproduced on this machine as part of building this feature: the
  SIGABRT with no Info.plist, the silent kill with an unbound Info.plist
  (`Info.plist=not bound` in `codesign -dv`), the "Could not find
  platform independent libraries" failure from copying the interpreter
  without `LSEnvironment`, and the full `run_voice.py` import chain
  constructing successfully (including `MicSpeech`'s tap installation)
  inside the built bundle. What has **not** been verified, because it
  requires a live microphone and a human voice, which this environment
  doesn't have: that the wake word is actually recognized from real
  speech, that the permission-prompt flow is exactly as smooth as
  described, and that session recycling behaves cleanly under real,
  continuous speech rather than silence. Treat the mic-facing half of
  this as reviewed-and-plausible, not proven.
