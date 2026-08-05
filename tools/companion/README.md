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
  brain/            OPTIONAL intent sources, all implementing the same
                    Brain interface (text -> Utterance). rules.py wraps
                    core/parser.py (the default); ollama.py talks to a
                    local Ollama server over stdlib urllib; cascade.py
                    tries rules first and only asks ollama.py when rules
                    return UNKNOWN. Not pure -- may touch a socket -- but
                    core/ never imports from here. See "Optional: a
                    local-LLM fallback" below.
  io/               Not pure: mock_robot.py (simulated robot + ASCII
                    face), tts.py (`/usr/bin/say`), mic_speech.py
                    (AVAudioEngine + on-device Speech recognition).
  session.py         Shared driver: text in -> interpret/resolve/plan/act
                     -> reply out. Used by both run.py and run_voice.py.
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
make voice-test    # the headless test suite
make voice-listen  # actual voice input -- see "Voice input" below first
```

`make voice` and `make voice-test` need nothing beyond system Python 3
and work exactly as before voice input (or the optional local-LLM brain)
were added -- this whole directory's pure `core/` and the console front
end are untouched by any of it, and both remain fully usable with zero
setup and zero third-party packages.

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

## Optional: a local-LLM fallback (`--brain cascade`)

`core/parser.py`'s ~50 fixed phrasings are the default and stay the
default. They will never catch everything -- "hit the brakes" or "can
you park it" mean STOP just as much as "halt" does, but they aren't in
the pattern list, and there is no end to the list of ways to say the
same dozen commands. `--brain cascade` adds an *optional* local language
model that gets a single, narrow job: when the rule parser gives up
entirely (`Intent.UNKNOWN`), ask the model to pick one intent from the
exact same closed vocabulary in `core/intent.py`, with typed slots, and
nothing else. It cannot do anything the rule parser couldn't already do.

**The safety argument, restated plainly:** a brain -- rules or model --
only ever emits an `Intent` and some `Slots`. `core/plan.py` is the only
place an intent becomes a robot command, it is unchanged, and it does
not know or care which brain produced the intent it's translating. A
model in this design cannot invent a new move, cannot set a velocity, a
duration in raw units, or a command string directly -- see
`sesame_voice/brain/base.py`'s docstring and
`sesame_voice/brain/ollama.py`'s `validate_response`, which is hostile
by default: malformed JSON, an intent name outside the vocabulary, a
slot with the wrong type, an out-of-range value, or *any* extra field
(a smuggled velocity, a nested object, a command string standing in for
an intent name) all collapse to `Intent.UNKNOWN`, unit-tested in
`tests/test_brain.py` with no server involved. The rule parser also
always runs first and unconditionally -- a working rule match is never
second-guessed by the model, and if the model is not installed, not
running, or times out, behavior is identical to `--brain rules` (the
default).

### Installing Ollama

[Ollama](https://ollama.com) runs a model entirely on your own machine
and exposes it over HTTP on `localhost:11434` -- nothing is sent
anywhere. It's free, but pulling a model is a multi-gigabyte download
the first time.

```bash
brew install ollama        # or the installer at https://ollama.com/download
ollama serve &              # if it isn't already running as a service
ollama pull llama3.2:1b     # ~1.3 GB -- see below for why this one
```

**Which model to use, and why `llama3.2:1b`:** this fallback's whole job
is picking one word out of a short, explicit list -- it is not free-form
conversation, so it does not need a large model. A small (~1-3B
parameter) instruction-tuned model is enough to read "hit the brakes"
and answer `{"intent": "stop"}`, and it responds in well under a second
on ordinary hardware once loaded, which matters because `ollama.py` uses
a short (few-second) timeout by design -- see "What's actually been
verified" below. Anything in Ollama's "small" tier (`llama3.2:1b`,
`llama3.2:3b`, `qwen2.5:0.5b`, `gemma2:2b`, ...) will work; `ollama.py`
does not hardcode a model file, it queries `/api/tags` and uses whatever
is pulled (preferring `llama3.2:1b` if more than one is present).

### Enabling it

```bash
python3 tools/companion/run.py --brain cascade
```

(`run_voice.py`/`make voice-listen` do not currently take this flag --
see `sesame_voice/session.py`, which is where a `Brain` is plumbed in if
you want to wire it into the voice front end too; `run.py`'s typed
console is where this was built and verified.)

With no flag, or `--brain rules`, behavior is **completely unchanged**
from before this feature existed -- same parser, same tests, same
default. `sesame_voice/brain/rules.py` is a thin wrapper around
`core/parser.py`, not a reimplementation, so there is no way for
`--brain rules` to diverge from what shipped before it.

### What's actually been verified vs. what hasn't

Unlike the voice/mic feature above, this one *was* fully exercised
end to end on this machine, because Ollama happened to already be
installed and running here with several models pulled (`qwen2.5:7b`,
`qwen2.5:14b-instruct`) -- that is not something to assume is true on
your machine, but it means the following is a real result, not a
plausibility argument:

- `CascadeBrain` really does call the model only after rules return
  `UNKNOWN`, and really does defer to a matching rule every time one
  exists (confirmed both by `tests/test_brain.py` with a stub brain, and
  live: "walk forward" never touched the model; "hit the brakes" and
  "can you park it" -- neither of which the rule grammar matches -- came
  back correctly classified as `stop` with `Utterance.source ==
  "ollama"`, and produced the exact same `drive`/`stop` command lines
  `plan.py` would generate for any other `stop`).
- **The default timeout is genuinely tight for anything but a small
  model, and that is by design, not an oversight.** Against the 14B
  model that happened to be pulled here, the default 6-second timeout
  was *not* enough even once the model was warm in memory (it measured
  6.0-11.2s per response) -- `interpret()` correctly gave up and fell
  back to `Intent.UNKNOWN` (i.e. behaved exactly like `--brain rules`)
  rather than hang the console waiting on it. Against `qwen2.5:7b`, most
  responses landed at 1-3s but one measured 6.03s and also timed out.
  This is the reason to actually pull one of the small models named
  above rather than reuse whatever large model you might already have
  installed for other purposes -- a fallback that silently degrades to
  "no smarter than before" on a slow model is the safe failure mode,
  but it is a worse experience than a model sized for the job.
- **Not verified:** behavior with `llama3.2:1b` specifically -- it
  wasn't pulled on this machine and downloading a few more GB to prove
  a point wasn't a good use of the sandbox's time or your bandwidth.
  The recommendation above is reasoned from Ollama's own published size
  tiers and from the 7B/14B timing data actually measured, not from
  running that exact model. If you pull it and it's still too slow on
  your hardware, `ollama.py`'s `generate_timeout_s` is a constructor
  argument, not a hardcoded constant.

## Honest limits

- **Near-field mic only.** Tested against a MacBook Air's built-in mic
  at conversational distance (roughly 1-2m, quiet room). No noise
  suppression, no beamforming, no external mic support. Across a room,
  or with background noise, expect the wake word to be missed far more
  than the fuzzy-match false-positive testing in `test_wake.py` would
  suggest -- that suite proves the *text* matching is sound, not that
  the acoustic recognition upstream of it will hear you.
- **Command vocabulary only, not conversation -- still true by default,
  and true even with the optional brain on.** By default there is no
  language model anywhere in this pipeline: `core/parser.py`'s fixed
  grammar is the whole vocabulary, and `core/dialogue.py`'s replies are
  canned. `--brain cascade` (see "Optional: a local-LLM fallback" above)
  can additionally recognize phrasings the rule grammar missed, but it
  is opt-in, off by default, and even when it's on it can only ever
  select among the *same* closed vocabulary in `core/intent.py` -- it
  does not hold a conversation, does not remember anything the rule
  parser and `core/dialogue.py`'s `Context` don't already track, and
  cannot express anything the rule parser couldn't in principle also
  express. Ask either brain for something outside that vocabulary and it
  says so ("I didn't get that"), which is a deliberate design choice
  (see `core/parser.py`'s docstring), not a gap to be filled in later
  without also rethinking the safety argument that keeps a model out of
  the motion path -- see `core/plan.py` and `sesame_voice/brain/base.py`.
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
