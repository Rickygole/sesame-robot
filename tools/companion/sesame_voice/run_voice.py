#!/usr/bin/env python3
"""Sesame companion -- voice entrypoint.

    hey sesame, <command>  ->  parse -> resolve -> plan -> mock robot
                            ->  spoken reply

Everything after "wake word heard, here is the command text" is exactly
run.py's pipeline (via sesame_voice.session.Session) -- voice is a front
end that only produces text, same as the console.

MUST be launched through the app bundle:

    make voice-listen

Running this file directly -- with system python3 OR with the venv's
python3.12 -- crashes with an uncatchable SIGABRT and NO PYTHON
TRACEBACK the instant Speech touches the microphone. That happens
because privacy-sensitive APIs (SFSpeechRecognizer, AVAudioEngine's mic
tap) require an Info.plist with NSSpeechRecognitionUsageDescription and
NSMicrophoneUsageDescription, and TCC finds that Info.plist by walking up
from the running executable's path to find an enclosing .app bundle --
which only exists at build/Sesame Voice.app, built by app/build_app.sh.
Even running the bundle's own copied interpreter from a plain shell
still crashes: TCC attributes the permission to the *responsible
process*, which is the shell, not the app, unless it's launched with
`open` (see tools/companion/README.md for the full story).

The guard immediately below is what turns that crash into a clear
message instead. It runs BEFORE importing anything that touches Speech.
"""

import os
import sys

_COMPANION_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _COMPANION_DIR)

EXPECTED_BUNDLE_ID = "com.sesame.voice"

NOT_BUNDLED_MESSAGE = """\
Sesame Voice must be launched through the app bundle, not run directly.

  Run this instead:
      make voice-listen

Why: on-device Speech recognition needs an Info.plist with
NSSpeechRecognitionUsageDescription and NSMicrophoneUsageDescription,
which only exists inside build/Sesame Voice.app. Calling Speech without
one crashes the process with an uncatchable SIGABRT and no Python
traceback -- this check exists so you get this message instead of that.
See tools/companion/README.md.
"""


def _bundle_id_or_none():
    """None under system python (no pyobjc -> ImportError) AND under
    the venv's python3.12 run directly (pyobjc present, but there is no
    enclosing .app so NSBundle.mainBundle() has nothing to find) --
    both are handled the same way below."""
    try:
        from Foundation import NSBundle
    except ImportError:
        return None
    return NSBundle.mainBundle().bundleIdentifier()


def _require_bundle():
    if _bundle_id_or_none() == EXPECTED_BUNDLE_ID:
        return
    sys.stderr.write(NOT_BUNDLED_MESSAGE)
    sys.exit(1)


_require_bundle()

# Only past this point is it safe to import anything that touches
# pyobjc / Speech / AVFoundation -- everything above must survive being
# run under plain system python3, which has no pyobjc installed at all.
import time  # noqa: E402

from sesame_voice.core import wake  # noqa: E402
from sesame_voice.io import mic_speech, tts  # noqa: E402
from sesame_voice.io.mock_robot import MockRobot  # noqa: E402
from sesame_voice.session import Session  # noqa: E402

# How long to keep listening for a command after the wake word, and how
# long a gap in speech means "they're done talking" before that.
COMMAND_TIMEOUT_S = 8.0
COMMAND_SILENCE_S = 1.5

BANNER = """\
Sesame Voice -- listening. Say "hey sesame" and then a command, e.g.:
    hey sesame, walk forward       hey sesame, turn left
    hey sesame, stop               hey sesame, look happy

Recognition is on-device and offline -- nothing is sent anywhere.
Near-field mic only; see tools/companion/README.md for known limits.
Ctrl-C to exit.
"""


def _capture_command(mic, wake_remainder,
                      timeout_s=COMMAND_TIMEOUT_S, silence_s=COMMAND_SILENCE_S):
    """Collect the command utterance that follows a wake word.

    `mic.updates()` yields None roughly every 100ms when nothing new was
    recognized (see mic_speech.MicSpeech.updates), which is what lets
    this loop notice silence and a hard timeout without ever blocking
    past either of them.
    """
    mic.restart_session()  # a clean transcript, not one still starting
                            # with "hey sesame"
    text = wake_remainder.strip()
    last_heard_at = time.monotonic() if text else None
    deadline = time.monotonic() + timeout_s

    for update in mic.updates():
        now = time.monotonic()
        if now >= deadline:
            break
        if update is None:
            if last_heard_at is not None and now - last_heard_at >= silence_s:
                break
            continue
        candidate = update.text.strip()
        if candidate:
            text = candidate
            last_heard_at = now
        if update.is_final:
            break

    return text


def main():
    sys.stdout.write(BANNER)
    robot = MockRobot()
    session = Session(robot)
    speaker = tts.Speaker()
    mic = mic_speech.MicSpeech(speaker=speaker)
    mic.start()
    detector = wake.WakeDetector()

    try:
        while True:
            detector.reset()
            wake_event = None
            for update in mic.updates():
                if update is None:
                    continue
                wake_event = detector.feed(update.text)
                if wake_event is not None:
                    break

            sys.stdout.write("\n(wake word heard, ratio=%.2f)\n" % wake_event.ratio)
            robot.send("setface listening")
            robot.render()

            command_text = _capture_command(mic, wake_event.remainder)
            if not command_text:
                sys.stdout.write("(heard nothing)\n")
                speaker.speak("I didn't catch that.")
                mic.restart_session()
                continue

            sys.stdout.write("you (heard) > %s\n" % command_text)
            utt, p, reply = session.handle(command_text)
            sys.stdout.write("sesame > %s\n" % reply)
            robot.render()

            speaker.speak(reply, blocking=True)  # blocking: don't start
                                                  # listening for the next
                                                  # wake word until the
                                                  # mute window has closed
            mic.restart_session()
    except KeyboardInterrupt:
        sys.stdout.write("\n")
    finally:
        mic.close()
        session.close()
        robot.send("stop")
        sys.stdout.write("stopped.\n")


if __name__ == "__main__":
    main()
