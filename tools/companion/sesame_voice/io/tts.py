"""Text -> speech via `/usr/bin/say`. Free, offline, no dependencies.

Not pure (it shells out and blocks a thread), so it lives in io/ rather
than core/ -- same rule as mock_robot.py and mic_speech.py.

The `speaking` flag is the load-bearing part of this module. `say`'s
output comes out of the laptop's speaker and straight back into its own
microphone, so without a mute signal the robot's own voice would be
picked up by the recognizer and could re-trigger the wake word or get
transcribed as a bogus command -- the assistant talking to itself.
io/mic_speech.py checks `speaking` before feeding audio into the
recognition request, so nothing said here is ever heard by us.
"""

import subprocess
import threading
import time

SAY_BIN = "/usr/bin/say"

# Speech keeps coming out of the speaker for a moment after the `say`
# process exits (audio buffering, room echo), so recognition stays muted
# a little past the end of the utterance rather than unmuting the instant
# the subprocess returns.
DEFAULT_TAIL_S = 0.35


class Speaker(object):
    """Wraps `say`. One utterance at a time; a new speak() cuts off the
    previous one, mirroring how a new drive command supersedes a repeat
    in Session -- the robot should always say the newest thing, not queue
    up everything it was ever asked to say."""

    def __init__(self, voice=None, rate=None, tail_s=DEFAULT_TAIL_S):
        self.voice = voice
        self.rate = rate
        self.tail_s = tail_s
        self._lock = threading.Lock()
        self._proc = None
        self._speaking = threading.Event()
        self._generation = 0  # so a superseded speak() can't clobber a
                               # newer one's flag when it finally exits

    @property
    def speaking(self):
        """True from the moment speech starts until `tail_s` after it
        ends. Mic front ends must not feed audio to the recognizer while
        this is set."""
        return self._speaking.is_set()

    def stop(self):
        """Cut off whatever is currently being said."""
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                self._proc.terminate()

    def speak(self, text, blocking=True):
        """Say `text`. Never raises -- a broken TTS should not take down
        a companion that can otherwise still move the robot and show
        expressions."""
        text = (text or "").strip()
        if not text:
            return

        self.stop()
        cmd = [SAY_BIN]
        if self.voice:
            cmd += ["-v", self.voice]
        if self.rate:
            cmd += ["-r", str(self.rate)]
        cmd.append(text)

        self._generation += 1
        my_generation = self._generation

        def _run():
            self._speaking.set()
            try:
                with self._lock:
                    self._proc = subprocess.Popen(cmd)
                    proc = self._proc
                proc.wait()
            except OSError:
                pass  # no `say` on this machine -- fail silent, not fatal
            finally:
                if self.tail_s > 0:
                    time.sleep(self.tail_s)
                # Only clear the flag if nothing newer superseded us --
                # otherwise a slow-to-exit stopped utterance could unmute
                # recognition mid-way through the one that replaced it.
                if my_generation == self._generation:
                    self._speaking.clear()

        if blocking:
            _run()
        else:
            threading.Thread(target=_run, daemon=True).start()
