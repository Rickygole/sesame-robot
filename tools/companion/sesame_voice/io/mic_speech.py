"""Microphone -> streaming transcripts, via on-device Speech recognition.

NOT pure -- this is the one module that touches pyobjc, AVFoundation, and
Speech. core/ (specifically core/wake.py) never imports this module or
anything in it; it only ever sees the plain strings this yields. That
split is what keeps the wake-word logic testable with `python3 -m
unittest` and no microphone -- see core/wake.py's docstring.

Only reachable from inside the app bundle: `import Speech` requires
pyobjc, which lives in tools/companion/.venv, and actually USING Speech
(not just importing it) requires an Info.plist with
NSSpeechRecognitionUsageDescription or the process is killed with an
uncatchable SIGABRT. run_voice.py's guard, which runs before this module
is ever imported, is what prevents that -- see its module docstring.

API surface here (SFSpeechRecognizer, SFSpeechAudioBufferRecognitionRequest,
AVAudioEngine.installTapOnBus_bufferSize_format_block_, ...) was checked
against the actual installed pyobjc-framework-Speech / -AVFoundation
stubs in tools/companion/.venv, not assumed from memory.
"""

import queue
import threading
import time

import AVFoundation
import Speech
from Foundation import NSLocale

ON_DEVICE_LOCALE = "en-US"

# SFSpeechRecognitionTask has a known-ish ~60s practical ceiling on a
# single request. It did not fire in a 95s silence test, but that was
# not a real test -- actual speech pushing the recognizer may behave
# differently, and there is no documented guarantee either way. Rather
# than trust the untested case, the session is recycled on a fixed timer
# regardless of activity. The audio engine and mic tap are NOT torn down
# across a recycle, only the recognizer-side request/task -- so there is
# no audio gap, just a transcript reset, which callers (wake.WakeDetector)
# already treat as the start of a new utterance.
SESSION_MAX_S = 45.0

_TAP_BUS = 0
_TAP_BUFFER_SIZE = 1024


class TranscriptUpdate(object):
    """One recognition callback. `text` is CUMULATIVE for a given
    session (see core/wake.py) until `is_final` is True, at which point
    the session's recognizer has committed to it."""
    __slots__ = ("text", "is_final")

    def __init__(self, text, is_final):
        self.text = text
        self.is_final = is_final

    def __repr__(self):
        return "TranscriptUpdate(%r, is_final=%r)" % (self.text, self.is_final)


class MicSpeech(object):
    """Continuous mic capture -> a stream of TranscriptUpdate.

        mic = MicSpeech(speaker=my_tts_speaker)
        mic.start()
        for update in mic.updates():
            ...
        mic.close()

    `speaker` is optional; if given, its `.speaking` flag (io/tts.py)
    mutes the mic tap so the robot's own `say` output is never fed back
    into recognition -- without this, the assistant hears itself talk.
    """

    def __init__(self, speaker=None, locale_id=ON_DEVICE_LOCALE,
                 session_max_s=SESSION_MAX_S):
        self._speaker = speaker
        self._session_max_s = session_max_s
        self._queue = queue.Queue()
        self._lock = threading.Lock()
        self._closed = False

        locale = NSLocale.alloc().initWithLocaleIdentifier_(locale_id)
        self._recognizer = Speech.SFSpeechRecognizer.alloc().initWithLocale_(locale)
        if self._recognizer is None:
            raise RuntimeError(
                "SFSpeechRecognizer unavailable for locale %r" % locale_id)
        if not self._recognizer.supportsOnDeviceRecognition():
            # Refuse rather than silently fall back to a server-side
            # recognizer -- that would send audio off this machine,
            # which is the one guarantee this whole design makes.
            raise RuntimeError(
                "on-device recognition not supported for %r on this "
                "machine" % locale_id)

        self._engine = AVFoundation.AVAudioEngine.alloc().init()
        self._input_node = self._engine.inputNode()
        self._tap_format = self._input_node.outputFormatForBus_(_TAP_BUS)

        self._request = None
        self._task = None
        self._session_started_at = 0.0

        self._install_tap()

    # --- setup -----------------------------------------------------------
    def _install_tap(self):
        def on_buffer(buffer, when):
            # Runs on a real-time audio thread: no logging, no blocking,
            # the cheapest possible check and nothing else. Muted while
            # the robot is speaking -- see io/tts.py's module docstring
            # for why that flag exists at all.
            if self._speaker is not None and self._speaker.speaking:
                return
            with self._lock:
                req = self._request
            if req is not None:
                req.appendAudioPCMBuffer_(buffer)

        self._input_node.installTapOnBus_bufferSize_format_block_(
            _TAP_BUS, _TAP_BUFFER_SIZE, self._tap_format, on_buffer)

    def start(self):
        ok, error = self._engine.startAndReturnError_(None)
        if not ok:
            raise RuntimeError("AVAudioEngine failed to start: %s" % error)
        self._new_session()

    # --- session recycling -------------------------------------------------
    def _new_session(self):
        """Tear down the current recognition request/task (if any) and
        start a fresh one. Safe to call at any time, including from
        updates()'s recycle timer or right after a wake word fires."""
        with self._lock:
            old_request, old_task = self._request, self._task
            self._request = None
        if old_request is not None:
            old_request.endAudio()
        if old_task is not None:
            old_task.cancel()

        request = Speech.SFSpeechAudioBufferRecognitionRequest.alloc().init()
        request.setShouldReportPartialResults_(True)
        request.setRequiresOnDeviceRecognition_(True)

        def on_result(result, error):
            if error is not None or result is None:
                # Recognition errors -- including a session hitting an
                # internal time limit -- surface here as a callback, not
                # an exception. There's nothing useful to do with the
                # error itself; the periodic recycle in updates() will
                # replace this session shortly regardless.
                return
            text = result.bestTranscription().formattedString()
            self._queue.put(TranscriptUpdate(text, bool(result.isFinal())))

        task = self._recognizer.recognitionTaskWithRequest_resultHandler_(
            request, on_result)

        with self._lock:
            self._request = request
            self._task = task
        self._session_started_at = time.monotonic()

    def updates(self, poll_s=0.1):
        """Yield TranscriptUpdate forever, until close() -- and yield
        None roughly every `poll_s` when nothing new has been recognized,
        so a caller with its own timeout (run_voice.py's command-capture
        silence detector) gets control back regularly instead of blocking
        indefinitely inside this generator waiting for speech that may
        never come.

        A session recycle is invisible to the caller beyond that: the
        transcript resets to empty and starts over, exactly what a
        genuinely new utterance looks like, which is why
        wake.WakeDetector needs no special-casing for it."""
        while not self._closed:
            if time.monotonic() - self._session_started_at > self._session_max_s:
                self._new_session()
            try:
                yield self._queue.get(timeout=poll_s)
            except queue.Empty:
                yield None

    def restart_session(self):
        """Force a fresh recognition session now, e.g. right after a
        wake word fires so the command utterance starts from an empty
        transcript rather than one that still contains "hey sesame"."""
        self._new_session()

    def close(self):
        self._closed = True
        with self._lock:
            request, task = self._request, self._task
            self._request = None
            self._task = None
        if request is not None:
            request.endAudio()
        if task is not None:
            task.cancel()
        try:
            self._input_node.removeTapOnBus_(_TAP_BUS)
        except Exception:
            pass
        if self._engine.isRunning():
            self._engine.stop()
