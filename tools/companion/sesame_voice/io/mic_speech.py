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
from Foundation import NSDate, NSLocale, NSRunLoop


def _pump(seconds):
    """Run the Cocoa run loop for `seconds`, then return.

    THIS IS LOAD-BEARING, not a nicety. Both the authorization callback
    and every SFSpeechRecognitionTask result are delivered as callbacks
    on the main run loop. If the main thread sits in a plain blocking
    wait -- queue.get(), Event.wait(), sleep() -- those callbacks can
    never be delivered, so:

      * the permission dialog never appears,
      * no transcript ever arrives,
      * and macOS eventually reports the app as "not responding",
        because from its point of view a process that never pumps its
        event loop is hung. Which it is.
    """
    NSRunLoop.currentRunLoop().runUntilDate_(
        NSDate.dateWithTimeIntervalSinceNow_(seconds))

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

    def request_authorization(self, timeout_s=30.0):
        """Ask for Speech Recognition access and WAIT for the answer.

        This is not optional and it is not implicit. Without an explicit
        requestAuthorization_ call macOS never shows the consent dialog,
        the status stays notDetermined forever, and recognition silently
        yields nothing at all -- the app looks like it is listening and
        simply never hears a word. That failure mode cost real debugging
        time, hence the loud return value and the caller that acts on it.

        Returns (authorized: bool, status_name: str).
        """
        names = {0: "notDetermined", 1: "denied", 2: "restricted",
                 3: "authorized"}
        done = threading.Event()
        result = {"status": None}

        def handler(status):
            result["status"] = int(status)
            done.set()

        Speech.SFSpeechRecognizer.requestAuthorization_(handler)
        # PUMP the run loop rather than blocking on `done`. The handler
        # above is delivered as a run-loop callback, so a blocking wait
        # here deadlocks: the thing we are waiting for can only happen on
        # the thread we just parked. Bounded, so a user who walks away
        # from the dialog does not wedge the process forever.
        deadline = time.monotonic() + timeout_s
        while not done.is_set() and time.monotonic() < deadline:
            _pump(0.1)

        status = result["status"]
        if status is None:
            status = int(Speech.SFSpeechRecognizer.authorizationStatus())
        return status == 3, names.get(status, str(status))

    def microphone_authorized(self):
        """Mic access is a SEPARATE grant from speech recognition.

        Having one and not the other produces two different silent
        failures, so they are reported separately.
        """
        names = {0: "notDetermined", 1: "restricted", 2: "denied",
                 3: "authorized"}
        status = int(
            AVFoundation.AVCaptureDevice.authorizationStatusForMediaType_("soun"))
        return status == 3, names.get(status, str(status))

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
            # Pump the run loop instead of blocking on the queue. The
            # recognition callback that FILLS this queue is itself a
            # run-loop callback, so queue.get(timeout=...) would park the
            # only thread that can deliver the thing being waited for --
            # the queue would stay empty forever and nothing would ever
            # be heard.
            _pump(poll_s)
            try:
                yield self._queue.get_nowait()
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
