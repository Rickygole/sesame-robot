"""Fuzzy wake-word detection over a stream of transcripts.

PURE: stdlib only (difflib, re). No pyobjc, no mic, no wall-clock -- which
is what makes this headlessly testable by feeding it a plain list of
strings (see tests/test_wake.py). The mic and Speech framework live in
io/mic_speech.py and never leak in here.

Two problems this module exists to solve:

1. On-device speech partials are CUMULATIVE. Each callback delivers the
   whole utterance recognized so far ("hey" -> "hey se" -> "hey sesame"),
   not just what changed. Naively re-scanning the full string every time
   would fire once per partial that happens to contain the wake phrase.
   WakeDetector tracks a consumed-up-to cursor (in words) and only scans
   the unseen tail, with a small lookback so a match cannot be missed by
   landing exactly on the cursor boundary. Once a detector fires it
   latches -- it will not fire again until the caller calls reset() (i.e.
   after the command utterance has been captured and handled) -- so an
   unsettled trailing word being revised mid-partial cannot double-fire.

2. "Hey Sesame" reliably comes back misrecognized. difflib.SequenceMatcher
   against the single string "hey sesame" is not enough: real
   misrecognitions like "hasi sammy" are, character-for-character, *less*
   similar to "hey sesame" than clearly-wrong phrases like "hey there"
   are -- so a single threshold against one canonical phrase cannot
   separate them. Instead this matches against a small set of known-good
   ANCHOR phrases (the canonical phrase plus the misrecognitions actually
   observed), and takes the best match against any anchor. That is also
   why this list is data, not a smarter algorithm: it is the cheapest fix
   for a fundamentally acoustic problem being solved with text.
"""

import difflib
import re
from typing import List, NamedTuple, Optional, Sequence

WAKE_PHRASE = "hey sesame"

# Real misrecognitions of "hey sesame" seen from on-device dictation.
# WAKE_PHRASE itself is included so an exact/near-exact match still wins.
KNOWN_VARIANTS = (
    "hey sesame",
    "hey sam",
    "hasi sammy",
    "he sesame",
    "hey sesamy",
)

# Tuned so every KNOWN_VARIANTS entry scores 1.0 (they're literal anchors)
# while the required false positives ("hey there" ~0.63, "same" ~0.62,
# "sesame street" ~0.55, "the same" ~0.82) all stay below it. See
# tools/companion/tests/test_wake.py for the numbers this was tuned
# against.
DEFAULT_THRESHOLD = 0.85

# Wake phrases are two words. Windows longer than that only add noise
# (a 3-word window pulls in an unrelated neighboring word and can drag
# the ratio up by coincidence), and windows are capped at this length
# when scanning the transcript tail.
_MAX_WINDOW_WORDS = 2


class WakeEvent(NamedTuple):
    """One detected wake word."""
    matched_text: str    # the words that matched, normalized
    remainder: str       # words after the match in the same transcript,
                          # "" if the wake word was the end of it (usual case)
    ratio: float          # best similarity against any anchor, 0..1


def _norm(text: str) -> str:
    """Lowercase, strip punctuation, collapse whitespace."""
    text = text.lower()
    text = re.sub(r"[^\w\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


class WakeDetector(object):
    """Feed it transcripts (partial or final); it emits WakeEvents.

    Stateful. `feed()` is the whole interface -- it never raises, and
    returns None on every call that isn't a fresh detection. Call
    reset() to arm it again for the next listening cycle (typically
    right before you start listening for the next wake word).
    """

    def __init__(self, threshold: float = DEFAULT_THRESHOLD,
                 anchors: Sequence[str] = KNOWN_VARIANTS):
        self.threshold = threshold
        self.anchors = tuple(anchors)
        self._consumed = 0             # words already scanned
        self._last_words: List[str] = []
        self._fired = False

    def reset(self) -> None:
        """Arm the detector for a new utterance / listening cycle."""
        self._consumed = 0
        self._last_words = []
        self._fired = False

    def feed(self, text: Optional[str]) -> Optional[WakeEvent]:
        """Feed one transcript update. Returns a WakeEvent at most once
        per reset() -- see the module docstring for why."""
        if not text or self._fired:
            return None
        words = _norm(text).split()
        if not words:
            return None

        # How much of the previous text this one still agrees with. A
        # cumulative partial extends its predecessor; a revision changes
        # a trailing word. Either way, only words STILL in agreement can
        # be trusted as "already scanned" -- if the cursor was past the
        # point of disagreement, pull it back to the safe boundary.
        common = 0
        limit = min(len(words), len(self._last_words))
        while common < limit and words[common] == self._last_words[common]:
            common += 1
        if common < self._consumed:
            self._consumed = common
        self._last_words = words

        start = max(0, self._consumed - (_MAX_WINDOW_WORDS - 1))
        tail = words[start:]

        best_ratio = 0.0
        best_start = best_end = None
        for size in range(1, min(_MAX_WINDOW_WORDS, len(tail)) + 1):
            for i in range(len(tail) - size + 1):
                window = " ".join(tail[i:i + size])
                for anchor in self.anchors:
                    r = difflib.SequenceMatcher(None, window, anchor).ratio()
                    if r > best_ratio:
                        best_ratio = r
                        best_start, best_end = start + i, start + i + size

        self._consumed = len(words)

        if best_end is not None and best_ratio >= self.threshold:
            matched = " ".join(words[best_start:best_end])
            remainder = " ".join(words[best_end:])
            self._fired = True
            return WakeEvent(matched, remainder, best_ratio)
        return None
