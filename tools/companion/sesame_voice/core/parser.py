"""Text -> Intent. The safety-critical command path.

This is NOT a degraded language model. It is a fixed vocabulary with
typed slots, and its failure mode is the honest one: "I didn't understand
that." A language model's failure mode is confidently doing the wrong
thing to a physical machine, which is why no model sits in this path.

The grammar is data (PATTERNS below), not code, so adding a phrasing is a
one-line edit and the test suite can enumerate the whole vocabulary.

PURE: stdlib only.
"""

import re
from typing import List, Optional, Tuple

from .intent import Direction, Intent, Modifier, Slots, Utterance

# Spoken numbers, because speech-to-text writes "three" not "3".
_WORD_NUMBERS = {
    "a": 1, "an": 1, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, "fifteen": 15,
    "twenty": 20, "thirty": 30, "sixty": 60, "half": 0.5,
}

_FACES = (
    "neutral", "happy", "sad", "angry", "sleep",
    "listening", "thinking", "confused", "talking",
)

# Synonyms that map onto a face name.
_FACE_SYNONYMS = {
    "smile": "happy", "cheerful": "happy", "glad": "happy",
    "frown": "sad", "unhappy": "sad", "upset": "sad",
    "mad": "angry", "cross": "angry",
    "sleepy": "sleep", "tired": "sleep",
    "normal": "neutral", "blank": "neutral",
}


def _norm(text: str) -> str:
    """Lowercase, strip punctuation, collapse whitespace."""
    text = text.lower().strip()
    text = re.sub(r"[^\w\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def _extract_duration(text: str) -> Optional[float]:
    """'for three seconds' / 'for 5s' / 'for a minute' -> seconds."""
    m = re.search(r"\bfor\s+(\w+)\s*(seconds?|secs?|s|minutes?|mins?|m)\b", text)
    if not m:
        return None
    raw, unit = m.group(1), m.group(2)
    try:
        value = float(raw)
    except ValueError:
        if raw not in _WORD_NUMBERS:
            return None
        value = float(_WORD_NUMBERS[raw])
    if unit.startswith(("m", "min")) and not unit.startswith("s"):
        value *= 60.0
    return max(0.1, min(value, 300.0))


# (regex, intent, slot-builder). First match wins, so order matters:
# more specific patterns must precede more general ones.
PATTERNS: List[Tuple[str, Intent, object]] = [
    # --- STOP first: it must never be shadowed by anything else. ---
    (r"\b(stop|halt|freeze|stay|whoa|wait|hold on|that s enough|enough)\b",
     Intent.STOP, lambda m, t: Slots()),
    (r"\b(emergency|estop|e stop)\b", Intent.STOP, lambda m, t: Slots()),

    # --- Sleep / wake. MUST precede DRIVE.
    #     "go to sleep" contains "go", so with drive first the robot
    #     would walk forward when told to sleep. Multi-word phrases
    #     containing a motion verb always go above the motion patterns. ---
    (r"\b(go to sleep|sleep now|good night|goodnight|power down|"
     r"take a nap|shut down|^sleep$)\b",
     Intent.SLEEP, lambda m, t: Slots()),
    (r"\b(wake up|good morning|hello again)\b", Intent.WAKE,
     lambda m, t: Slots()),

    # --- Turning. Before DRIVE, since "turn left" contains no drive verb
    #     but "go left" would otherwise match the drive pattern. ---
    (r"\b(spin|turn|rotate)\b.*\b(around|360|180)\b", Intent.TURN,
     lambda m, t: Slots(direction=Direction.AROUND,
                        duration_s=_extract_duration(t))),
    (r"\b(turn|rotate|spin|steer|look)\b.*\bleft\b", Intent.TURN,
     lambda m, t: Slots(direction=Direction.LEFT,
                        duration_s=_extract_duration(t))),
    (r"\b(turn|rotate|spin|steer|look)\b.*\bright\b", Intent.TURN,
     lambda m, t: Slots(direction=Direction.RIGHT,
                        duration_s=_extract_duration(t))),

    # --- Driving. ---
    (r"\b(back up|backwards?|reverse|go back|come back)\b", Intent.DRIVE,
     lambda m, t: Slots(direction=Direction.BACKWARD,
                        duration_s=_extract_duration(t))),
    (r"\b(walk|go|move|drive|run|come|head|forward|forwards|ahead|straight)\b",
     Intent.DRIVE,
     lambda m, t: Slots(direction=Direction.FORWARD,
                        duration_s=_extract_duration(t))),

    # --- Postures. ---
    (r"\b(stand|stand up|get up|up)\b", Intent.STAND, lambda m, t: Slots()),
    (r"\b(sit|sit down)\b", Intent.SIT, lambda m, t: Slots()),
    (r"\b(rest|lie down|lay down|relax|down|crouch)\b", Intent.REST,
     lambda m, t: Slots()),

    # --- Modifiers, resolved against dialogue context. ---
    (r"\b(faster|quicker|speed up|hurry)\b", Intent.MODIFY,
     lambda m, t: Slots(modifier=Modifier.FASTER)),
    (r"\b(slower|slow down|easy|gently)\b", Intent.MODIFY,
     lambda m, t: Slots(modifier=Modifier.SLOWER)),
    (r"\b(again|repeat|do that again|once more)\b", Intent.MODIFY,
     lambda m, t: Slots(modifier=Modifier.AGAIN)),
    (r"\b(other way|opposite|switch|turn around)\b", Intent.MODIFY,
     lambda m, t: Slots(modifier=Modifier.REVERSE)),

    # --- Social / meta. ---
    (r"\b(hello|hi|hey there|greetings|good to see you)\b", Intent.GREET,
     lambda m, t: Slots()),
    (r"\b(help|what can you do|commands|options)\b", Intent.HELP,
     lambda m, t: Slots()),
    (r"\b(status|how are you|report|what s up)\b", Intent.STATUS,
     lambda m, t: Slots()),
]


def _match_face(text: str) -> Optional[str]:
    """'look happy' / 'be sad' / 'smile' -> a face name."""
    for face in _FACES:
        if re.search(r"\b" + face + r"\b", text):
            return face
    for word, face in _FACE_SYNONYMS.items():
        if re.search(r"\b" + word + r"\b", text):
            return face
    return None


def _suggest(text: str) -> Optional[str]:
    """Nearest known phrase, so UNKNOWN can teach the vocabulary.

    Deliberately crude -- shared-word overlap, not edit distance. The goal
    is a helpful nudge ("did you mean 'walk forward'?"), not retrieval.
    """
    known = [
        "walk forward", "go backward", "turn left", "turn right",
        "spin around", "stop", "stand up", "sit down", "rest",
        "look happy", "go to sleep", "faster", "slower",
    ]
    words = set(text.split())
    best, best_score = None, 0
    for phrase in known:
        score = len(words & set(phrase.split()))
        if score > best_score:
            best, best_score = phrase, score
    return best if best_score > 0 else None


def parse(text: str) -> Utterance:
    """Parse one utterance. Never raises; unrecognized input is UNKNOWN."""
    if text is None:
        return Utterance(Intent.UNKNOWN, Slots())
    norm = _norm(text)
    if not norm:
        return Utterance(Intent.UNKNOWN, Slots())

    # Strafe is checked FIRST, before any motion pattern.
    #
    # The mechanism has no lateral authority at all (see GaitCommand in
    # firmware/sesame/src/core/gait.h), so strafe was cut rather than
    # degraded. Without this check "walk sideways" matches the "walk" in
    # the drive pattern and the robot walks FORWARD -- silently doing a
    # different thing than asked, which is exactly the failure this
    # project has been avoiding everywhere else. Say no, and say why.
    if re.search(r"\b(sideways|sidewise|strafe|side step|sidestep|crab|"
                 r"laterally|to the side)\b", norm):
        return Utterance(
            Intent.UNKNOWN, Slots(),
            say="I can't move sideways -- each leg only has two joints, so "
                "there's no sideways push. I can turn and then walk.")

    # A face request is checked before the generic patterns, because
    # "look happy" would otherwise match the "look" in the turn pattern.
    #
    # It requires an explicit face-setting VERB, not merely the presence
    # of a face word. Matching on the word alone made "go to sleep" set
    # the sleep FACE instead of putting the robot to sleep -- the body
    # stayed awake while the eyes closed.
    face_verb = re.search(r"\b(look|looks|be|seem|act|show|make|face|"
                          r"expression)\b", norm)
    if face_verb is not None:
        face = _match_face(norm)
        if face is not None:
            return Utterance(Intent.SET_FACE, Slots(face=face))

    # A bare face word on its own ("happy", "smile") is a face request.
    # "sleep" is excluded: alone it means go to sleep, not show the
    # sleeping face.
    if len(norm.split()) <= 2 and "sleep" not in norm:
        face = _match_face(norm)
        if face is not None:
            return Utterance(Intent.SET_FACE, Slots(face=face))

    for pattern, intent, build in PATTERNS:
        m = re.search(pattern, norm)
        if m:
            return Utterance(intent, build(m, norm))

    return Utterance(Intent.UNKNOWN, Slots(suggestion=_suggest(norm)))
