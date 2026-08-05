"""The closed intent vocabulary. This is the contract.

Every brain -- the rule parser, a future local LLM, a future API-backed
model -- must emit a member of this enum and nothing else. Anything it
cannot express becomes UNKNOWN.

That constraint is the whole safety design. A brain never emits motion;
it emits an intent. The Intent -> robot command translation lives in
plan.py, is pure and deterministic, and is identical no matter which
brain produced the intent. So swapping in a language model later cannot
introduce a new way to move the robot -- it can only choose differently
among moves that already existed and were already clamped.

PURE: stdlib only. No pyobjc, no sockets, no wall-clock time.
"""

from enum import Enum
from typing import NamedTuple, Optional


class Intent(Enum):
    # Motion
    DRIVE = "drive"          # slots: direction, speed_scale, duration_s
    TURN = "turn"            # slots: direction, speed_scale, duration_s
    STOP = "stop"
    # Postures
    STAND = "stand"
    REST = "rest"
    SIT = "sit"
    # Expression
    SET_FACE = "set_face"    # slots: face
    # Meta / conversational
    MODIFY = "modify"        # slots: modifier -- "faster", "again", "other way"
    GREET = "greet"
    HELP = "help"
    SLEEP = "sleep"
    WAKE = "wake"
    STATUS = "status"
    UNKNOWN = "unknown"      # slots: suggestion (nearest known phrase, if any)


class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    AROUND = "around"        # 180-degree spin


class Modifier(Enum):
    FASTER = "faster"
    SLOWER = "slower"
    AGAIN = "again"
    REVERSE = "reverse"      # "the other way"


# Default motion speed, as a fraction of the mechanism's maximum.
#
# Deliberately NOT 1.0. Starting at the cap leaves "faster" with nothing
# to do, so the first thing a user tries after "walk forward" fails. It
# also means every command runs the servos at their limit, which on a
# 600mAh pack is the fastest route to a brownout.
DEFAULT_SPEED_SCALE = 0.6


class Slots(NamedTuple):
    """Typed payload. Every field optional; meaning depends on the Intent."""
    direction: Optional[Direction] = None
    speed_scale: float = DEFAULT_SPEED_SCALE  # fraction of max speed
    duration_s: Optional[float] = None  # None => continuous until STOP
    face: Optional[str] = None
    modifier: Optional[Modifier] = None
    suggestion: Optional[str] = None  # for UNKNOWN: nearest known phrase


class Utterance(NamedTuple):
    """A parsed utterance: what the robot should do, and what to say back."""
    intent: Intent
    slots: Slots = Slots()
    say: Optional[str] = None    # spoken reply, if any
    confidence: float = 1.0
    source: str = "rules"        # which brain produced this

    @property
    def is_motion(self) -> bool:
        return self.intent in (Intent.DRIVE, Intent.TURN)
