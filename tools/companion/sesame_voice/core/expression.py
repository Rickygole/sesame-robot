"""Dialogue state -> face.

Expression is a pure function of conversation state, NOT a side effect of
motion, and it is emitted on state TRANSITION rather than on action
completion. The face has to react while the robot is still deciding --
if it only changes once the action finishes, the robot looks frozen
during exactly the moments a person is waiting on it, which reads as
broken rather than thoughtful.

PURE: stdlib only.
"""

from typing import Optional

from .dialogue import Context, State
from .intent import Intent, Utterance

# Must match enum class FaceId in
# firmware/sesame/src/core/command.h. Names, not numbers, cross the wire.
FACE_NEUTRAL = "neutral"
FACE_HAPPY = "happy"
FACE_SAD = "sad"
FACE_ANGRY = "angry"
FACE_SLEEP = "sleep"
FACE_LISTENING = "listening"
FACE_THINKING = "thinking"
FACE_CONFUSED = "confused"
FACE_TALKING = "talking"


def face_for_state(state: State) -> str:
    if state is State.LISTENING:
        return FACE_LISTENING
    if state is State.THINKING:
        return FACE_THINKING
    if state is State.TALKING:
        return FACE_TALKING
    if state is State.ASLEEP:
        return FACE_SLEEP
    return FACE_NEUTRAL


def face_for_utterance(utt: Utterance, ctx: Optional[Context] = None) -> str:
    """The face to show in response to a parsed utterance."""
    i = utt.intent
    if i is Intent.UNKNOWN:
        return FACE_CONFUSED
    if i is Intent.SLEEP:
        return FACE_SLEEP
    if i is Intent.GREET or i is Intent.WAKE:
        return FACE_HAPPY
    if i is Intent.SET_FACE and utt.slots.face:
        return utt.slots.face
    if utt.is_motion:
        return FACE_HAPPY
    if i is Intent.STOP:
        return FACE_NEUTRAL
    return FACE_NEUTRAL
