"""Conversation state, and resolving modifiers against it.

This module is what makes the assistant feel conversational without a
language model. "faster", "again", "the other way" mean nothing on their
own -- they only mean something relative to what was just said. Holding
last_intent and rewriting those into concrete intents is cheap and buys
most of the perceived intelligence.

PURE: stdlib only. No wall-clock -- callers pass elapsed time in.
"""

from enum import Enum
from typing import NamedTuple, Optional

from .intent import Direction, Intent, Modifier, Slots, Utterance

_SPEED_STEP = 0.25
_MIN_SCALE = 0.25
_MAX_SCALE = 1.0

_REVERSE = {
    Direction.FORWARD: Direction.BACKWARD,
    Direction.BACKWARD: Direction.FORWARD,
    Direction.LEFT: Direction.RIGHT,
    Direction.RIGHT: Direction.LEFT,
    Direction.AROUND: Direction.AROUND,
}


class State(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    ACTING = "acting"
    TALKING = "talking"
    ASLEEP = "asleep"


class Context(NamedTuple):
    """Immutable conversation context. Transitions return a new one."""
    state: State = State.IDLE
    last_motion: Optional[Utterance] = None  # last DRIVE/TURN, for "again"
    speed_scale: float = 1.0
    asleep: bool = False

    def with_state(self, state: State) -> "Context":
        return self._replace(state=state)


def resolve(utt: Utterance, ctx: Context) -> Utterance:
    """Rewrite a context-dependent utterance into a concrete one.

    MODIFY intents are meaningless alone. If there is nothing to modify,
    this returns UNKNOWN with a helpful reply rather than guessing --
    silently inventing a direction would be the robot deciding to move on
    its own, which is never acceptable.
    """
    if utt.intent is not Intent.MODIFY:
        return utt

    mod = utt.slots.modifier
    prev = ctx.last_motion

    if prev is None:
        return Utterance(
            Intent.UNKNOWN, Slots(),
            say="There's nothing to change yet. Try 'walk forward' first.")

    if mod is Modifier.FASTER or mod is Modifier.SLOWER:
        step = _SPEED_STEP if mod is Modifier.FASTER else -_SPEED_STEP
        scale = max(_MIN_SCALE, min(prev.slots.speed_scale + step, _MAX_SCALE))
        if abs(scale - prev.slots.speed_scale) < 1e-6:
            edge = "fastest" if step > 0 else "slowest"
            return Utterance(Intent.UNKNOWN, Slots(),
                             say="Already at my %s." % edge)
        return prev._replace(slots=prev.slots._replace(speed_scale=scale),
                             say="Okay, %s." % mod.value)

    if mod is Modifier.AGAIN:
        return prev._replace(say="Again.")

    if mod is Modifier.REVERSE:
        direction = prev.slots.direction
        if direction is None:
            return Utterance(Intent.UNKNOWN, Slots(),
                             say="I'm not sure which way you mean.")
        return prev._replace(
            slots=prev.slots._replace(direction=_REVERSE[direction]),
            say="Other way.")

    return utt


def advance(utt: Utterance, ctx: Context) -> Context:
    """Fold a resolved utterance into the context."""
    new = ctx

    if utt.intent is Intent.SLEEP:
        return new._replace(state=State.ASLEEP, asleep=True)
    if utt.intent is Intent.WAKE:
        return new._replace(state=State.IDLE, asleep=False)

    # While asleep, only WAKE is honored -- everything else leaves the
    # context untouched so a passing conversation cannot start the robot.
    if ctx.asleep:
        return ctx

    if utt.is_motion:
        new = new._replace(last_motion=utt, state=State.ACTING,
                           speed_scale=utt.slots.speed_scale)
    elif utt.intent is Intent.STOP:
        new = new._replace(state=State.IDLE)
    elif utt.intent is Intent.UNKNOWN:
        new = new._replace(state=State.IDLE)
    else:
        new = new._replace(state=State.IDLE)
    return new


def reply_for(utt: Utterance) -> str:
    """A short spoken reply. Canned, and deliberately so.

    These are NOT conversation. Without a language model the assistant
    has a command vocabulary and a handful of social responses, and the
    README says so plainly rather than letting anyone discover it.
    """
    if utt.say:
        return utt.say

    i = utt.intent
    if i is Intent.DRIVE:
        d = utt.slots.direction
        return "Walking %s." % (d.value if d else "forward")
    if i is Intent.TURN:
        d = utt.slots.direction
        if d is Direction.AROUND:
            return "Spinning around."
        return "Turning %s." % (d.value if d else "left")
    if i is Intent.STOP:
        return "Stopping."
    if i is Intent.STAND:
        return "Standing up."
    if i is Intent.SIT or i is Intent.REST:
        return "Lying down."
    if i is Intent.SLEEP:
        return "Good night."
    if i is Intent.WAKE:
        return "Good morning."
    if i is Intent.SET_FACE:
        return "Okay."
    if i is Intent.GREET:
        return "Hello. Say 'help' to hear what I can do."
    if i is Intent.STATUS:
        return "I'm here and listening."
    if i is Intent.HELP:
        return ("I can walk forward or backward, turn left or right, spin "
                "around, stop, stand, sit, and change my face. Say 'faster', "
                "'slower', or 'again' to adjust what I just did.")
    if utt.slots.suggestion:
        return "I didn't get that. Did you mean '%s'?" % utt.slots.suggestion
    return "I didn't get that."
