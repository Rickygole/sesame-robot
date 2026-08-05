"""Rules first, always. A model is asked only when rules give up.

This is the only place an optional LLM brain is ever consulted from, and
the order is not configurable: the rule parser -- the safety-critical
path -- runs first and unconditionally. A working rule match is never
second-guessed by a model, no matter how confident the model's answer
would have been. The model is asked exactly once, only when the rule
parser returned `Intent.UNKNOWN`, and only if it reports itself
available; if it is not installed or not running (the common case) this
behaves identically to plain `rules.py`.
"""

from .base import Brain
from ..core.dialogue import Context
from ..core.intent import Intent, Utterance


class CascadeBrain(Brain):
    name = "cascade"

    def __init__(self, rules: Brain, llm: Brain) -> None:
        self.rules = rules
        self.llm = llm

    def interpret(self, utterance: str, ctx: Context) -> Utterance:
        utt = self.rules.interpret(utterance, ctx)
        if utt.intent is not Intent.UNKNOWN:
            return utt  # rules matched -- the model is never asked

        if not self.llm.available():
            return utt  # no model reachable; behave exactly like rules

        llm_utt = self.llm.interpret(utterance, ctx)
        if llm_utt.intent is Intent.UNKNOWN:
            # The model also could not place it. Keep the rule parser's
            # UNKNOWN (it may carry a helpful suggestion or a "why not"
            # message, e.g. the strafe refusal) rather than the model's.
            return utt
        return llm_utt

    def available(self) -> bool:
        # The rule parser always answers something, so a cascade brain is
        # always "available" in the sense that interpret() will not hang
        # or fail -- whether the model half of it is reachable is a
        # separate, per-call question answered inside interpret().
        return self.rules.available()
