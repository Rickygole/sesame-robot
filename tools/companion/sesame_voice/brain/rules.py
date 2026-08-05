"""The default brain: the existing rule parser, wrapped in `Brain`.

This does not change `core/parser.py` at all -- it is a thin adapter so
`core.parser.parse` can sit next to (and always win against) an optional
model in `cascade.py`. Always available, never asked to guess: it either
matches a known phrasing or honestly says `Intent.UNKNOWN`.
"""

from .base import Brain
from ..core import parser
from ..core.dialogue import Context
from ..core.intent import Utterance


class RulesBrain(Brain):
    name = "rules"

    def interpret(self, utterance: str, ctx: Context) -> Utterance:
        # ctx is deliberately ignored: the rule parser is context-free by
        # design (see parser.py's docstring). Modifier resolution against
        # ctx happens one layer up, in dialogue.resolve -- same as before
        # brains existed at all.
        return parser.parse(utterance)

    def available(self) -> bool:
        return True
