"""The Brain interface: the one door any intent source walks through.

A brain turns text into an `Utterance` carrying exactly one `Intent` from
the closed vocabulary in `core/intent.py`, plus typed slots. That is the
entire contract. A brain NEVER emits motion, velocities, angles,
durations in raw units, or command strings -- `core/plan.py` is the only
place that happens, is pure and deterministic, and does not know or care
which brain produced the intent it is translating.

`core/parser.py` (wrapped by `rules.py`) is the reference implementation
and the one that ships enabled by default. Anything else -- today just
`ollama.py` -- is optional, and must degrade to `Intent.UNKNOWN` rather
than raise or hang if it cannot answer.

Not PURE: this package (and anything under it) may reach out to a local
process over the network. `core/` must never import from here -- see the
purity test in tests/test_core.py, which would fail the build if it did.
"""

from abc import ABC, abstractmethod

from ..core.dialogue import Context
from ..core.intent import Utterance


class Brain(ABC):
    """Common interface for every intent source."""

    name: str = "brain"

    @abstractmethod
    def interpret(self, utterance: str, ctx: Context) -> Utterance:
        """Turn text into an Utterance. Must never raise.

        `ctx` is passed through for brains that want to ground their
        answer in what just happened (e.g. the last motion), but a brain
        is not required to use it -- the rule parser ignores it entirely.
        """
        raise NotImplementedError

    @abstractmethod
    def available(self) -> bool:
        """Whether this brain can answer right now.

        Must never raise and must never block for long -- called on the
        hot path before every fallback attempt. A brain that cannot
        currently answer (model not installed, server not running)
        returns False here rather than letting `interpret` discover that
        the slow way.
        """
        raise NotImplementedError
