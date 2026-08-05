"""Optional brains that sit in front of (or beside) the rule parser.

`rules` (default, unchanged) is `core/parser.py` wrapped in the `Brain`
interface. `cascade` additionally asks a local Ollama model, but ONLY
when the rule parser could not classify the utterance at all, and only
if Ollama is actually installed, running, and has a model pulled -- see
`rules.py`, `cascade.py`, and `ollama.py`.

Not PURE: this package may reach out over the network (to localhost) via
`urllib`. `core/` must never import from here -- see the purity test in
`tests/test_core.py`.
"""

from .base import Brain
from .cascade import CascadeBrain
from .ollama import OllamaBrain
from .rules import RulesBrain

BRAIN_NAMES = ("rules", "cascade")


def build_brain(name: str = "rules") -> Brain:
    """Construct a brain by name. Unknown names fall back to 'rules' --
    the default and safety-critical path must always be reachable even
    from a typo, rather than raising and refusing to start."""
    if name == "cascade":
        return CascadeBrain(RulesBrain(), OllamaBrain())
    return RulesBrain()
