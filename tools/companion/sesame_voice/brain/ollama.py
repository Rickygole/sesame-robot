"""An optional local-LLM brain, talking to a local Ollama server.

Ollama (https://ollama.com) runs a small language model entirely on your
own machine and exposes it over HTTP on localhost:11434 -- no API key,
nothing sent over the network. This module uses stdlib `urllib` only
(no `requests`, no `ollama` package), so it costs nothing to import even
if Ollama is never installed.

Two separate ideas are deliberately kept apart:

1. Talking to the server (`OllamaBrain`): short timeouts, availability
   detection that returns False rather than raising or hanging, and a
   prompt that hands the model the closed vocabulary and nothing else.

2. Trusting what comes back (`validate_response`): a pure function, unit
   tested without a server, that treats the model's JSON as hostile
   input. Unknown intent name, wrong types, out-of-range values,
   malformed JSON, and extra fields all become `Intent.UNKNOWN` -- the
   model can only ever select among intents/slots that already existed
   and were already clamped by `core/plan.py`; it can never smuggle a
   velocity, a duration in the wrong units, or a raw command string
   through as if it were a real field.

PURE this is not -- it imports urllib and reaches out over a socket.
`core/` must never import from here.
"""

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

from .base import Brain
from ..core.dialogue import Context
from ..core.intent import (
    DEFAULT_SPEED_SCALE, Direction, Intent, Modifier, Slots, Utterance)
from ..core.parser import _FACES

DEFAULT_HOST = "http://localhost:11434"

# Small, fast, and commonly the first thing people pull -- see the README
# for why this one specifically. Not load-bearing: if it is not the model
# that happens to be pulled, _pick_model falls back to whatever is.
DEFAULT_MODEL = "llama3.2:1b"

# Short on purpose. A local model that is slow, swapping to disk, or
# simply not running must never wedge the voice loop -- the rule parser
# already answered (that is the only time this brain is even asked), so
# there is nothing waiting on this beyond an optional second opinion.
CONNECT_TIMEOUT_S = 2.0

# Generation deadline.
#
# MEASURED, not guessed. On this machine qwen2.5:7b answers this prompt
# in ~6s and qwen2.5:14b in 6-11s, so the original 6s default meant the
# fallback silently never fired: every request hit the deadline, returned
# UNKNOWN, and the feature looked like it simply did not work.
#
# 20s is chosen so a mid-size model on a laptop actually gets to answer.
# It is deliberately generous because this path is only reached when the
# rule parser has ALREADY failed -- the alternative is not a fast answer,
# it is no answer at all.
#
# For VOICE, 20s of silence is unacceptable UX, so run_voice.py passes a
# shorter deadline. A small model (llama3.2:1b, ~1.3GB) is what makes
# this usable by voice; see the README.
GENERATE_TIMEOUT_S = 20.0

# Voice cannot tolerate a long pause -- the robot appears to have frozen.
# Better to give up and say "I didn't get that" than to stare silently.
VOICE_GENERATE_TIMEOUT_S = 4.0

# Everything a model is allowed to name. UNKNOWN is deliberately excluded
# from the choices offered to the model: a model "choosing" to hedge with
# unknown is indistinguishable from any other invalid answer, and the
# validator below already sends both there.
_VALID_INTENTS = sorted(i.value for i in Intent if i is not Intent.UNKNOWN)
_VALID_DIRECTIONS = {d.value for d in Direction}
_VALID_MODIFIERS = {m.value for m in Modifier}
_VALID_FACES = set(_FACES)
_SLOT_KEYS = {"direction", "speed_scale", "duration_s", "face", "modifier"}
_TOP_KEYS = {"intent", "slots"}

_PROMPT = """You are the language-understanding layer in front of a small \
walking robot toy. You do not control the robot and cannot invent new \
commands: you may only pick ONE intent from the exact list below and fill \
in the listed slots. Anything you output outside the JSON format below is \
discarded, and an invalid answer is treated the same as not understanding \
at all -- so only use a slot value from its list, and leave a slot out (or \
null) if it does not apply.

intents: %(intents)s
directions (only for drive/turn): %(directions)s
modifiers (only for modify): %(modifiers)s
faces (only for set_face): %(faces)s

Reply with STRICT JSON and NOTHING ELSE, in exactly this shape:
{"intent": "<one intent from the list>", "slots": {"direction": null, \
"speed_scale": null, "duration_s": null, "face": null, "modifier": null}}

Never include any key other than "intent" and "slots". Never include a \
slot key other than the five shown. Never invent a value outside the \
lists above.
%(context)s
User said: "%(utterance)s"
JSON:"""


def _context_hint(ctx: Context) -> str:
    """One optional grounding line about what just happened.

    Not required for correctness -- the validator clamps whatever comes
    back regardless -- but it helps the model resolve things like "keep
    going" that the rule parser's fixed grammar did not catch.
    """
    prev = ctx.last_motion if ctx is not None else None
    if prev is None:
        return ""
    direction = prev.slots.direction.value if prev.slots.direction else ""
    return "\nFor context, the robot's last motion was: %s %s." % (
        prev.intent.value, direction)


def _build_prompt(utterance: str, ctx: Context) -> str:
    return _PROMPT % {
        "intents": ", ".join(_VALID_INTENTS),
        "directions": ", ".join(sorted(_VALID_DIRECTIONS)),
        "modifiers": ", ".join(sorted(_VALID_MODIFIERS)),
        "faces": ", ".join(sorted(_VALID_FACES)),
        "context": _context_hint(ctx),
        "utterance": utterance,
    }


def _validate_slots(raw: Any) -> Optional[Slots]:
    """Validate a `slots` value. Returns None if anything about it is
    wrong -- missing entirely is fine (means "no slots"), but a present
    value must match the schema exactly."""
    if raw is None:
        return Slots()
    if not isinstance(raw, dict):
        return None
    if not set(raw.keys()) <= _SLOT_KEYS:
        return None

    direction: Optional[Direction] = None
    d = raw.get("direction")
    if d is not None:
        if not isinstance(d, str) or d not in _VALID_DIRECTIONS:
            return None
        direction = Direction(d)

    speed_scale = raw.get("speed_scale")
    if speed_scale is None:
        speed_scale = DEFAULT_SPEED_SCALE
    else:
        if isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float)):
            return None
        speed_scale = float(speed_scale)
        if not (0.1 <= speed_scale <= 1.0):
            return None

    duration_s: Optional[float] = None
    ds = raw.get("duration_s")
    if ds is not None:
        if isinstance(ds, bool) or not isinstance(ds, (int, float)):
            return None
        duration_s = float(ds)
        if not (0.1 <= duration_s <= 300.0):
            return None

    face: Optional[str] = None
    f = raw.get("face")
    if f is not None:
        if not isinstance(f, str) or f not in _VALID_FACES:
            return None
        face = f

    modifier: Optional[Modifier] = None
    mod = raw.get("modifier")
    if mod is not None:
        if not isinstance(mod, str) or mod not in _VALID_MODIFIERS:
            return None
        modifier = Modifier(mod)

    return Slots(direction=direction, speed_scale=speed_scale,
                 duration_s=duration_s, face=face, modifier=modifier)


def validate_response(text: str) -> Utterance:
    """Turn the model's raw text into a safe Utterance. Never raises.

    This is the entire trust boundary between a language model and the
    rest of the pipeline. It is intentionally hostile-by-default: JSON
    that does not parse, isn't an object, has extra top-level or slot
    keys, names an intent outside the closed vocabulary, or has a
    wrong-typed or out-of-range slot value all become `Intent.UNKNOWN`
    rather than a best effort. There is no partial credit -- a model that
    gets 90% of the shape right and slips in one velocity field, one
    nested object, or one command string is rejected exactly as hard as
    one that returns garbage.
    """
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return Utterance(Intent.UNKNOWN, Slots(), source="ollama")

    if not isinstance(data, dict) or not set(data.keys()) <= _TOP_KEYS:
        return Utterance(Intent.UNKNOWN, Slots(), source="ollama")

    intent_name = data.get("intent")
    if not isinstance(intent_name, str) or intent_name not in _VALID_INTENTS:
        return Utterance(Intent.UNKNOWN, Slots(), source="ollama")

    slots = _validate_slots(data.get("slots"))
    if slots is None:
        return Utterance(Intent.UNKNOWN, Slots(), source="ollama")

    return Utterance(Intent(intent_name), slots, source="ollama")


class OllamaBrain(Brain):
    name = "ollama"

    def __init__(self, host: str = DEFAULT_HOST, model: Optional[str] = None,
                 connect_timeout_s: float = CONNECT_TIMEOUT_S,
                 generate_timeout_s: float = GENERATE_TIMEOUT_S) -> None:
        self.host = host.rstrip("/")
        self._preferred_model = model
        self.connect_timeout_s = connect_timeout_s
        self.generate_timeout_s = generate_timeout_s
        # Why the last interpret() failed, or None. Lets the caller
        # distinguish "the model timed out" from "the model genuinely
        # could not place this utterance" -- see interpret().
        self.last_error: Optional[str] = None

    def _list_models(self) -> List[str]:
        req = urllib.request.Request(self.host + "/api/tags")
        with urllib.request.urlopen(req, timeout=self.connect_timeout_s) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        if not isinstance(data, dict):
            return []
        names = []
        for m in data.get("models", []) or []:
            if isinstance(m, dict) and isinstance(m.get("name"), str):
                names.append(m["name"])
        return names

    def _pick_model(self) -> Optional[str]:
        """The model to ask, or None if Ollama is unreachable or has
        nothing pulled. Never raises -- every failure mode (server not
        running, DNS/connection error, malformed response, timeout)
        collapses to "no model available", which is the common case on
        a machine that never installed Ollama at all."""
        try:
            models = self._list_models()
        except (urllib.error.URLError, OSError, ValueError, TimeoutError):
            return None
        if not models:
            return None
        if self._preferred_model and self._preferred_model in models:
            return self._preferred_model
        if DEFAULT_MODEL in models:
            return DEFAULT_MODEL
        return models[0]

    def available(self) -> bool:
        return self._pick_model() is not None

    def _generate(self, model: str, utterance: str, ctx: Context) -> str:
        payload: Dict[str, Any] = {
            "model": model,
            "prompt": _build_prompt(utterance, ctx),
            "format": "json",  # ask Ollama to constrain output to JSON
            "stream": False,
        }
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.host + "/api/generate", data=body,
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=self.generate_timeout_s) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        response = data.get("response") if isinstance(data, dict) else None
        if not isinstance(response, str):
            raise ValueError("ollama response missing a 'response' string")
        return response

    def interpret(self, utterance: str, ctx: Context) -> Utterance:
        # Belt and suspenders on top of the specific excepts below: a
        # local model is an external process this app does not control,
        # and nothing it does -- slow, unreachable, or malformed -- may
        # ever propagate as an exception into the voice loop.
        try:
            model = self._pick_model()
            if model is None:
                self.last_error = "no model available"
                return Utterance(Intent.UNKNOWN, Slots(), source="ollama")
            raw = self._generate(model, utterance, ctx)
        except Exception as exc:  # noqa: BLE001 -- see the comment above
            # Record WHY. A silent UNKNOWN is indistinguishable from "the
            # model genuinely could not place this", which is how a
            # timeout masquerades as a working feature that just never
            # helps. The caller can surface this.
            name = type(exc).__name__
            if "timeout" in str(exc).lower() or name in ("timeout", "TimeoutError"):
                self.last_error = (
                    "model timed out after %.0fs -- try a smaller model "
                    "(see tools/companion/README.md)" % self.generate_timeout_s)
            else:
                self.last_error = "%s: %s" % (name, exc)
            return Utterance(Intent.UNKNOWN, Slots(), source="ollama")
        self.last_error = None
        return validate_response(raw)
