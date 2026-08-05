"""Headless tests for the optional brain/ package.

No microphone, no robot, and -- the point of this file -- NO NETWORK.
`OllamaBrain.available()`/`interpret()` are tested against a loopback
port with nothing listening on it, never against an actual Ollama
server, so this suite passes identically whether or not Ollama is
installed on the machine running it.

    python3 -m unittest discover -s tools/companion/tests
"""

import os
import socket
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from sesame_voice.brain import BRAIN_NAMES, build_brain  # noqa: E402
from sesame_voice.brain.base import Brain  # noqa: E402
from sesame_voice.brain.cascade import CascadeBrain  # noqa: E402
from sesame_voice.brain.ollama import OllamaBrain, validate_response  # noqa: E402
from sesame_voice.brain.rules import RulesBrain  # noqa: E402
from sesame_voice.core import parser  # noqa: E402
from sesame_voice.core.dialogue import Context  # noqa: E402
from sesame_voice.core.intent import Direction, Intent, Slots, Utterance  # noqa: E402


def _closed_port() -> int:
    """A TCP port on localhost guaranteed to have nothing listening on
    it: bind, read back the OS-assigned port, then close immediately.
    Used instead of hardcoding 11434 so this test does not depend on --
    or interfere with -- an Ollama server that might actually be running
    on the machine executing the suite."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class _StubBrain(Brain):
    """A scripted brain for exercising CascadeBrain without a network."""

    def __init__(self, utt: Utterance, avail: bool = True):
        self.name = "stub"
        self._utt = utt
        self._avail = avail
        self.calls = 0

    def interpret(self, utterance, ctx):
        self.calls += 1
        return self._utt

    def available(self):
        return self._avail


class TestValidateResponse(unittest.TestCase):
    """The trust boundary between a model's raw text and the rest of the
    pipeline. Every case here must be tested WITHOUT a server -- that is
    the entire point of factoring validation out as a pure function."""

    def test_valid_minimal(self):
        u = validate_response('{"intent": "stop"}')
        self.assertIs(u.intent, Intent.STOP)
        self.assertEqual(u.source, "ollama")

    def test_valid_with_slots(self):
        u = validate_response(
            '{"intent": "drive", '
            '"slots": {"direction": "forward", "duration_s": 4.5}}')
        self.assertIs(u.intent, Intent.DRIVE)
        self.assertIs(u.slots.direction, Direction.FORWARD)
        self.assertAlmostEqual(u.slots.duration_s, 4.5)

    def test_valid_set_face(self):
        u = validate_response(
            '{"intent": "set_face", "slots": {"face": "happy"}}')
        self.assertIs(u.intent, Intent.SET_FACE)
        self.assertEqual(u.slots.face, "happy")

    def test_malformed_json(self):
        for text in ("", "not json", "{intent: drive}", "[1, 2, 3]",
                      '{"intent": "drive"', "null", "42", '"just a string"'):
            u = validate_response(text)
            self.assertIs(u.intent, Intent.UNKNOWN, text)

    def test_unknown_intent_name(self):
        u = validate_response('{"intent": "dance_party"}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_model_cannot_say_unknown_either(self):
        # UNKNOWN is excluded from what the model is invited to name; a
        # model that names it anyway is treated the same as any other
        # invalid answer.
        u = validate_response('{"intent": "unknown"}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_velocity_smuggled_in_rejected(self):
        # A model trying to emit motion directly, not just an intent.
        u = validate_response(
            '{"intent": "drive", '
            '"slots": {"direction": "forward", "vx_mm_s": 999}}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_command_string_as_intent_rejected(self):
        u = validate_response('{"intent": "drive forward fast"}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_nested_object_in_slot_rejected(self):
        u = validate_response(
            '{"intent": "drive", "slots": {"direction": {"value": "forward"}}}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_wrong_type_rejected(self):
        cases = [
            '{"intent": "drive", "slots": {"duration_s": "five"}}',
            '{"intent": "drive", "slots": {"speed_scale": "fast"}}',
            '{"intent": "drive", "slots": {"speed_scale": true}}',
            '{"intent": "set_face", "slots": {"face": 7}}',
            '{"intent": 5}',
        ]
        for text in cases:
            self.assertIs(validate_response(text).intent, Intent.UNKNOWN, text)

    def test_out_of_range_rejected(self):
        cases = [
            '{"intent": "drive", "slots": {"duration_s": 10000}}',
            '{"intent": "drive", "slots": {"duration_s": -1}}',
            '{"intent": "drive", "slots": {"speed_scale": 5.0}}',
            '{"intent": "drive", "slots": {"speed_scale": 0.0}}',
        ]
        for text in cases:
            self.assertIs(validate_response(text).intent, Intent.UNKNOWN, text)

    def test_injection_attempt_in_slot_value_rejected(self):
        cases = [
            '{"intent": "set_face", '
            '"slots": {"face": "happy\\"; rm -rf / #"}}',
            '{"intent": "drive", '
            '"slots": {"direction": "forward; stop; drive 999 0 0 0"}}',
            '{"intent": "turn", "slots": {"direction": "$(reboot)"}}',
        ]
        for text in cases:
            self.assertIs(validate_response(text).intent, Intent.UNKNOWN, text)

    def test_extra_top_level_field_rejected(self):
        u = validate_response(
            '{"intent": "stop", "slots": {}, "confidence": 1.0}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_extra_slot_field_rejected(self):
        u = validate_response(
            '{"intent": "drive", '
            '"slots": {"direction": "forward", "note": "go now"}}')
        self.assertIs(u.intent, Intent.UNKNOWN)

    def test_never_raises(self):
        hostile = [
            None, "", "\x00\x01", "{" * 5000, "a" * 20000,
            '{"intent": null}', '{"slots": {}}', "{}",
        ]
        for text in hostile:
            try:
                validate_response(text)
            except Exception as exc:  # pragma: no cover - failure path
                self.fail("validate_response raised on %r: %r" % (text, exc))


class TestOllamaAvailability(unittest.TestCase):
    """Nothing here ever talks to an actual Ollama server."""

    def test_available_false_when_nothing_listening(self):
        brain = OllamaBrain(host="http://127.0.0.1:%d" % _closed_port(),
                             connect_timeout_s=1.0)
        self.assertFalse(brain.available())

    def test_available_is_fast_not_hanging(self):
        import time
        brain = OllamaBrain(host="http://127.0.0.1:%d" % _closed_port(),
                             connect_timeout_s=1.0)
        start = time.monotonic()
        brain.available()
        self.assertLess(time.monotonic() - start, 5.0)

    def test_interpret_returns_unknown_when_unreachable(self):
        brain = OllamaBrain(host="http://127.0.0.1:%d" % _closed_port(),
                             connect_timeout_s=1.0, generate_timeout_s=1.0)
        u = brain.interpret("do a backflip", Context())
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertEqual(u.source, "ollama")

    def test_interpret_never_raises_when_unreachable(self):
        brain = OllamaBrain(host="http://127.0.0.1:%d" % _closed_port(),
                             connect_timeout_s=1.0, generate_timeout_s=1.0)
        try:
            brain.interpret("anything at all", Context())
        except Exception as exc:  # pragma: no cover - failure path
            self.fail("interpret() raised: %r" % exc)


class TestRulesBrain(unittest.TestCase):
    def test_matches_parser_exactly(self):
        brain = RulesBrain()
        for text in ("walk forward", "turn left", "xyzzy plugh", ""):
            self.assertEqual(brain.interpret(text, Context()),
                             parser.parse(text))

    def test_always_available(self):
        self.assertTrue(RulesBrain().available())


class TestCascadeBrain(unittest.TestCase):
    def test_rules_win_when_they_match(self):
        llm = _StubBrain(Utterance(Intent.DRIVE, Slots(), source="ollama"))
        cascade = CascadeBrain(RulesBrain(), llm)
        u = cascade.interpret("walk forward", Context())
        self.assertIs(u.intent, Intent.DRIVE)
        self.assertEqual(u.source, "rules")
        self.assertEqual(llm.calls, 0, "the model must not be asked at all")

    def test_falls_back_to_llm_only_on_unknown(self):
        llm = _StubBrain(Utterance(Intent.STAND, Slots(), source="ollama"))
        cascade = CascadeBrain(RulesBrain(), llm)
        u = cascade.interpret("xyzzy plugh frobnicate", Context())
        self.assertIs(u.intent, Intent.STAND)
        self.assertEqual(u.source, "ollama")
        self.assertEqual(llm.calls, 1)

    def test_llm_unavailable_behaves_like_rules_alone(self):
        llm = _StubBrain(Utterance(Intent.STAND, Slots()), avail=False)
        cascade = CascadeBrain(RulesBrain(), llm)
        rules_only = parser.parse("xyzzy plugh frobnicate")
        u = cascade.interpret("xyzzy plugh frobnicate", Context())
        self.assertIs(u.intent, rules_only.intent)
        self.assertEqual(llm.calls, 0,
                         "must not call interpret() on an unavailable brain")

    def test_llm_also_unknown_keeps_rules_reply(self):
        llm = _StubBrain(Utterance(Intent.UNKNOWN, Slots(), source="ollama"))
        cascade = CascadeBrain(RulesBrain(), llm)
        u = cascade.interpret("spin cartwheels", Context())
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertEqual(u.source, "rules")  # not "ollama"
        self.assertIsNotNone(u.slots.suggestion)  # rules' suggestion kept

    def test_cascade_with_real_unreachable_ollama_is_a_noop(self):
        # End-to-end (minus a network): the default construction a user
        # gets from `--brain cascade` on a machine without Ollama running.
        ollama = OllamaBrain(host="http://127.0.0.1:%d" % _closed_port(),
                              connect_timeout_s=1.0)
        cascade = CascadeBrain(RulesBrain(), ollama)
        for text in ("walk forward", "xyzzy plugh frobnicate", "turn left"):
            self.assertEqual(cascade.interpret(text, Context()),
                             parser.parse(text))


class TestBuildBrain(unittest.TestCase):
    def test_default_is_rules(self):
        self.assertIsInstance(build_brain(), RulesBrain)
        self.assertIsInstance(build_brain("rules"), RulesBrain)

    def test_cascade_wraps_rules_and_ollama(self):
        brain = build_brain("cascade")
        self.assertIsInstance(brain, CascadeBrain)
        self.assertIsInstance(brain.rules, RulesBrain)
        self.assertIsInstance(brain.llm, OllamaBrain)

    def test_all_advertised_names_buildable(self):
        for name in BRAIN_NAMES:
            build_brain(name)  # must not raise


class TestSessionDefaultUnchanged(unittest.TestCase):
    """The whole point of the flag: absent it, nothing changes."""

    def test_session_defaults_to_rules_brain(self):
        from sesame_voice.session import Session
        from sesame_voice.io.mock_robot import MockRobot

        session = Session(MockRobot())
        try:
            self.assertIsInstance(session.brain, RulesBrain)
        finally:
            session.close()

    def test_session_handle_matches_pre_brain_pipeline(self):
        from sesame_voice.session import Session
        from sesame_voice.io.mock_robot import MockRobot
        from sesame_voice.core import dialogue, plan

        session = Session(MockRobot())
        try:
            for text in ("walk forward", "turn left", "stop", "gibberish xyz"):
                utt, p, reply = session.handle(text)
                expected_utt = dialogue.resolve(parser.parse(text), Context())
                self.assertEqual(utt.intent, expected_utt.intent, text)
                self.assertEqual(utt.slots, expected_utt.slots, text)
                self.assertEqual(p, plan.plan(expected_utt), text)
                self.assertEqual(reply, dialogue.reply_for(expected_utt), text)
        finally:
            session.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
