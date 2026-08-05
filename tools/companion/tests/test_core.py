"""Headless tests for the companion's pure core.

No microphone, no robot, no network, no third-party packages. Everything
that decides what the robot does is testable this way, which is the same
contract src/core/ has in the firmware.

    python3 -m unittest discover -s tools/companion/tests
"""

import ast
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from sesame_voice.core import dialogue, expression, parser, plan  # noqa: E402
from sesame_voice.core.dialogue import Context  # noqa: E402
from sesame_voice.core.intent import (  # noqa: E402
    DEFAULT_SPEED_SCALE, Direction, Intent, Modifier)
from sesame_voice.io.mock_robot import MockRobot  # noqa: E402


class TestParser(unittest.TestCase):
    def test_motion_phrasings(self):
        for text in ("walk forward", "go forward", "move ahead", "go straight",
                     "run forward", "walk forwards please"):
            self.assertIs(parser.parse(text).intent, Intent.DRIVE, text)
            self.assertIs(parser.parse(text).slots.direction,
                          Direction.FORWARD, text)

    def test_backward(self):
        for text in ("go backward", "back up", "reverse", "go back"):
            u = parser.parse(text)
            self.assertIs(u.intent, Intent.DRIVE, text)
            self.assertIs(u.slots.direction, Direction.BACKWARD, text)

    def test_turns(self):
        self.assertIs(parser.parse("turn left").slots.direction, Direction.LEFT)
        self.assertIs(parser.parse("turn right").slots.direction,
                      Direction.RIGHT)
        self.assertIs(parser.parse("spin around").slots.direction,
                      Direction.AROUND)

    def test_stop_is_never_shadowed(self):
        # STOP must win against anything it is combined with -- it is the
        # one command that must never be misread.
        for text in ("stop", "stop walking", "stop going forward", "halt",
                     "freeze", "whoa", "that's enough"):
            self.assertIs(parser.parse(text).intent, Intent.STOP, text)

    def test_duration_words_and_digits(self):
        self.assertAlmostEqual(
            parser.parse("walk forward for three seconds").slots.duration_s, 3.0)
        self.assertAlmostEqual(
            parser.parse("go forward for 5 seconds").slots.duration_s, 5.0)
        self.assertAlmostEqual(
            parser.parse("walk forward for a minute").slots.duration_s, 60.0)

    def test_faces(self):
        self.assertEqual(parser.parse("look happy").slots.face, "happy")
        self.assertEqual(parser.parse("be sad").slots.face, "sad")
        self.assertEqual(parser.parse("smile").slots.face, "happy")

    def test_unknown_suggests_when_it_can(self):
        # Shares "spin" with a known phrase, so it can nudge.
        u = parser.parse("spin cartwheels")
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertIsNotNone(u.slots.suggestion)

    def test_sleep_phrases_do_not_trigger_motion(self):
        # "go to sleep" contains "go". With the drive pattern first, the
        # robot walked FORWARD when told to sleep.
        for text in ("go to sleep", "good night", "take a nap", "shut down"):
            u = parser.parse(text)
            self.assertIs(u.intent, Intent.SLEEP, text)
            for line in plan.plan(u).lines:
                self.assertFalse(line.startswith("drive"), text)

    def test_strafe_is_refused_not_silently_reinterpreted(self):
        # The mechanism has no lateral authority, so strafe was cut.
        # Quietly walking FORWARD instead would be the robot doing a
        # different thing than it was asked.
        for text in ("walk sideways", "strafe left", "move to the side",
                     "side step right", "crab walk"):
            u = parser.parse(text)
            self.assertIs(u.intent, Intent.UNKNOWN, text)
            self.assertIsNotNone(u.say, text)
            self.assertIn("sideways", u.say.lower(), text)
            # And it must emit no motion whatsoever.
            self.assertEqual(plan.plan(u).lines, [], text)

    def test_unknown_with_no_overlap_says_nothing_rather_than_guessing(self):
        # "scoot over there" shares no word with any known phrase. Offering
        # an unrelated suggestion would be worse than admitting ignorance.
        u = parser.parse("xyzzy plugh frobnicate")
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertIsNone(u.slots.suggestion)
        self.assertIn("didn't get that", dialogue.reply_for(u))

    def test_never_raises(self):
        for text in ("", "   ", None, "!!!", "\n", "a" * 5000, "123 456"):
            parser.parse(text)  # must not raise


class TestDialogue(unittest.TestCase):
    def test_modifier_without_context_does_not_invent_motion(self):
        # The robot must never decide to move on its own.
        u = dialogue.resolve(parser.parse("faster"), Context())
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertIn("nothing to change", u.say.lower())

    def test_faster_then_slower(self):
        ctx = Context()
        walk = parser.parse("walk forward")
        ctx = dialogue.advance(walk, ctx)
        faster = dialogue.resolve(parser.parse("faster"), ctx)
        self.assertIs(faster.intent, Intent.DRIVE)
        self.assertGreater(faster.slots.speed_scale, DEFAULT_SPEED_SCALE)

        ctx = dialogue.advance(faster, ctx)
        slower = dialogue.resolve(parser.parse("slower"), ctx)
        self.assertLess(slower.slots.speed_scale, faster.slots.speed_scale)

    def test_speed_saturates_without_error(self):
        ctx = dialogue.advance(parser.parse("walk forward"), Context())
        for _ in range(10):
            u = dialogue.resolve(parser.parse("faster"), ctx)
            if u.intent is Intent.DRIVE:
                ctx = dialogue.advance(u, ctx)
        u = dialogue.resolve(parser.parse("faster"), ctx)
        self.assertIs(u.intent, Intent.UNKNOWN)
        self.assertIn("fastest", u.say.lower())

    def test_other_way_reverses(self):
        ctx = dialogue.advance(parser.parse("turn left"), Context())
        u = dialogue.resolve(parser.parse("the other way"), ctx)
        self.assertIs(u.slots.direction, Direction.RIGHT)

    def test_asleep_ignores_motion(self):
        ctx = dialogue.advance(parser.parse("go to sleep"), Context())
        self.assertTrue(ctx.asleep)
        after = dialogue.advance(parser.parse("walk forward"), ctx)
        # A passing conversation must not start a sleeping robot.
        self.assertTrue(after.asleep)
        self.assertIsNone(after.last_motion)


class TestPlan(unittest.TestCase):
    def test_envelope_matches_firmware_formula(self):
        env = plan.Envelope()
        # Cross-check against the simulator: at vx=40 the sim reports
        # strideMm 42.857.
        import math
        a = 40.0 * env.duty / (2 * env.gait_freq_hz * env.stance_radius_mm)
        self.assertAlmostEqual(2 * a * env.stance_radius_mm, 42.857, places=2)
        self.assertGreater(env.max_vx_mm_s, 0.0)
        self.assertLess(env.max_vx_mm_s, 200.0)

    def test_drive_is_a_stream_not_a_packet(self):
        p = plan.plan(parser.parse("walk forward"))
        self.assertTrue(p.repeat)
        self.assertIsNone(p.duration_s)  # until STOP
        self.assertTrue(p.lines[0].startswith("drive "))

    def test_backward_is_negative_vx(self):
        p = plan.plan(parser.parse("back up"))
        self.assertLess(float(p.lines[0].split()[1]), 0.0)

    def test_turn_right_is_negative_omega(self):
        p = plan.plan(parser.parse("turn right"))
        self.assertLess(float(p.lines[0].split()[2]), 0.0)

    def test_spin_around_terminates(self):
        # "Spin around" must not spin forever.
        p = plan.plan(parser.parse("spin around"))
        self.assertIsNotNone(p.duration_s)
        self.assertGreater(p.duration_s, 0.0)

    def test_never_exceeds_envelope(self):
        env = plan.Envelope()
        for text in ("walk forward", "back up", "turn left", "turn right",
                     "spin around"):
            u = parser.parse(text)
            u = u._replace(slots=u.slots._replace(speed_scale=99.0))
            p = plan.plan(u, env)
            if not p.lines or not p.lines[0].startswith("drive"):
                continue
            parts = p.lines[0].split()
            self.assertLessEqual(abs(float(parts[1])), env.max_vx_mm_s + 1e-6)
            self.assertLessEqual(abs(float(parts[2])),
                                 env.max_omega_deg_s + 1e-6)

    def test_non_motion_intents_emit_no_motion(self):
        for text in ("hello", "help", "status", "gibberish xyzzy"):
            p = plan.plan(parser.parse(text))
            for line in p.lines:
                self.assertFalse(line.startswith("drive"), text)


class TestMockRobotWatchdog(unittest.TestCase):
    def test_watchdog_stops_an_abandoned_drive(self):
        r = MockRobot()
        r.send("drive 40 0 27 8", now=0.0)
        self.assertEqual(r.mode, "drive")
        self.assertFalse(r.tick(now=0.3))   # still fresh
        self.assertTrue(r.tick(now=1.0))    # abandoned
        self.assertEqual(r.vx, 0.0)
        self.assertEqual(r.mode, "stand")

    def test_keepalive_prevents_watchdog(self):
        r = MockRobot()
        t = 0.0
        for _ in range(20):
            r.send("drive 40 0 27 8", now=t)
            t += 0.2
            self.assertFalse(r.tick(now=t))
        self.assertEqual(r.mode, "drive")

    def test_rejects_unknown_face(self):
        r = MockRobot()
        self.assertFalse(r.send("setface banana")["ok"])
        self.assertEqual(r.face, "neutral")


class TestExpression(unittest.TestCase):
    def test_unknown_shows_confused(self):
        u = parser.parse("xyzzy plugh")
        self.assertEqual(expression.face_for_utterance(u), "confused")

    def test_face_names_match_firmware_enum(self):
        """Names cross the wire, so they must match FaceId exactly."""
        header = os.path.join(
            os.path.dirname(_HERE), "..", "..",
            "firmware/sesame/src/core/command.h")
        header = os.path.normpath(header)
        if not os.path.exists(header):
            self.skipTest("firmware header not found")
        with open(header) as fh:
            text = fh.read()
        start = text.index("enum class FaceId")
        body = text[start:text.index("};", start)]
        names = set()
        for line in body.splitlines():
            line = line.strip()
            if "=" in line and not line.startswith("//"):
                name = line.split("=")[0].strip()
                if name not in ("Count",) and name.isidentifier():
                    names.add(name.lower())
        for face in ("neutral", "happy", "sad", "angry", "sleep",
                     "listening", "thinking", "confused", "talking"):
            self.assertIn(face, names,
                          "companion uses '%s' but FaceId has no such member"
                          % face)


class TestPurityContract(unittest.TestCase):
    """core/ must stay importable with stdlib only.

    Enforced mechanically rather than by good intentions -- the same role
    -Werror plays for the firmware's purity contract.
    """

    ALLOWED = {
        "enum", "typing", "re", "math", "dataclasses", "collections",
        "itertools", "functools", "abc", "__future__", "difflib",
    }

    def test_core_imports_only_stdlib(self):
        core_dir = os.path.join(os.path.dirname(_HERE), "sesame_voice", "core")
        for fname in os.listdir(core_dir):
            if not fname.endswith(".py"):
                continue
            path = os.path.join(core_dir, fname)
            with open(path) as fh:
                tree = ast.parse(fh.read(), path)
            for node in ast.walk(tree):
                mod = None
                if isinstance(node, ast.Import):
                    mod = node.names[0].name.split(".")[0]
                elif isinstance(node, ast.ImportFrom):
                    if node.level > 0:
                        continue  # relative import within core/
                    mod = (node.module or "").split(".")[0]
                if mod and mod not in self.ALLOWED:
                    self.fail("%s imports non-stdlib '%s' -- core/ must stay "
                              "pure and headlessly testable" % (fname, mod))


if __name__ == "__main__":
    unittest.main(verbosity=2)
