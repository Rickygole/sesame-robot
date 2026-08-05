"""Headless tests for the fuzzy wake-word detector.

No microphone, no Speech framework -- WakeDetector is fed plain strings,
exactly the way it will be fed real (but unpredictable) transcripts from
io/mic_speech.py.

    python3 -m unittest discover -s tools/companion/tests
"""

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from sesame_voice.core.wake import WakeDetector  # noqa: E402

# Real misrecognitions of "hey sesame" observed from on-device dictation.
_TRUE_POSITIVES = (
    "hey sesame", "hey, sesame!", "hasi sammy", "hey sam", "he sesame",
    "hey sesamy",
)

# Phrases that sound adjacent but must never wake the robot.
_FALSE_POSITIVES = (
    "hey there", "same", "sesame street",
)


class TestWakeDetectorPositives(unittest.TestCase):
    def test_known_variants_trigger(self):
        for phrase in _TRUE_POSITIVES:
            d = WakeDetector()
            ev = d.feed(phrase)
            self.assertIsNotNone(ev, phrase)
            self.assertGreaterEqual(ev.ratio, d.threshold, phrase)

    def test_variant_embedded_in_a_longer_sentence(self):
        d = WakeDetector()
        ev = d.feed("okay hey sesame walk forward")
        self.assertIsNotNone(ev)
        self.assertEqual(ev.remainder, "walk forward")

    def test_variant_with_command_attached(self):
        d = WakeDetector()
        ev = d.feed("hasi sammy stop")
        self.assertIsNotNone(ev)
        self.assertEqual(ev.remainder, "stop")


class TestWakeDetectorNegatives(unittest.TestCase):
    def test_lookalikes_do_not_trigger(self):
        for phrase in _FALSE_POSITIVES:
            d = WakeDetector()
            ev = d.feed(phrase)
            self.assertIsNone(ev, phrase)

    def test_empty_and_none_never_raise_or_trigger(self):
        for text in ("", "   ", None):
            d = WakeDetector()
            self.assertIsNone(d.feed(text))

    def test_unrelated_speech_does_not_trigger(self):
        for phrase in ("what time is it", "turn left please",
                        "the weather today", "good morning everyone"):
            d = WakeDetector()
            self.assertIsNone(d.feed(phrase), phrase)


class TestCumulativePartials(unittest.TestCase):
    """Speech partials are cumulative -- each callback repeats everything
    recognized so far. A naive re-scan fires once per partial that
    contains the wake phrase; WakeDetector must fire at most once."""

    def test_growing_partial_fires_once(self):
        d = WakeDetector()
        partials = ["hey", "hey se", "hey sesa", "hey sesame",
                    "hey sesame walk", "hey sesame walk forward",
                    "hey sesame walk forward please"]
        hits = [d.feed(p) for p in partials]
        fired = [h for h in hits if h is not None]
        self.assertEqual(len(fired), 1, hits)

    def test_trailing_word_revision_after_match_does_not_double_fire(self):
        # STT commonly revises the last word or two as more audio arrives
        # even after a match has already been found upstream of it.
        d = WakeDetector()
        partials = ["hey sesame", "hey sesame walk", "hey sesame walked",
                    "hey sesame walk forward"]
        results = [d.feed(p) for p in partials]
        fired = [r for r in results if r is not None]
        self.assertLessEqual(len(fired), 1, results)

    def test_reset_rearms_for_the_next_wake(self):
        d = WakeDetector()
        first = d.feed("hey sesame stop")
        self.assertIsNotNone(first)
        self.assertIsNone(d.feed("hey sesame sit down"))  # latched
        d.reset()
        second = d.feed("hey sesame sit down")
        self.assertIsNotNone(second)

    def test_never_raises_on_garbage_partial_stream(self):
        d = WakeDetector()
        for text in ("", None, "   ", "a" * 3000, "!!!", "123 456"):
            d.feed(text)  # must not raise


if __name__ == "__main__":
    unittest.main(verbosity=2)
