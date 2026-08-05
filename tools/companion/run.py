#!/usr/bin/env python3
"""Sesame companion -- console driver.

Type what you would say. The full pipeline runs: parse -> resolve against
conversation context -> plan robot commands -> send to a simulated robot
-> render its face. No microphone, no permissions, no network, no robot.

Voice input is a later stage and is deliberately a swappable front end:
it only produces text, and everything downstream of that is already here
and unit-tested.

    python3 tools/companion/run.py
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sesame_voice.core import dialogue, expression, parser, plan  # noqa: E402
from sesame_voice.core.dialogue import Context, State  # noqa: E402
from sesame_voice.core.intent import Intent  # noqa: E402
from sesame_voice.io.mock_robot import MockRobot  # noqa: E402

KEEPALIVE_HZ = 5.0  # comfortably inside the robot's 500ms watchdog


class Session(object):
    def __init__(self, robot):
        self.robot = robot
        self.ctx = Context()
        self.env = plan.Envelope()
        self._repeat_lines = []
        self._repeat_until = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._keepalive, daemon=True)
        self._thr.start()

    def _keepalive(self):
        """Resend the active drive command.

        "Walk until I say stop" is a command stream, not one packet. The
        robot's watchdog stops it if this thread dies -- which is the
        point: a crashed companion cannot leave the robot walking.
        """
        period = 1.0 / KEEPALIVE_HZ
        while not self._stop.wait(period):
            now = time.monotonic()
            with self._lock:
                if self._repeat_until is not None and now >= self._repeat_until:
                    self._repeat_lines = []
                    self._repeat_until = None
                    self.robot.send("stop", now)
                    continue
                for line in self._repeat_lines:
                    self.robot.send(line, now)
            self.robot.tick(now)

    def handle(self, text):
        utt = parser.parse(text)
        utt = dialogue.resolve(utt, self.ctx)

        # Face first: it must change while the robot is still deciding.
        face = expression.face_for_utterance(utt, self.ctx)
        self.robot.send("setface %s" % face)

        p = plan.plan(utt, self.env)
        reply = dialogue.reply_for(utt)
        self.ctx = dialogue.advance(utt, self.ctx)

        with self._lock:
            # Any new utterance supersedes an in-flight drive.
            self._repeat_lines = []
            self._repeat_until = None
            now = time.monotonic()
            for line in p.lines:
                self.robot.send(line, now)
            if p.repeat and p.lines:
                self._repeat_lines = list(p.lines)
                if p.duration_s is not None:
                    self._repeat_until = now + p.duration_s

        return utt, p, reply

    def close(self):
        self._stop.set()


BANNER = """\
Sesame companion -- simulated robot, no hardware required.

Type what you would say, e.g.:
    walk forward            turn left            spin around
    faster                  again                the other way
    stop                    sit down             stand up
    look happy              go to sleep          help

Ctrl-C or 'quit' to exit.
"""


def main():
    robot = MockRobot()
    session = Session(robot)
    sys.stdout.write(BANNER)
    robot.render()
    try:
        while True:
            try:
                text = input("you > ").strip()
            except EOFError:
                break
            if text.lower() in ("quit", "exit"):
                break
            if not text:
                continue

            utt, p, reply = session.handle(text)
            sys.stdout.write("sesame > %s\n" % reply)
            detail = "    [%s" % utt.intent.value
            if utt.slots.direction:
                detail += " %s" % utt.slots.direction.value
            if utt.slots.speed_scale != 1.0:
                detail += " x%.2f" % utt.slots.speed_scale
            if p.repeat:
                detail += " | streaming @%.0fHz" % KEEPALIVE_HZ
                detail += (" for %.1fs" % p.duration_s) if p.duration_s \
                    else " until stop"
            detail += "]"
            sys.stdout.write("%s\n" % detail)
            for line in p.lines:
                sys.stdout.write("    -> %s\n" % line)
            robot.render()
    except KeyboardInterrupt:
        sys.stdout.write("\n")
    finally:
        session.close()
        robot.send("stop")
        sys.stdout.write("stopped.\n")


if __name__ == "__main__":
    main()
