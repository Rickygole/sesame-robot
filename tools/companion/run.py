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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sesame_voice.io.mock_robot import MockRobot  # noqa: E402
from sesame_voice.session import KEEPALIVE_HZ, Session  # noqa: E402


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
