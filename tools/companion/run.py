#!/usr/bin/env python3
"""Sesame companion -- console driver.

Type what you would say. The full pipeline runs: parse -> resolve against
conversation context -> plan robot commands -> send to a simulated robot
-> render its face. No microphone, no permissions, no network, no robot.

Voice input is a separate front end (run_voice.py) and is deliberately
swappable: it only produces text, and everything downstream of that is
already here and unit-tested.

    python3 tools/companion/run.py                    # simulated robot
    python3 tools/companion/run.py --robot            # real, sesame.local
    python3 tools/companion/run.py --robot 192.168.4.1
    python3 tools/companion/run.py --brain cascade     # + optional local LLM

The real and simulated robots expose the SAME interface, so nothing
above this file knows which one it is driving. Likewise --brain: the
default ('rules') is the rule parser alone, unchanged from before that
flag existed. See sesame_voice/brain/ and this directory's README.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sesame_voice.brain import BRAIN_NAMES, build_brain  # noqa: E402
from sesame_voice.io.mock_robot import MockRobot  # noqa: E402
from sesame_voice.io.robot_http import RobotHttp  # noqa: E402
from sesame_voice.session import KEEPALIVE_HZ, Session  # noqa: E402


def select_brain(argv):
    """--brain cascade also asks a local Ollama model when the rule
    parser cannot place an utterance at all; --brain rules (or no flag)
    is the rule parser alone, exactly as before this flag existed."""
    name = "rules"
    if "--brain" in argv:
        i = argv.index("--brain")
        if i + 1 < len(argv) and not argv[i + 1].startswith("-"):
            name = argv[i + 1]
    if name not in BRAIN_NAMES:
        sys.stdout.write(
            "unknown --brain '%s'; using 'rules' (choices: %s)\n"
            % (name, ", ".join(BRAIN_NAMES)))
        name = "rules"
    brain = build_brain(name)
    if name == "cascade" and not brain.llm.available():
        sys.stdout.write(
            "note: --brain cascade is on, but no Ollama model is reachable "
            "at %s -- behaving exactly like --brain rules until one is. "
            "See tools/companion/README.md.\n" % brain.llm.host)
    return brain


def build_robot(argv):
    """--robot [host] selects real hardware; default is the simulator."""
    if "--robot" not in argv:
        return MockRobot(), False
    i = argv.index("--robot")
    host = "sesame.local"
    if i + 1 < len(argv) and not argv[i + 1].startswith("-"):
        host = argv[i + 1]
    robot = RobotHttp(host)
    sys.stdout.write("connecting to %s ...\n" % robot.base)
    if not robot.wait_online():
        sys.stdout.write(
            "could not reach the robot at %s\n"
            "  - is it powered on?\n"
            "  - are you on its WiFi (the 'Sesame' network), or is it on yours?\n"
            "  - try the IP printed on its serial console instead of the name\n"
            "continuing anyway; commands will report 'unreachable'.\n\n"
            % robot.base)
    return robot, True


BANNER = """\
Sesame companion.

Type what you would say, e.g.:
    walk forward            turn left            spin around
    faster                  again                the other way
    stop                    sit down             stand up
    look happy              go to sleep          help

Ctrl-C or 'quit' to exit.
"""


def main():
    robot, is_real = build_robot(sys.argv[1:])
    brain = select_brain(sys.argv[1:])
    session = Session(robot, brain=brain)
    sys.stdout.write(BANNER)
    if is_real:
        sys.stdout.write(
            "*** driving a REAL robot -- keep the area clear ***\n")
    else:
        sys.stdout.write(
            "Simulated robot; no hardware required. Use --robot for the real one.\n")
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
