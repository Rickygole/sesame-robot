"""The companion's session: text in, robot commands + spoken reply out.

Shared by both front ends -- the typed console (run.py) and the voice
entrypoint (run_voice.py). It only knows about core/ (resolve, plan,
expression), a `Brain` (text -> Utterance; see brain/), and a robot
object with a mock_robot.MockRobot-shaped interface (send/tick). It has
no idea whether the text it's fed came from a keyboard or a microphone,
which is the whole point of the split: voice is a front end that only
produces text, not a second implementation of what the text means.
"""

import threading
import time

from .brain.rules import RulesBrain
from .core import dialogue, expression, plan
from .core.dialogue import Context

KEEPALIVE_HZ = 5.0  # comfortably inside the robot's 500ms watchdog


class Session(object):
    def __init__(self, robot, brain=None):
        self.robot = robot
        # Default brain is the rule parser alone -- identical to every
        # Session before brains existed. Passing a brain (e.g. a
        # CascadeBrain) is the only way behavior changes at all.
        self.brain = brain if brain is not None else RulesBrain()
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
        utt = self.brain.interpret(text, self.ctx)
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
