"""Talk to a real Sesame over WiFi.

Deliberately exposes the SAME interface as MockRobot -- send(line, now),
tick(now), render() -- so Session works with either and the whole voice
pipeline can be developed against the mock and then pointed at hardware
by changing one flag. Nothing upstream of this file knows the difference.

The wire format is one line of the robot's CLI grammar posted as
text/plain to /cmd, which is exactly what the firmware's
control_server.cpp accepts and what you would type into the serial
console. Same grammar, same parser, same tests.

stdlib only (urllib) -- no requests dependency.
"""

import json
import sys
import time
import urllib.error
import urllib.request
from typing import Dict, Optional

DEFAULT_TIMEOUT_S = 1.0


class RobotHttp:
    """HTTP client for a real robot.

    Failures are reported, never raised into the caller's control loop:
    a dropped packet while walking must not crash the companion, because
    the companion crashing is what the robot's 500ms watchdog is there to
    survive. Every failure returns the same {"ok": False, "reason": ...}
    shape MockRobot uses.
    """

    def __init__(self, host: str = "sesame.local", port: int = 80,
                 timeout: float = DEFAULT_TIMEOUT_S) -> None:
        self.base = "http://%s:%d" % (host, port)
        self.timeout = timeout
        self.face = "neutral"
        self.mode = "?"
        self.vx = 0.0
        self.omega = 0.0
        self.attached = False
        self.online = False
        self.last_error: Optional[str] = None
        self.consecutive_failures = 0

    # --- the wire ----------------------------------------------------
    def send(self, line: str, now: Optional[float] = None) -> Dict[str, object]:
        del now  # the robot timestamps its own watchdog
        try:
            req = urllib.request.Request(
                self.base + "/cmd", data=line.encode("utf-8"),
                headers={"Content-Type": "text/plain"}, method="POST")
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8", "replace")
            self.online = True
            self.consecutive_failures = 0
            self.last_error = None
            try:
                parsed = json.loads(body)
            except ValueError:
                return {"ok": False, "reason": "bad json: %s" % body[:80]}
            # Mirror locally so render() has something to show without an
            # extra round trip.
            if line.startswith("drive") and parsed.get("ok"):
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        self.vx = float(parts[1])
                        self.omega = float(parts[2])
                    except ValueError:
                        pass
            elif line.startswith("setface") and parsed.get("ok"):
                self.face = line.split()[-1]
            elif line in ("stop", "stand", "rest") and parsed.get("ok"):
                self.vx = self.omega = 0.0
            return parsed
        except (urllib.error.URLError, OSError) as exc:
            self.online = False
            self.consecutive_failures += 1
            self.last_error = str(exc)
            return {"ok": False, "reason": "unreachable: %s" % exc}

    def status(self) -> Dict[str, object]:
        try:
            with urllib.request.urlopen(self.base + "/status",
                                        timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode("utf-8", "replace"))
            self.online = True
            self.mode = str(data.get("mode", "?"))
            self.attached = bool(data.get("attached", False))
            self.vx = float(data.get("vx", 0.0))
            self.omega = float(data.get("omega", 0.0))
            return data
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.online = False
            self.last_error = str(exc)
            return {}

    def estop(self) -> bool:
        try:
            req = urllib.request.Request(self.base + "/estop", data=b"",
                                         method="POST")
            with urllib.request.urlopen(req, timeout=self.timeout):
                return True
        except (urllib.error.URLError, OSError):
            return False

    def tick(self, now: Optional[float] = None) -> bool:
        """No-op: the REAL watchdog lives on the robot.

        Present only so RobotHttp and MockRobot are interchangeable.
        Returns False always -- the robot stops itself if this process
        stops sending, which is the entire point of the watchdog.
        """
        del now
        return False

    # --- display -----------------------------------------------------
    def render(self, out=sys.stdout) -> None:
        state = "online" if self.online else "OFFLINE"
        motion = "still"
        if abs(self.vx) > 0.01 or abs(self.omega) > 0.01:
            motion = "vx=%.0fmm/s omega=%.0f deg/s" % (self.vx, self.omega)
        out.write("\n    [robot @ %s] %s | %s | %s | %s\n"
                  % (self.base, state, self.mode, self.face, motion))
        if not self.online and self.last_error:
            out.write("    last error: %s\n" % self.last_error[:70])
        out.write("\n")
        out.flush()

    def wait_online(self, attempts: int = 5) -> bool:
        for i in range(attempts):
            if self.status():
                return True
            time.sleep(0.4 * (i + 1))
        return False
