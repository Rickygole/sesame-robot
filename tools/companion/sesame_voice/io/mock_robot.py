"""A simulated Sesame, so the whole pipeline is demonstrable with no robot.

It speaks the same one-line CLI grammar as the firmware and enforces the
same envelope, including the 500 ms drive watchdog. That matters: the
watchdog is what turns "walk until I say stop" into a command STREAM, and
if the mock did not enforce it, the companion could be built against a
robot that does not exist and fall over on real hardware.

It renders the 128x64 OLED face as ASCII in the terminal, so expression
changes are visible now rather than after the display driver exists.
"""

import sys
import time
from typing import Dict, List, Optional

DRIVE_WATCHDOG_S = 0.5  # must match kDriveWatchdogMs in robot_config.h

_FACES = {
    "neutral":   [" ●    ● ", "        ", "  ────  "],
    "happy":     [" ^    ^ ", "        ", "  \\__/  "],
    "sad":       [" ●    ● ", "        ", "  /‾‾\\  "],
    "angry":     [" \\    / ", "  ●  ●  ", "  /‾‾\\  "],
    "sleep":     [" ─    ─ ", "        ", "   zzz  "],
    "listening": [" ●    ● ", "   ((   ", "   ──   "],
    "thinking":  [" ●    ◦ ", "        ", "   ...  "],
    "confused":  [" ●    ○ ", "        ", "   ~~   "],
    "talking":   [" ●    ● ", "        ", "  ◡◡◡◡  "],
}


class MockRobot:
    """In-process mock. No sockets, so unit tests can drive it directly."""

    def __init__(self, echo: bool = True) -> None:
        self.echo = echo
        self.face = "neutral"
        self.mode = "stand"
        self.vx = 0.0
        self.omega = 0.0
        self.attached = True
        self.last_drive_at: Optional[float] = None
        self.log: List[str] = []
        self.rejected: List[str] = []

    # --- the wire ----------------------------------------------------
    def send(self, line: str, now: Optional[float] = None) -> Dict[str, object]:
        """Accept one CLI line. Returns what was ACTUALLY accepted.

        Returning achieved-vs-requested rather than a bare ok is the
        structural reason a voice command cannot become a lying knob --
        the same principle as GaitReport in the firmware.
        """
        now = time.monotonic() if now is None else now
        self.log.append(line)
        parts = line.split()
        if not parts:
            return {"ok": False, "reason": "empty"}
        cmd = parts[0]

        if cmd == "drive":
            try:
                vx, omega = float(parts[1]), float(parts[2])
            except (IndexError, ValueError):
                self.rejected.append(line)
                return {"ok": False, "reason": "malformed drive"}
            self.vx, self.omega = vx, omega
            self.mode = "drive"
            self.last_drive_at = now
            self.attached = True
            return {"ok": True, "achieved": {"vx": vx, "omega": omega}}

        if cmd == "stop":
            self.vx = self.omega = 0.0
            self.mode = "hold"
            self.last_drive_at = None
            return {"ok": True}

        if cmd in ("stand", "rest"):
            self.vx = self.omega = 0.0
            self.mode = cmd
            self.attached = True
            self.last_drive_at = None
            return {"ok": True}

        if cmd == "detach":
            self.attached = False
            self.vx = self.omega = 0.0
            self.mode = "detached"
            return {"ok": True}

        if cmd == "setface":
            if len(parts) < 2 or parts[1] not in _FACES:
                self.rejected.append(line)
                return {"ok": False, "reason": "unknown face"}
            self.face = parts[1]
            return {"ok": True}

        self.rejected.append(line)
        return {"ok": False, "reason": "unknown command"}

    def tick(self, now: Optional[float] = None) -> bool:
        """Advance the watchdog. True if it just fired.

        Same rule as the firmware: no drive command within the window and
        the robot stops itself. A crashed companion stops the robot for
        free.
        """
        now = time.monotonic() if now is None else now
        if self.mode == "drive" and self.last_drive_at is not None:
            if now - self.last_drive_at > DRIVE_WATCHDOG_S:
                self.vx = self.omega = 0.0
                self.mode = "stand"
                self.last_drive_at = None
                return True
        return False

    # --- display -----------------------------------------------------
    def render(self, out=sys.stdout) -> None:
        art = _FACES.get(self.face, _FACES["neutral"])
        motion = "still"
        if abs(self.vx) > 0.01 or abs(self.omega) > 0.01:
            motion = "vx=%.0fmm/s omega=%.0f deg/s" % (self.vx, self.omega)
        out.write("\n    ┌──────────┐\n")
        for row in art:
            out.write("    │ %-8s │\n" % row)
        out.write("    └──────────┘\n")
        out.write("     %s | %s | %s\n\n"
                  % (self.mode, self.face, motion))
        out.flush()
