"""Intent -> robot command lines.

Pure and deterministic, and identical no matter which brain produced the
intent. This is the only place motion is generated, which is what makes
swapping in a language model later a safe change.

Commands are emitted in the robot's EXISTING CLI grammar (see
firmware/sesame/src/core/command.h) -- the same strings you can type into
the serial console, and the same parser that is already unit-tested in
firmware/host/test_command.cpp. No second grammar, no JSON parser on the
ESP32, no new bug surface.

VELOCITY CAPS ARE DERIVED, NOT GUESSED. See Envelope below.

PURE: stdlib only.
"""

import math
from typing import List, NamedTuple, Optional

from .intent import Direction, Intent, Slots, Utterance


class Envelope(NamedTuple):
    """Advisory limits, derived from the robot's geometry.

    ADVISORY ONLY. The firmware is authoritative and clamps everything
    again; this exists so the companion can give immediate feedback and
    avoid asking for things it knows are impossible.

    The caps are COMPUTED from link lengths and the gait, never typed in
    as constants -- a plausible-sounding hardcoded number would be
    silently unreachable, which is exactly the class of bug that made the
    original gait's knobs meaningless.

    On the placeholder geometry this yields ~59 mm/s. Note that is NOT
    stride x frequency (~44 mm/s), which drops the duty-factor term and
    is only correct at duty == 1. The formula below is the exact inverse
    of the gait's own A = |v|*duty / (2*f*r), cross-checked against the
    simulator: at vx=40 the sim reports strideMm 42.857, and
    2*A*r with A = 40*0.75/(2*0.7*72.63) gives 42.86.
    """
    coxa_mm: float = 25.0
    leg_mm: float = 55.0
    body_height_frac: float = 0.50
    step_height_frac: float = 0.15
    max_half_sweep_deg: float = 25.0   # mechanical, from gait.h
    gait_freq_hz: float = 0.7          # crawl, from gait.h
    duty: float = 0.75

    @property
    def body_height_mm(self) -> float:
        return self.body_height_frac * self.leg_mm

    @property
    def step_height_mm(self) -> float:
        return self.step_height_frac * self.leg_mm

    @property
    def stance_radius_mm(self) -> float:
        phi = math.asin(self.body_height_mm / self.leg_mm)
        return self.coxa_mm + self.leg_mm * math.cos(phi)

    @property
    def max_vx_mm_s(self) -> float:
        """Fastest forward speed the mechanism can actually deliver.

        A half-sweep A over stance time gives stride 2*A*r; at freq f the
        body advances that far each cycle. Inverting the gait's own
        A = |v|*duty / (2*f*r) at A = A_max gives v_max.
        """
        a_max = math.radians(self.max_half_sweep_deg)
        return a_max * 2.0 * self.gait_freq_hz * self.stance_radius_mm / self.duty

    @property
    def max_omega_deg_s(self) -> float:
        """Turn rate whose tangential foot speed equals the linear cap."""
        return math.degrees(self.max_vx_mm_s / self.stance_radius_mm)


class Plan(NamedTuple):
    """What to send, and whether it needs to keep being sent.

    `repeat` is the load-bearing field. The robot's drive watchdog stops
    it if no command arrives within 500 ms, so "walk until I say stop"
    is a COMMAND STREAM, not one packet. That is a safety property, not
    an inconvenience: if the Mac crashes, the WiFi drops, or this process
    is killed, the robot stops on its own within half a second.
    """
    lines: List[str]
    repeat: bool = False           # keep resending `lines` until superseded
    duration_s: Optional[float] = None  # None + repeat => until STOP
    say: Optional[str] = None


def _trunc1(v: float) -> float:
    """Truncate toward zero to 1 decimal.

    Not rounding: "%.1f" would round 59.157 up to 59.2, so the emitted
    command would exceed the cap we just computed. Tiny, but it is the
    same "the number does not mean what we said" pattern that made the
    original gait's knobs untrustworthy. Truncation cannot overshoot.
    """
    return math.floor(abs(v) * 10.0) / 10.0 * (1.0 if v >= 0 else -1.0)


def _fmt(vx: float, omega: float, env: Envelope) -> str:
    return "drive %.1f %.1f %.1f %.1f" % (
        _trunc1(vx), _trunc1(omega),
        _trunc1(env.body_height_mm), _trunc1(env.step_height_mm))


def plan(utt: Utterance, env: Envelope = Envelope()) -> Plan:
    """Translate one intent into robot commands. Never raises."""
    s: Slots = utt.slots
    scale = max(0.1, min(s.speed_scale, 1.0))

    if utt.intent is Intent.DRIVE:
        vx = env.max_vx_mm_s * scale
        if s.direction is Direction.BACKWARD:
            vx = -vx
        return Plan([_fmt(vx, 0.0, env)], repeat=True,
                    duration_s=s.duration_s)

    if utt.intent is Intent.TURN:
        omega = env.max_omega_deg_s * scale
        if s.direction is Direction.RIGHT:
            omega = -omega
        duration = s.duration_s
        if s.direction is Direction.AROUND and duration is None:
            # A 180 at this rate, so "spin around" terminates by itself
            # rather than spinning forever.
            duration = 180.0 / max(abs(omega), 1e-3)
        return Plan([_fmt(0.0, omega, env)], repeat=True, duration_s=duration)

    if utt.intent is Intent.STOP:
        return Plan(["stop"], repeat=False)

    if utt.intent is Intent.STAND:
        return Plan(["stand"], repeat=False)

    if utt.intent is Intent.SIT or utt.intent is Intent.REST:
        return Plan(["rest"], repeat=False)

    if utt.intent is Intent.SLEEP:
        # Face first, then release torque: the expression should land
        # before the body goes limp, or it reads as a crash.
        return Plan(["setface sleep", "detach"], repeat=False)

    if utt.intent is Intent.WAKE:
        return Plan(["setface neutral", "stand"], repeat=False)

    if utt.intent is Intent.SET_FACE and s.face:
        return Plan(["setface %s" % s.face], repeat=False)

    # GREET / HELP / STATUS / UNKNOWN move nothing.
    return Plan([], repeat=False)
