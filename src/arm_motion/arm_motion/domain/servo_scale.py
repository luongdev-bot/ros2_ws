"""Conversion between Hiwonder-style servo pulses and URDF radians.

The editor UI speaks *pulses* (0..1000, 500 = centre) because that is what the
action-group tables have always contained. Gazebo and the URDF speak
*radians*. This module is the only place the two meet.

Default scaling: a 0..1000 pulse span covers 240 deg, i.e. +/-2.0944 rad
around centre, which matches the +/-2.09 limits of joint1..joint5.
"""

from dataclasses import dataclass

DEFAULT_MIN_PULSE = 0
DEFAULT_MAX_PULSE = 1000
# +/-120 degrees.
DEFAULT_MIN_RAD = -2.0943951023931953
DEFAULT_MAX_RAD = 2.0943951023931953

CENTER_PULSE = 500


@dataclass(frozen=True)
class ServoScale:
    """Linear map between one servo's pulse range and its joint's radian range.

    Attributes:
        servo_id: Bus id shown in the editor (ID:1 ... ID:10).
        joint_name: URDF joint this servo drives.
        min_pulse / max_pulse: Pulse range the editor slider spans.
        min_rad / max_rad: Radian range those pulses map onto.
        invert: Swap the direction, for servos mounted mirrored.
    """

    servo_id: int
    joint_name: str
    min_pulse: int = DEFAULT_MIN_PULSE
    max_pulse: int = DEFAULT_MAX_PULSE
    min_rad: float = DEFAULT_MIN_RAD
    max_rad: float = DEFAULT_MAX_RAD
    invert: bool = False

    def __post_init__(self) -> None:
        if self.max_pulse == self.min_pulse:
            raise ValueError(
                f"servo {self.servo_id}: min_pulse and max_pulse must differ"
            )
        if self.max_rad == self.min_rad:
            raise ValueError(
                f"servo {self.servo_id}: min_rad and max_rad must differ"
            )

    @property
    def _pulse_span(self) -> float:
        return float(self.max_pulse - self.min_pulse)

    @property
    def _rad_span(self) -> float:
        return float(self.max_rad - self.min_rad)

    def to_radians(self, pulse: float) -> float:
        """Map a pulse value onto the joint's radian range."""
        ratio = (float(pulse) - self.min_pulse) / self._pulse_span
        if self.invert:
            ratio = 1.0 - ratio
        return self.min_rad + ratio * self._rad_span

    def to_pulse(self, radians: float) -> int:
        """Map a joint angle back onto a pulse value, rounded to an integer.

        The result is clamped to ``[min_pulse, max_pulse]`` so the editor
        never displays a slider position it cannot render.
        """
        ratio = (float(radians) - self.min_rad) / self._rad_span
        if self.invert:
            ratio = 1.0 - ratio
        pulse = self.min_pulse + ratio * self._pulse_span
        lo, hi = sorted((self.min_pulse, self.max_pulse))
        return int(round(min(max(pulse, lo), hi)))

    def clamp_pulse(self, pulse: int) -> int:
        lo, hi = sorted((self.min_pulse, self.max_pulse))
        return min(max(int(pulse), lo), hi)
