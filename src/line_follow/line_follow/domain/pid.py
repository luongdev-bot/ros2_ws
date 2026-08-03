"""Pure PID control with no ROS or clock dependency."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PidGains:
    """PID gains loaded from line-following configuration."""

    kp: float
    ki: float
    kd: float


class PID:
    """PID controller whose caller supplies both error and elapsed time."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        *,
        output_min: Optional[float] = None,
        output_max: Optional[float] = None,
    ) -> None:
        if (
            output_min is not None
            and output_max is not None
            and output_min > output_max
        ):
            raise ValueError("output_min must not exceed output_max")

        self._kp = float(kp)
        self._ki = float(ki)
        self._kd = float(kd)
        self._output_min = output_min
        self._output_max = output_max
        self.reset()

    def update(self, error: float, dt: float) -> float:
        """Return the control output for ``error`` over elapsed ``dt``."""
        derivative = 0.0
        if dt > 0.0:
            if self._previous_error is not None:
                derivative = (error - self._previous_error) / dt

            integral_delta = error * dt
            candidate_integral = self._integral + integral_delta
            candidate_output = (
                self._kp * error
                + self._ki * candidate_integral
                + self._kd * derivative
            )
            integral_effect = self._ki * integral_delta
            pushes_past_upper = (
                self._output_max is not None
                and candidate_output > self._output_max
                and integral_effect > 0.0
            )
            pushes_past_lower = (
                self._output_min is not None
                and candidate_output < self._output_min
                and integral_effect < 0.0
            )
            if self._ki == 0.0 or not (
                pushes_past_upper or pushes_past_lower
            ):
                self._integral = candidate_integral

        self._previous_error = error
        output = (
            self._kp * error
            + self._ki * self._integral
            + self._kd * derivative
        )

        if self._output_min is not None:
            output = max(self._output_min, output)
        if self._output_max is not None:
            output = min(self._output_max, output)
        return output

    def reset(self) -> None:
        """Clear accumulated integral and derivative history."""
        self._integral = 0.0
        self._previous_error = None
