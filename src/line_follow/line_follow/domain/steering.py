"""Steering output produced by the line-following use case."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SteeringCommand:
    """Planar velocity command and whether its source line was visible."""

    linear_x: float
    angular_z: float
    line_found: bool
