"""One action group in a colour's motion sequence."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionStep:
    """A single action group to play, with its time budget.

    A colour's response is an ordered sequence of these - typically a
    shared ``pick`` followed by a colour-specific ``place`` - played one
    after another, each starting only once the previous one succeeds.
    """

    name: str
    #: Playback budget in ms; 0 keeps the timings stored in the .d6a file.
    duration_ms: int = 0

    def __post_init__(self) -> None:
        if not self.name or not self.name.strip():
            raise ValueError("motion step name must not be blank")
        if self.duration_ms < 0:
            raise ValueError(
                f"duration_ms must not be negative, got {self.duration_ms}"
            )
