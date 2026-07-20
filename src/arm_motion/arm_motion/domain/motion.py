"""Motion = an ordered list of timed waypoints ("action group")."""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .errors import InvalidMotionError
from .pose import Pose

# Guard rails so a typo in the editor cannot command a 0 ms or multi-hour move.
MIN_STEP_DURATION_MS = 20
MAX_STEP_DURATION_MS = 600_000


@dataclass(frozen=True)
class MotionStep:
    """One waypoint.

    ``duration_ms`` is the time allotted to travel *into* this pose from the
    previous step (or, for the first step, from wherever the arm currently is).
    This matches the "Time" column of the action-group table.
    """

    pose: Pose
    duration_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.duration_ms, int):
            object.__setattr__(self, "duration_ms", int(self.duration_ms))
        if self.duration_ms < MIN_STEP_DURATION_MS:
            raise InvalidMotionError(
                f"step duration {self.duration_ms} ms is below the minimum "
                f"{MIN_STEP_DURATION_MS} ms"
            )
        if self.duration_ms > MAX_STEP_DURATION_MS:
            raise InvalidMotionError(
                f"step duration {self.duration_ms} ms exceeds the maximum "
                f"{MAX_STEP_DURATION_MS} ms"
            )

    def with_duration(self, duration_ms: int) -> "MotionStep":
        return MotionStep(self.pose, duration_ms)

    def with_pose(self, pose: Pose) -> "MotionStep":
        return MotionStep(pose, self.duration_ms)


@dataclass(frozen=True)
class Motion:
    """A named, ordered sequence of steps — one ``.d6a`` file.

    Attributes:
        name: Unique action-group name (no spaces — it becomes a filename).
        steps: Ordered waypoints.
        description: Free-form note shown in the editor.
        updated_at: ISO-8601 timestamp, set by the repository on save.
    """

    name: str
    steps: Sequence[MotionStep] = field(default_factory=tuple)
    description: str = ""
    updated_at: Optional[str] = None

    def __post_init__(self) -> None:
        validate_motion_name(self.name)
        object.__setattr__(self, "steps", tuple(self.steps))

    def __len__(self) -> int:
        return len(self.steps)

    @property
    def total_duration_ms(self) -> int:
        return sum(step.duration_ms for step in self.steps)

    def require_steps(self) -> None:
        if not self.steps:
            raise InvalidMotionError(f"motion '{self.name}' has no steps")

    def cumulative_times_ms(self) -> List[int]:
        """Time-from-start of each step, for building a joint trajectory."""
        times: List[int] = []
        running = 0
        for step in self.steps:
            running += step.duration_ms
            times.append(running)
        return times

    def rescaled(self, total_duration_ms: int) -> "Motion":
        """Return a copy stretched/squeezed to finish in ``total_duration_ms``.

        Step durations keep their relative proportions. This is what lets a
        caller say "run 'home', and be done within 1000 ms" regardless of the
        timings that were recorded in the editor.
        """
        self.require_steps()
        if total_duration_ms <= 0:
            raise InvalidMotionError("total_duration_ms must be > 0")

        floor_total = MIN_STEP_DURATION_MS * len(self.steps)
        if total_duration_ms < floor_total:
            raise InvalidMotionError(
                f"motion '{self.name}' has {len(self.steps)} steps and so "
                f"needs at least {floor_total} ms, not {total_duration_ms} ms"
            )
        ceiling_total = MAX_STEP_DURATION_MS * len(self.steps)
        if total_duration_ms > ceiling_total:
            raise InvalidMotionError(
                f"motion '{self.name}' has {len(self.steps)} steps and so "
                f"cannot exceed {ceiling_total} ms, not {total_duration_ms} ms"
            )

        original_total = self.total_duration_ms
        if original_total <= 0:
            # Degenerate but recoverable: spread the budget evenly.
            share = total_duration_ms // len(self.steps)
            raw = [_clamp_step(share)] * len(self.steps)
        else:
            factor = total_duration_ms / original_total
            raw = [_clamp_step(round(s.duration_ms * factor)) for s in self.steps]

        raw = _absorb_rounding_error(raw, total_duration_ms)
        rescaled_steps = tuple(
            step.with_duration(duration)
            for step, duration in zip(self.steps, raw)
        )
        return Motion(
            name=self.name,
            steps=rescaled_steps,
            description=self.description,
            updated_at=self.updated_at,
        )

    # -- editing helpers (the editor's Add / Insert / Delete / Up / Down) --

    def with_step_appended(self, step: MotionStep) -> "Motion":
        return self._replacing(list(self.steps) + [step])

    def with_step_inserted(self, index: int, step: MotionStep) -> "Motion":
        steps = list(self.steps)
        steps.insert(_bounded_insert_index(index, len(steps)), step)
        return self._replacing(steps)

    def with_step_replaced(self, index: int, step: MotionStep) -> "Motion":
        steps = list(self.steps)
        steps[_checked_index(index, len(steps))] = step
        return self._replacing(steps)

    def with_step_removed(self, index: int) -> "Motion":
        steps = list(self.steps)
        del steps[_checked_index(index, len(steps))]
        return self._replacing(steps)

    def with_step_moved(self, index: int, offset: int) -> Tuple["Motion", int]:
        """Move a step up/down. Returns the new motion and the new index."""
        steps = list(self.steps)
        current = _checked_index(index, len(steps))
        target = current + offset
        if not 0 <= target < len(steps):
            # Already at an end — a no-op is friendlier than an error here.
            return self, current
        steps[current], steps[target] = steps[target], steps[current]
        return self._replacing(steps), target

    def prepended_with(self, other: "Motion") -> "Motion":
        """Return this motion with ``other``'s steps run first.

        Used to make every motion pass through a safety pose. The result
        keeps this motion's name and metadata - only the step list grows.
        """
        return self._replacing(list(other.steps) + list(self.steps))

    def renamed(self, name: str) -> "Motion":
        return Motion(name, self.steps, self.description, self.updated_at)

    def _replacing(self, steps: Sequence[MotionStep]) -> "Motion":
        return Motion(self.name, tuple(steps), self.description, self.updated_at)


def _checked_index(index: int, length: int) -> int:
    if not 0 <= index < length:
        raise IndexError(f"step index {index} out of range (0..{length - 1})")
    return index


def _bounded_insert_index(index: int, length: int) -> int:
    return min(max(index, 0), length)


def _clamp_step(duration: int) -> int:
    """Keep a single step within the per-step [min, max] bounds."""
    return min(max(int(duration), MIN_STEP_DURATION_MS), MAX_STEP_DURATION_MS)


def _absorb_rounding_error(durations: List[int], target_total: int) -> List[int]:
    """Nudge durations so they sum to exactly ``target_total``.

    Rounding each step independently leaves a few ms of drift; push the
    remainder onto the steps that have room, respecting both the per-step
    minimum and maximum. The caller guarantees the target fits within
    ``[min*n, max*n]``, so a feasible distribution always exists.
    """
    drift = target_total - sum(durations)
    if drift == 0:
        return durations

    positive = drift > 0
    # For extra time, fill the smallest steps first; for less, drain the
    # largest — this keeps proportions as close as possible.
    order = sorted(
        range(len(durations)), key=lambda i: durations[i], reverse=not positive
    )
    step = 1 if positive else -1
    cursor = 0
    guard = abs(drift) * len(durations) + len(durations)
    while drift != 0 and guard > 0:
        guard -= 1
        i = order[cursor % len(order)]
        cursor += 1
        if positive and durations[i] >= MAX_STEP_DURATION_MS:
            continue
        if not positive and durations[i] <= MIN_STEP_DURATION_MS:
            continue
        durations[i] += step
        drift -= step
    return durations


def validate_motion_name(name: str) -> None:
    """Action-group names become filenames, so they must be tame.

    Mirrors the constraint the Hiwonder editor states explicitly: no spaces.
    """
    if not name:
        raise InvalidMotionError("motion name must not be empty")
    if any(ch.isspace() for ch in name):
        raise InvalidMotionError(f"motion name '{name}' must not contain spaces")
    forbidden = set('/\\:*?"<>|')
    bad = forbidden.intersection(name)
    if bad:
        raise InvalidMotionError(
            f"motion name '{name}' contains forbidden characters: "
            + " ".join(sorted(bad))
        )
    if name in (".", ".."):
        raise InvalidMotionError(f"motion name '{name}' is reserved")
