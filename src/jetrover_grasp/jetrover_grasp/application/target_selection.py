"""Pure target selection for detected graspable blocks."""

from dataclasses import dataclass


@dataclass(frozen=True)
class DetectedBlock:
    """One colour detection in image coordinates."""

    color: str
    u: float
    v: float
    area: float


def select_target(
    detections: list[DetectedBlock],
    allowed_colors: set[str],
) -> DetectedBlock | None:
    """Return the largest detection whose colour is allowed."""
    ranked = rank_targets(detections, allowed_colors)
    return ranked[0] if ranked else None


def rank_targets(
    detections: list[DetectedBlock],
    allowed_colors: set[str],
) -> list[DetectedBlock]:
    """Return all allowed detections in descending area order.

    Equal-area detections retain their input order as a deterministic
    tie-break, which also preserves ``select_target``'s historical behavior.
    """
    allowed = [
        (index, detection)
        for index, detection in enumerate(detections)
        if detection.color in allowed_colors
    ]
    allowed.sort(key=lambda item: (-item[1].area, item[0]))
    return [detection for _, detection in allowed]


__all__ = ["DetectedBlock", "rank_targets", "select_target"]
