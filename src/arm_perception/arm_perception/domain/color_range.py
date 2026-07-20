"""Colour definitions expressed as a box in OpenCV 8-bit LAB space.

LAB is used rather than HSV because the L (lightness) axis is separable
from the a/b (chromaticity) axes: a block keeps roughly the same a/b as
the lighting changes, so a wide L range plus tight a/b survives shadow
far better than an HSV hue window does.

OpenCV packs LAB into three uint8 channels:
  L: 0..255 (0 = black, 255 = white)
  a: 0..255 (128 = neutral, <128 green, >128 magenta)
  b: 0..255 (128 = neutral, <128 blue,  >128 yellow)
"""

from dataclasses import dataclass
from typing import Dict, Iterator, Mapping, Tuple

from .errors import InvalidColorRangeError

#: A LAB triple as OpenCV stores it.
LabTriple = Tuple[int, int, int]

_CHANNEL_NAMES = ("L", "a", "b")


@dataclass(frozen=True)
class ColorRange:
    """One named colour, as inclusive lower/upper LAB bounds.

    ``min_area_px`` rejects specks: a contour smaller than this is noise,
    not a block. It is compared against the area measured on the
    *downscaled* processing frame, not the full-resolution image.
    """

    name: str
    lab_min: LabTriple
    lab_max: LabTriple
    min_area_px: int = 200

    def __post_init__(self) -> None:
        if not self.name:
            raise InvalidColorRangeError("colour name must not be empty")

        for bound, label in ((self.lab_min, "lab_min"), (self.lab_max, "lab_max")):
            if len(bound) != 3:
                raise InvalidColorRangeError(
                    f"{self.name}.{label} must have 3 channels, got {len(bound)}"
                )
            for channel, value in zip(_CHANNEL_NAMES, bound):
                if not 0 <= value <= 255:
                    raise InvalidColorRangeError(
                        f"{self.name}.{label}.{channel} = {value} is outside 0..255"
                    )

        for channel, low, high in zip(_CHANNEL_NAMES, self.lab_min, self.lab_max):
            if low > high:
                raise InvalidColorRangeError(
                    f"{self.name}: {channel} lower bound {low} exceeds upper bound {high}"
                )

        if self.min_area_px < 0:
            raise InvalidColorRangeError(
                f"{self.name}.min_area_px must not be negative, got {self.min_area_px}"
            )

    def contains(self, lab: LabTriple) -> bool:
        """Whether a single LAB sample falls inside this box.

        Used by the colour-picker helper to report which configured
        colour a clicked pixel matches; the per-frame masking itself is
        done in the adapter by OpenCV.
        """
        return all(
            low <= value <= high
            for low, value, high in zip(self.lab_min, lab, self.lab_max)
        )


@dataclass(frozen=True)
class ColorPalette:
    """The set of colours the robot is configured to recognise."""

    colors: Mapping[str, ColorRange]

    def __post_init__(self) -> None:
        if not self.colors:
            raise InvalidColorRangeError("palette must define at least one colour")
        for key, color_range in self.colors.items():
            if key != color_range.name:
                raise InvalidColorRangeError(
                    f"palette key {key!r} does not match colour name {color_range.name!r}"
                )

    @classmethod
    def of(cls, *ranges: ColorRange) -> "ColorPalette":
        return cls({r.name: r for r in ranges})

    def __getitem__(self, name: str) -> ColorRange:
        return self.colors[name]

    def __contains__(self, name: str) -> bool:
        return name in self.colors

    def __iter__(self) -> Iterator[ColorRange]:
        return iter(self.colors.values())

    def __len__(self) -> int:
        return len(self.colors)

    def names(self) -> Tuple[str, ...]:
        return tuple(self.colors)

    def as_dict(self) -> Dict[str, ColorRange]:
        return dict(self.colors)
