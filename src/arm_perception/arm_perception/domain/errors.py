"""Domain errors for the colour-block perception pipeline."""


class PerceptionError(Exception):
    """Base class for every error this package raises deliberately."""


class InvalidColorRangeError(PerceptionError):
    """A colour range or palette is malformed."""


class UnknownColorError(PerceptionError):
    """A colour was referenced that the palette does not define."""


class MotionPlaybackError(PerceptionError):
    """The arm refused, aborted, or never accepted a motion request."""
