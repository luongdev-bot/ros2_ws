"""Domain errors for the line-following pipeline."""


class LineFollowError(Exception):
    """Base class for every error this package raises deliberately."""


class InvalidLineConfigError(LineFollowError, ValueError):
    """A line colour, ROI band, or configuration value is malformed."""
