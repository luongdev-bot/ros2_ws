"""Domain errors. Pure Python — no ROS dependencies."""


class ArmMotionError(Exception):
    """Base class for every error raised by the arm_motion domain."""


class UnknownJointError(ArmMotionError):
    """A joint name was referenced that is not part of the robot profile."""


class JointLimitError(ArmMotionError):
    """A commanded position lies outside the joint's allowed range."""

    def __init__(self, joint_name: str, requested: float, lower: float, upper: float):
        super().__init__(
            f"joint '{joint_name}': {requested:.4f} is outside "
            f"[{lower:.4f}, {upper:.4f}]"
        )
        self.joint_name = joint_name
        self.requested = requested
        self.lower = lower
        self.upper = upper


class UnsupportedJointMotionError(ArmMotionError):
    """The requested motion is not something this joint can physically do.

    Raised e.g. when a continuous jog is requested on a gripper that only
    supports discrete open/close.
    """


class IncompletePoseError(ArmMotionError):
    """A pose does not cover every joint of the robot profile."""


class MotionNotFoundError(ArmMotionError):
    """No motion with the requested name exists in the library."""


class MotionAlreadyExistsError(ArmMotionError):
    """A motion with that name exists and overwrite was not requested."""


class InvalidMotionError(ArmMotionError):
    """The motion is structurally invalid (no steps, bad duration, ...)."""


class MotionExecutionError(ArmMotionError):
    """Execution on the hardware/simulator failed or was rejected."""


class MotionCancelledError(ArmMotionError):
    """Execution was cancelled before completing."""
