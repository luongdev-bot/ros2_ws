"""The line-following use case wired to fakes, with no ROS imports."""

import pytest

from line_follow.application.line_follow import LineFollowUseCase
from line_follow.domain.detection import LineDetection
from line_follow.domain.pid import PID
from line_follow.domain.ports import LineDetector
from line_follow.domain.steering import SteeringCommand


class FakeDetector(LineDetector):
    def __init__(self, detections=()):
        self.detections = list(detections)
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        if not self.detections:
            return None
        return self.detections.pop(0)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def build(detections, **overrides):
    clock = FakeClock()
    max_angular_speed = overrides.pop("max_angular_speed", 0.8)
    use_case = LineFollowUseCase(
        FakeDetector(detections),
        PID(
            kp=overrides.pop("kp", 0.01),
            ki=0.0,
            kd=0.0,
            output_min=-max_angular_speed,
            output_max=max_angular_speed,
        ),
        cruise_speed=overrides.pop("cruise_speed", 0.2),
        max_angular_speed=max_angular_speed,
        min_speed_scale=overrides.pop("min_speed_scale", 0.4),
        lost_line_timeout_s=overrides.pop("lost_line_timeout_s", 0.4),
        clock=clock,
        **overrides,
    )
    return use_case, clock


def test_line_left_of_center_turns_left_and_right_turns_right():
    use_case, clock = build([
        LineDetection(center_x=200.0, confidence=1.0),
        LineDetection(center_x=440.0, confidence=1.0),
    ])

    left = use_case.process_frame(object(), frame_width=640.0)
    clock.advance(0.1)
    right = use_case.process_frame(object(), frame_width=640.0)

    assert left.angular_z > 0.0
    assert right.angular_z < 0.0
    assert left.line_found is True
    assert right.line_found is True


def test_sharp_turn_tapers_forward_speed_to_minimum_scale():
    use_case, _ = build([
        LineDetection(center_x=0.0, confidence=1.0),
    ])

    command = use_case.process_frame(object(), frame_width=640.0)

    assert command.angular_z == pytest.approx(0.8)
    assert command.linear_x == pytest.approx(0.2 * 0.4)


def test_brief_detection_flicker_coasts_on_last_command():
    use_case, clock = build([
        LineDetection(center_x=240.0, confidence=1.0),
        None,
    ])
    good = use_case.process_frame(object(), frame_width=640.0)
    clock.advance(0.2)

    flicker = use_case.process_frame(object(), frame_width=640.0)

    assert flicker == SteeringCommand(
        good.linear_x,
        good.angular_z,
        line_found=False,
    )
    assert good.line_found is True


def test_detection_lost_past_timeout_stops():
    use_case, clock = build([
        LineDetection(center_x=240.0, confidence=1.0),
        None,
    ])
    use_case.process_frame(object(), frame_width=640.0)
    clock.advance(0.5)

    command = use_case.process_frame(object(), frame_width=640.0)

    assert command == SteeringCommand(0.0, 0.0, line_found=False)


def test_backward_clock_jump_stops_instead_of_coasting():
    use_case, clock = build([
        LineDetection(center_x=240.0, confidence=1.0),
        None,
    ])
    clock.advance(10.0)
    use_case.process_frame(object(), frame_width=640.0)
    clock.advance(-5.0)

    command = use_case.process_frame(object(), frame_width=640.0)

    assert command == SteeringCommand(0.0, 0.0, line_found=False)


def test_line_never_seen_stops_immediately():
    use_case, _ = build([None])

    command = use_case.process_frame(object(), frame_width=640.0)

    assert command == SteeringCommand(0.0, 0.0, line_found=False)


def test_reset_clears_timing_pid_and_stale_coasting_state():
    clock = FakeClock()
    detection = LineDetection(center_x=319.5, confidence=1.0)
    use_case = LineFollowUseCase(
        FakeDetector([detection, detection, detection, None]),
        PID(
            kp=0.0,
            ki=1.0,
            kd=0.0,
            output_min=-0.8,
            output_max=0.8,
        ),
        cruise_speed=0.2,
        max_angular_speed=0.8,
        clock=clock,
    )
    use_case.process_frame(object(), frame_width=640.0)
    clock.advance(1.0)
    assert use_case.process_frame(
        object(), frame_width=640.0
    ).angular_z == pytest.approx(0.5)

    use_case.reset()
    clock.advance(100.0)
    fresh = use_case.process_frame(object(), frame_width=640.0)

    assert fresh.angular_z == pytest.approx(0.0)

    use_case.reset()
    stopped = use_case.process_frame(object(), frame_width=640.0)
    assert stopped == SteeringCommand(0.0, 0.0, line_found=False)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"cruise_speed": -0.1}, "cruise_speed"),
        ({"max_angular_speed": 0.0}, "max_angular_speed"),
        ({"min_speed_scale": 0.0}, "min_speed_scale"),
        ({"min_speed_scale": 1.1}, "min_speed_scale"),
        ({"lost_line_timeout_s": -0.1}, "lost_line_timeout_s"),
    ],
)
def test_constructor_rejects_invalid_control_values(kwargs, message):
    with pytest.raises(ValueError, match=message):
        build([], **kwargs)
