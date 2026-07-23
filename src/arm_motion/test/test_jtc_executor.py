"""Executor tests with fake action clients — no ROS graph required.

These cover the failure paths that are hard to provoke against a real
controller: a partially-accepted motion, cancellation targeting, and two
callers racing for the arm.
"""

import threading
import time

import pytest
from control_msgs.action import FollowJointTrajectory
from rclpy.clock import ClockType
from rclpy.time import Time

from arm_motion.domain.errors import MotionCancelledError, MotionExecutionError
from arm_motion.domain.joint_spec import JointKind, JointSpec
from arm_motion.domain.motion import Motion, MotionStep
from arm_motion.domain.robot_profile import build_profile
from arm_motion.domain.servo_scale import ServoScale
from arm_motion.infrastructure.ros import jtc_executor as module
from arm_motion.infrastructure.ros.jtc_executor import JtcTrajectoryExecutor

STATUS_SUCCEEDED = 4
STATUS_ABORTED = 5


class FakeClock:
    def __init__(self, seconds=1.0):
        self._nanoseconds = int(seconds * 1_000_000_000)

    def now(self):
        return Time(
            nanoseconds=self._nanoseconds,
            clock_type=ClockType.ROS_TIME,
        )

    def set(self, seconds):
        self._nanoseconds = int(seconds * 1_000_000_000)

    def advance(self, seconds):
        self._nanoseconds += int(seconds * 1_000_000_000)


class FakeNode:
    def __init__(self, clock):
        self._clock = clock

    def get_clock(self):
        return self._clock


class FakeFuture:
    def __init__(self, value=None, ready=True):
        self._value = value
        self._ready = ready
        self._callbacks = []

    def resolve(self, value):
        self._value = value
        self._ready = True
        for callback in list(self._callbacks):
            callback(self)
        self._callbacks.clear()

    def add_done_callback(self, callback):
        # Mirrors rclpy: fires immediately when the future is already done.
        if self._ready:
            callback(self)
        else:
            self._callbacks.append(callback)

    def done(self):
        return self._ready

    def result(self):
        return self._value


class FakeResult:
    def __init__(self, status=STATUS_SUCCEEDED, error_code=0):
        self.status = status
        self.result = type(
            'R', (), {'error_code': error_code, 'error_string': 'fake failure'}
        )()


class FakeHandle:
    def __init__(self, accepted=True, duration_s=0.05, result=None):
        self.accepted = accepted
        self.cancelled = False
        self._duration_s = duration_s
        self._result = result or FakeResult()
        self._future = FakeFuture(ready=False)

    def get_result_async(self):
        def finish():
            time.sleep(self._duration_s)
            if not self._future.done():
                self._future.resolve(self._result)
        threading.Thread(target=finish, daemon=True).start()
        return self._future

    def cancel_goal_async(self):
        self.cancelled = True
        # A real controller answers a cancel with a CANCELED result.
        self._future.resolve(FakeResult(status=6))
        return FakeFuture(None)


class ManualHandle(FakeHandle):
    """A handle whose result is resolved explicitly by a deterministic test."""

    def get_result_async(self):
        return self._future

    def finish(self):
        self._future.resolve(self._result)


class JumpOnResultHandle(ManualHandle):
    """Move the fake ROS clock when the result wait begins."""

    def __init__(self, clock, jump_to):
        super().__init__()
        self._clock = clock
        self._jump_to = jump_to

    def get_result_async(self):
        self._clock.set(self._jump_to)
        return self._future


class FakeClient:
    def __init__(self, handle=None, server_available=True):
        self.handle = handle if handle is not None else FakeHandle()
        self.server_available = server_available
        self.goals_sent = 0
        #: When set, send_goal_async returns an unresolved future, simulating
        #: a server that has not answered the goal request yet.
        self.defer_goal_response = False
        self.pending = []

    def wait_for_server(self, timeout_sec=None):
        return self.server_available

    def send_goal_async(self, goal):
        self.goals_sent += 1
        if self.defer_goal_response:
            future = FakeFuture(ready=False)
            self.pending.append(future)
            return future
        return FakeFuture(self.handle)

    def accept_pending(self):
        """Let the 'server' finally accept the goals it was sitting on."""
        for future in self.pending:
            future.resolve(self.handle)
        self.pending.clear()


@pytest.fixture
def profile():
    joints = [
        JointSpec('joint1', lower=-2.09, upper=2.09),
        JointSpec(
            'r_joint', lower=-1.57, upper=1.57, kind=JointKind.GRIPPER,
            group='gripper', open_position=-1.0, closed_position=0.3,
        ),
    ]
    scales = [
        ServoScale(1, 'joint1', 0, 1000, -2.0943951, 2.0943951),
        ServoScale(10, 'r_joint', 0, 1000, -1.57, 1.57),
    ]
    return build_profile(joints, scales)


@pytest.fixture
def executor(profile, monkeypatch):
    """An executor whose ActionClients are fakes."""
    created = {}
    clock = FakeClock()

    def fake_action_client(node, action_type, namespace, callback_group=None):
        client = FakeClient()
        created[namespace] = client
        return client

    monkeypatch.setattr(module, 'ActionClient', fake_action_client)
    ex = JtcTrajectoryExecutor(
        node=FakeNode(clock),
        profile=profile,
        controller_namespaces={'arm': '/arm_ctl', 'gripper': '/grip_ctl'},
    )
    ex.fakes = created
    ex.clock = clock
    return ex


def make_motion(profile, steps=2, duration_ms=100):
    pose = profile.home_pose()
    return Motion('m', tuple(MotionStep(pose, duration_ms) for _ in range(steps)))


def _install_manual_handles(executor):
    handles = [ManualHandle(), ManualHandle()]
    executor.fakes['/arm_ctl'].handle = handles[0]
    executor.fakes['/grip_ctl'].handle = handles[1]
    return handles


def _drive_polling(
    monkeypatch,
    clock,
    handles,
    *,
    wall_step_s,
    ros_step_s,
    finish_after=None,
):
    """Advance wall and ROS time independently on each executor poll."""
    timing = {'polls': 0, 'wall_s': 0.0, 'ros_s': 0.0}

    def fake_monotonic():
        return timing['wall_s']

    def advance(_interval):
        timing['polls'] += 1
        timing['wall_s'] += wall_step_s
        timing['ros_s'] += ros_step_s
        clock.advance(ros_step_s)
        if finish_after is not None and timing['polls'] == finish_after:
            for handle in handles:
                handle.finish()

    monkeypatch.setattr(module.time, 'monotonic', fake_monotonic)
    monkeypatch.setattr(module.time, 'sleep', advance)
    return timing


class TestHappyPath:

    def test_both_groups_receive_a_goal(self, executor, profile):
        executor.execute(make_motion(profile))
        assert executor.fakes['/arm_ctl'].goals_sent == 1
        assert executor.fakes['/grip_ctl'].goals_sent == 1

    def test_progress_is_reported_for_every_step(self, executor, profile):
        seen = []
        executor.execute(
            make_motion(profile, steps=3), on_progress=lambda i, n: seen.append((i, n))
        )
        assert seen[-1] == (3, 3)

    def test_not_busy_after_completion(self, executor, profile):
        executor.execute(make_motion(profile))
        assert not executor.is_busy()


class TestRosTimeDeadline:

    def test_slow_simulation_finishes_within_ros_time_budget(
        self, executor, profile, monkeypatch
    ):
        handles = _install_manual_handles(executor)
        timing = _drive_polling(
            monkeypatch,
            executor.clock,
            handles,
            wall_step_s=2.0,
            ros_step_s=0.4,
            finish_after=10,
        )
        progress = []
        pose = profile.home_pose()
        motion = Motion(
            'm',
            tuple(
                MotionStep(pose, duration_ms)
                for duration_ms in (1000, 1000, 2400)
            ),
        )

        executor.execute(
            motion,
            on_progress=lambda i, n: progress.append((i, n)),
        )

        assert timing['wall_s'] == pytest.approx(20.0)
        assert timing['ros_s'] == pytest.approx(4.0)
        assert progress == [(0, 3), (1, 3), (2, 3), (3, 3)]

    def test_motion_times_out_only_after_ros_time_budget(
        self, executor, profile, monkeypatch
    ):
        handles = _install_manual_handles(executor)
        timing = _drive_polling(
            monkeypatch,
            executor.clock,
            handles,
            wall_step_s=0.01,
            ros_step_s=1.0,
        )

        with pytest.raises(MotionExecutionError, match='did not finish'):
            executor.execute(make_motion(profile, steps=1, duration_ms=100))

        assert timing['wall_s'] < 0.1
        assert timing['ros_s'] == pytest.approx(6.0)
        assert all(handle.cancelled for handle in handles)

    def test_waits_for_sim_clock_before_starting_deadline(
        self, executor, profile, monkeypatch
    ):
        executor.clock.set(0.0)
        handles = _install_manual_handles(executor)
        timing = {'polls': 0, 'wall_s': 0.0}

        def fake_monotonic():
            return timing['wall_s']

        def advance(_interval):
            timing['polls'] += 1
            timing['wall_s'] += 10.0
            if timing['polls'] == 3:
                executor.clock.set(1.0)
            elif timing['polls'] == 4:
                executor.clock.advance(0.1)
                for handle in handles:
                    handle.finish()

        monkeypatch.setattr(module.time, 'monotonic', fake_monotonic)
        monkeypatch.setattr(module.time, 'sleep', advance)

        executor.execute(make_motion(profile, steps=1, duration_ms=100))

        assert timing == {'polls': 4, 'wall_s': 40.0}

    def test_missing_sim_clock_times_out_and_releases_executor(
        self, executor, profile, monkeypatch
    ):
        executor.clock.set(0.0)
        handles = _install_manual_handles(executor)
        timing = {'wall_s': 0.0}

        monkeypatch.setattr(
            module.time, 'perf_counter', lambda: timing['wall_s']
        )

        def advance(_interval):
            timing['wall_s'] += 1.0
            if executor.clock.now().nanoseconds != 0:
                for handle in handles:
                    handle.finish()

        monkeypatch.setattr(module.time, 'sleep', advance)

        with pytest.raises(MotionExecutionError, match='ROS clock never started'):
            executor.execute(make_motion(profile))

        assert not executor.is_busy()
        assert executor.fakes['/arm_ctl'].goals_sent == 0

        executor.clock.set(1.0)
        executor.execute(make_motion(profile))

        assert executor.fakes['/arm_ctl'].goals_sent == 1
        assert not executor.is_busy()

    def test_backward_clock_jump_aborts_motion(
        self, executor, profile, monkeypatch
    ):
        handles = _install_manual_handles(executor)

        def jump_back(_interval):
            executor.clock.set(0.5)

        monkeypatch.setattr(module.time, 'sleep', jump_back)

        with pytest.raises(
            MotionExecutionError, match='clock moved backwards'
        ):
            executor.execute(make_motion(profile, duration_ms=1000))

        assert all(handle.cancelled for handle in handles)

    def test_partial_backward_clock_jump_aborts_and_releases_executor(
        self, executor, profile, monkeypatch
    ):
        handles = _install_manual_handles(executor)
        samples = iter((4.0, 2.0))

        def advance_then_jump_back(_interval):
            executor.clock.set(next(samples))

        monkeypatch.setattr(module.time, 'sleep', advance_then_jump_back)

        with pytest.raises(MotionExecutionError, match='clock moved backwards'):
            executor.execute(make_motion(profile, duration_ms=1000))

        assert all(handle.cancelled for handle in handles)
        assert not executor.is_busy()

        later_handles = _install_manual_handles(executor)

        def finish_later(_interval):
            executor.clock.advance(0.1)
            for handle in later_handles:
                handle.finish()

        monkeypatch.setattr(module.time, 'sleep', finish_later)
        executor.execute(make_motion(profile))

        assert not executor.is_busy()

    def test_backward_jump_between_goal_and_result_waits_aborts_and_releases_lock(
        self, executor, profile, monkeypatch
    ):
        handles = [
            JumpOnResultHandle(executor.clock, 2.0),
            JumpOnResultHandle(executor.clock, 2.0),
        ]
        for namespace, handle in zip(('/arm_ctl', '/grip_ctl'), handles):
            executor.fakes[namespace].handle = handle
            executor.fakes[namespace].defer_goal_response = True

        polls = {'count': 0}

        def advance(_interval):
            polls['count'] += 1
            if polls['count'] == 1:
                executor.clock.set(4.0)
            elif polls['count'] == 2:
                for client in executor.fakes.values():
                    client.accept_pending()
            else:
                executor.clock.advance(10.0)

        monkeypatch.setattr(module.time, 'sleep', advance)

        with pytest.raises(MotionExecutionError, match='clock moved backwards'):
            executor.execute(make_motion(profile, duration_ms=1000))

        assert all(handle.cancelled for handle in handles)
        assert not executor.is_busy()


class TestPartialAcceptance:

    def test_rejection_by_one_group_cancels_the_other(self, executor, profile):
        """The arm must not keep moving when the gripper goal is refused."""
        accepted = FakeHandle(accepted=True, duration_s=5.0)
        executor.fakes['/arm_ctl'].handle = accepted
        executor.fakes['/grip_ctl'].handle = FakeHandle(accepted=False)

        with pytest.raises(MotionExecutionError):
            executor.execute(make_motion(profile))

        assert accepted.cancelled, 'accepted arm goal was left running'

    def test_controller_error_cancels_the_other_group(self, executor, profile):
        failing = FakeHandle(result=FakeResult(status=STATUS_ABORTED, error_code=-1))
        other = FakeHandle(duration_s=5.0)
        executor.fakes['/arm_ctl'].handle = failing
        executor.fakes['/grip_ctl'].handle = other

        with pytest.raises(MotionExecutionError):
            executor.execute(make_motion(profile))

        assert other.cancelled

    def test_unavailable_server_is_reported(self, executor, profile):
        executor.fakes['/grip_ctl'].server_available = False
        with pytest.raises(MotionExecutionError, match='not available'):
            executor.execute(make_motion(profile))

    def test_early_failure_aborts_the_still_running_group(self, executor, profile):
        """One group failing quickly must cancel the other before it finishes."""
        quick_fail = FakeHandle(
            duration_s=0.05, result=FakeResult(status=STATUS_ABORTED, error_code=-1)
        )
        long_running = FakeHandle(duration_s=10.0)
        executor.fakes['/arm_ctl'].handle = quick_fail
        executor.fakes['/grip_ctl'].handle = long_running

        start = time.time()
        with pytest.raises(MotionExecutionError):
            executor.execute(make_motion(profile, duration_ms=10000))
        elapsed = time.time() - start

        # We must not have waited for the 10 s group to finish on its own.
        assert elapsed < 2.0, f'did not abort early (waited {elapsed:.1f}s)'
        assert long_running.cancelled


class TestCancellation:

    def test_cancel_stops_the_running_motion(self, executor, profile):
        handle = FakeHandle(duration_s=5.0)
        executor.fakes['/arm_ctl'].handle = handle
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=5.0)

        error = {}

        def run():
            try:
                executor.execute(make_motion(profile, duration_ms=5000))
            except Exception as exc:  # noqa: BLE001
                error['exc'] = exc

        thread = threading.Thread(target=run)
        thread.start()
        time.sleep(0.2)
        executor.cancel()
        thread.join(timeout=5.0)

        assert not thread.is_alive()
        assert isinstance(error.get('exc'), MotionCancelledError)
        assert handle.cancelled

    def test_cancelled_before_start_sends_no_goal(self, executor, profile):
        """A goal cancelled while queued must never reach the controller."""
        with pytest.raises(MotionCancelledError, match='before it started'):
            executor.execute(make_motion(profile), should_cancel=lambda: True)
        assert executor.fakes['/arm_ctl'].goals_sent == 0
        assert executor.fakes['/grip_ctl'].goals_sent == 0

    def test_should_cancel_callback_aborts(self, executor, profile):
        executor.fakes['/arm_ctl'].handle = FakeHandle(duration_s=5.0)
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=5.0)
        with pytest.raises(MotionCancelledError):
            executor.execute(
                make_motion(profile, duration_ms=5000), should_cancel=lambda: True
            )

    def test_cancel_with_nothing_running_is_harmless(self, executor):
        assert executor.cancel() is False

    def test_goal_accepted_after_a_cancel_is_still_stopped(
        self, executor, profile
    ):
        """The window between 'goal sent' and 'goal accepted'.

        If we give up while the request is still in flight, the controller may
        accept it moments later. That goal must be cancelled too, or the arm
        moves after we have already reported failure.
        """
        late = FakeHandle(duration_s=5.0)
        arm = executor.fakes['/arm_ctl']
        arm.handle = late
        arm.defer_goal_response = True
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=5.0)

        error = {}

        def run():
            try:
                executor.execute(make_motion(profile, duration_ms=1000))
            except Exception as exc:  # noqa: BLE001
                error['exc'] = exc

        thread = threading.Thread(target=run)
        thread.start()
        time.sleep(0.2)

        executor.cancel()
        thread.join(timeout=5.0)
        assert not thread.is_alive()
        assert isinstance(error.get('exc'), MotionCancelledError)
        assert not late.cancelled, 'precondition: not accepted yet'

        # The server finally answers, after we already gave up.
        arm.accept_pending()
        assert late.cancelled, 'a late-accepted goal was left running'


class TestOwnerScopedCancel:

    def test_cancel_with_a_different_owner_is_ignored(self, executor, profile):
        handle = FakeHandle(duration_s=3.0)
        executor.fakes['/arm_ctl'].handle = handle
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=3.0)

        thread = threading.Thread(
            target=_swallow(executor, profile, 3000, owner='jog'), daemon=True
        )
        thread.start()
        time.sleep(0.2)

        # A queued PlayMotion goal cancelling itself must not stop the jog.
        assert executor.cancel(owner=b'some-other-goal') is False
        assert not handle.cancelled

        assert executor.cancel(owner='jog') is True
        thread.join(timeout=5.0)
        assert handle.cancelled

    def test_untagged_cancel_still_stops_anything(self, executor, profile):
        handle = FakeHandle(duration_s=3.0)
        executor.fakes['/arm_ctl'].handle = handle
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=3.0)

        thread = threading.Thread(
            target=_swallow(executor, profile, 3000, owner='jog'), daemon=True
        )
        thread.start()
        time.sleep(0.2)
        assert executor.cancel() is True
        thread.join(timeout=5.0)
        assert handle.cancelled


def _swallow(executor, profile, duration_ms, owner=None):
    """Run a motion in a thread, absorbing the cancel these tests provoke."""
    def run():
        try:
            executor.execute(
                make_motion(profile, duration_ms=duration_ms), owner=owner
            )
        except MotionCancelledError:
            pass
    return run


class TestSerialisation:

    def test_second_caller_is_refused_rather_than_interleaved(
        self, executor, profile
    ):
        executor.fakes['/arm_ctl'].handle = FakeHandle(duration_s=2.0)
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=2.0)

        thread = threading.Thread(target=_swallow(executor, profile, 2000))
        thread.start()
        time.sleep(0.2)

        with pytest.raises(MotionExecutionError, match='still running'):
            executor.execute(make_motion(profile), acquire_timeout_s=0.1)

        executor.cancel()
        thread.join(timeout=5.0)

    def test_preempt_takes_over_from_the_running_motion(self, executor, profile):
        first = FakeHandle(duration_s=3.0)
        executor.fakes['/arm_ctl'].handle = first
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=3.0)

        thread = threading.Thread(target=_swallow(executor, profile, 3000), daemon=True)
        thread.start()
        time.sleep(0.2)
        assert executor.is_busy()

        # Fresh fast handles for the preempting move.
        executor.fakes['/arm_ctl'].handle = FakeHandle(duration_s=0.05)
        executor.fakes['/grip_ctl'].handle = FakeHandle(duration_s=0.05)
        executor.execute(make_motion(profile), preempt=True, acquire_timeout_s=5.0)

        assert first.cancelled, 'the superseded motion was not cancelled'
        thread.join(timeout=5.0)


def test_trajectory_layout(executor, profile):
    """Waypoint times must be cumulative and the groups must line up."""
    motion = Motion(
        'm',
        (
            MotionStep(profile.home_pose(), 200),
            MotionStep(profile.home_pose(), 500),
            MotionStep(profile.home_pose(), 1300),
        ),
    )
    trajectories = executor._build_trajectories(motion)

    assert set(trajectories) == {'arm', 'gripper'}
    arm = trajectories['arm']
    assert arm.joint_names == ['joint1']
    times = [
        p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in arm.points
    ]
    assert times == pytest.approx([0.2, 0.7, 2.0])
    # Both groups share the same time base, so they stay synchronised.
    gripper_times = [
        p.time_from_start.sec + p.time_from_start.nanosec / 1e9
        for p in trajectories['gripper'].points
    ]
    assert gripper_times == pytest.approx(times)


def test_result_type_is_the_real_action(executor):
    """Guard against the fake drifting from the real action definition."""
    assert FollowJointTrajectory.Result.SUCCESSFUL == 0
