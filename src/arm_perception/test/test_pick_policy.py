"""The debounce state machine, exercised without ROS, OpenCV, or sleeping."""

import pytest

from arm_perception.domain.detection import BlockDetection
from arm_perception.domain.pick_policy import PickPolicy, PolicyState


def block(color="red", x=160.0, y=120.0, area=500.0):
    return BlockDetection(
        color=color, center_x=x, center_y=y,
        width=40.0, height=40.0, angle_deg=0.0, area_px=area,
    )


class TestConfirmation:
    def test_single_frame_does_not_trigger(self):
        policy = PickPolicy(confirm_frames=3)
        assert policy.observe(block(), now=0.0).should_play is False

    def test_triggers_on_the_nth_consecutive_frame(self):
        policy = PickPolicy(confirm_frames=3)
        assert policy.observe(block(), 0.0).should_play is False
        assert policy.observe(block(), 0.1).should_play is False

        decision = policy.observe(block(), 0.2)
        assert decision.should_play is True
        assert decision.color == "red"

    def test_confirm_frames_of_one_triggers_immediately(self):
        policy = PickPolicy(confirm_frames=1)
        assert policy.observe(block(), 0.0).should_play is True

    def test_losing_the_block_resets_the_streak(self):
        policy = PickPolicy(confirm_frames=3)
        policy.observe(block(), 0.0)
        policy.observe(block(), 0.1)
        policy.observe(None, 0.2)  # block left the frame

        assert policy.streak == 0
        assert policy.observe(block(), 0.3).should_play is False

    def test_a_different_colour_restarts_the_streak(self):
        policy = PickPolicy(confirm_frames=3)
        policy.observe(block("red"), 0.0)
        policy.observe(block("red"), 0.1)

        # Blue appears - must not inherit red's two confirmations.
        decision = policy.observe(block("blue"), 0.2)
        assert decision.should_play is False
        assert policy.streak == 1

    def test_rejects_nonsense_construction(self):
        with pytest.raises(ValueError):
            PickPolicy(confirm_frames=0)
        with pytest.raises(ValueError):
            PickPolicy(cooldown_s=-1.0)
        with pytest.raises(ValueError):
            PickPolicy(center_tolerance_px=0.0)


class TestBusyAndCooldown:
    def test_no_second_trigger_while_the_arm_is_playing(self):
        policy = PickPolicy(confirm_frames=1)
        assert policy.observe(block(), 0.0).should_play is True
        assert policy.state is PolicyState.PLAYING

        for t in (0.1, 0.2, 0.3):
            decision = policy.observe(block(), t)
            assert decision.should_play is False
            assert decision.reason == "arm busy"

    def test_cooldown_blocks_retrigger_then_expires(self):
        policy = PickPolicy(confirm_frames=1, cooldown_s=3.0)
        policy.observe(block(), 0.0)
        policy.motion_finished(now=1.0)
        assert policy.state is PolicyState.COOLDOWN

        assert policy.observe(block(), 2.0).should_play is False
        assert policy.observe(block(), 3.9).should_play is False
        # 1.0 + 3.0 = 4.0, so this frame is the first one allowed through.
        assert policy.observe(block(), 4.0).should_play is True

    def test_cooldown_runs_after_a_failed_motion_too(self):
        policy = PickPolicy(confirm_frames=1, cooldown_s=2.0)
        policy.observe(block(), 0.0)
        policy.motion_finished(now=0.5)  # failure path calls this identically

        assert policy.observe(block(), 1.0).should_play is False

    def test_confirmation_restarts_after_cooldown(self):
        policy = PickPolicy(confirm_frames=3, cooldown_s=1.0)
        for t in (0.0, 0.1, 0.2):
            policy.observe(block(), t)
        policy.motion_finished(now=0.3)

        # Cooldown over: needs a fresh full streak, not one more frame.
        assert policy.observe(block(), 1.5).should_play is False
        assert policy.observe(block(), 1.6).should_play is False
        assert policy.observe(block(), 1.7).should_play is True

    def test_reset_clears_everything(self):
        policy = PickPolicy(confirm_frames=1, cooldown_s=10.0)
        policy.observe(block(), 0.0)
        policy.motion_finished(now=0.1)

        policy.reset()

        assert policy.state is PolicyState.IDLE
        assert policy.observe(block(), 0.2).should_play is True


class TestCentering:
    def test_off_centre_block_does_not_trigger(self):
        policy = PickPolicy(confirm_frames=1, center_tolerance_px=50.0)
        far = block(x=300.0, y=120.0)

        decision = policy.observe(far, 0.0, frame_center=(160.0, 120.0))

        assert decision.should_play is False
        assert "centre" in decision.reason

    def test_centred_block_triggers(self):
        policy = PickPolicy(confirm_frames=1, center_tolerance_px=50.0)
        near = block(x=180.0, y=130.0)

        decision = policy.observe(near, 0.0, frame_center=(160.0, 120.0))

        assert decision.should_play is True

    def test_check_is_skipped_when_tolerance_is_unset(self):
        policy = PickPolicy(confirm_frames=1, center_tolerance_px=None)
        far = block(x=1000.0, y=1000.0)

        assert policy.observe(far, 0.0, frame_center=(160.0, 120.0)).should_play is True

    def test_drifting_off_centre_resets_the_streak(self):
        policy = PickPolicy(confirm_frames=3, center_tolerance_px=50.0)
        center = (160.0, 120.0)
        policy.observe(block(x=160.0), 0.0, frame_center=center)
        policy.observe(block(x=160.0), 0.1, frame_center=center)
        policy.observe(block(x=400.0), 0.2, frame_center=center)  # drifted away

        assert policy.streak == 0
