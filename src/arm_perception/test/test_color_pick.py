"""The use case wired to fakes - no camera, no arm, no ROS."""

import pytest

from arm_perception.application.color_motion_map import (
    MotionBinding,
    StaticColorMotionMap,
)
from arm_perception.application.color_pick import ColorPickUseCase
from arm_perception.domain.detection import BlockDetection
from arm_perception.domain.errors import MotionPlaybackError, UnknownColorError
from arm_perception.domain.pick_policy import PickPolicy
from arm_perception.domain.ports import BlockDetector, MotionPlayer


def block(color="red", area=500.0):
    return BlockDetection(
        color=color, center_x=160.0, center_y=120.0,
        width=40.0, height=40.0, angle_deg=0.0, area_px=area,
    )


class FakeDetector(BlockDetector):
    def __init__(self, detections=()):
        self.detections = list(detections)
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        return list(self.detections)


class FakePlayer(MotionPlayer):
    def __init__(self, *, fail=False):
        self.played = []
        self.fail = fail
        self._callback = None
        self._busy = False

    def play(self, motion_name, *, duration_ms=0):
        if self.fail:
            raise MotionPlaybackError("server unavailable")
        self.played.append((motion_name, duration_ms))
        self._busy = True

    def on_finished(self, callback):
        self._callback = callback

    def is_busy(self):
        return self._busy

    def cancel(self):
        self.cancelled = getattr(self, "cancelled", 0) + 1
        self._busy = False

    def complete(self, success=True):
        """Simulate the arm reporting the motion ended."""
        self._busy = False
        self._callback(self.played[-1][0], success)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def build(detector, player, *, confirm_frames=1, cooldown_s=0.0, bindings=None):
    clock = FakeClock()
    motion_map = StaticColorMotionMap(bindings or {
        "red": MotionBinding("pick", 0),
        "blue": MotionBinding("place_left", 1500),
    })
    use_case = ColorPickUseCase(
        detector,
        PickPolicy(confirm_frames=confirm_frames, cooldown_s=cooldown_s),
        motion_map,
        player,
        clock=clock,
    )
    return use_case, clock


class TestDispatch:
    def test_confirmed_block_plays_its_action_group(self):
        player = FakePlayer()
        use_case, _ = build(FakeDetector([block("red")]), player)

        use_case.process_frame(frame=object())

        assert player.played == [("pick", 0)]

    def test_duration_comes_from_the_binding(self):
        player = FakePlayer()
        use_case, _ = build(FakeDetector([block("blue")]), player)

        use_case.process_frame(frame=object())

        assert player.played == [("place_left", 1500)]

    def test_largest_block_wins_when_several_are_visible(self):
        player = FakePlayer()
        detector = FakeDetector([block("red", area=100.0), block("blue", area=900.0)])
        use_case, _ = build(detector, player)

        use_case.process_frame(frame=object())

        assert player.played == [("place_left", 1500)]

    def test_empty_frame_plays_nothing(self):
        player = FakePlayer()
        use_case, _ = build(FakeDetector([]), player)

        assert use_case.process_frame(frame=object()).should_play is False
        assert player.played == []

    def test_one_motion_per_sighting_not_one_per_frame(self):
        player = FakePlayer()
        use_case, clock = build(FakeDetector([block("red")]), player, cooldown_s=2.0)

        for _ in range(10):
            use_case.process_frame(frame=object())
            clock.advance(0.03)

        assert player.played == [("pick", 0)]

    def test_next_sighting_fires_after_completion_and_cooldown(self):
        player = FakePlayer()
        use_case, clock = build(FakeDetector([block("red")]), player, cooldown_s=2.0)

        use_case.process_frame(frame=object())
        clock.advance(1.0)
        player.complete(success=True)
        clock.advance(2.0)

        use_case.process_frame(frame=object())

        assert player.played == [("pick", 0), ("pick", 0)]


class TestPickThenPlace:
    def _seq_case(self, player, *, cooldown_s=0.0):
        return build(
            FakeDetector([block("red")]), player, cooldown_s=cooldown_s,
            bindings={
                "red": MotionBinding(
                    "pick", 0, place="place_left", place_duration_ms=1500
                ),
            },
        )

    def test_place_runs_only_after_pick_succeeds(self):
        player = FakePlayer()
        use_case, _ = self._seq_case(player)

        use_case.process_frame(frame=object())
        assert player.played == [("pick", 0)]      # place not queued yet

        player.complete(success=True)              # pick finished
        assert player.played == [("pick", 0), ("place_left", 1500)]

    def test_camera_is_ignored_while_the_sequence_runs(self):
        player = FakePlayer()
        use_case, clock = self._seq_case(player)

        use_case.process_frame(frame=object())     # -> pick
        clock.advance(0.03)
        # Pick still running: another sighting must not queue a second grasp.
        assert use_case.process_frame(frame=object()).should_play is False
        assert player.played == [("pick", 0)]

    def test_failed_pick_abandons_the_place(self):
        player = FakePlayer()
        use_case, clock = self._seq_case(player, cooldown_s=5.0)

        use_case.process_frame(frame=object())
        player.complete(success=False)             # grasp failed
        assert player.played == [("pick", 0)]      # nothing flung at the bin

        # And the pipeline is in cooldown, not wedged mid-sequence.
        assert use_case.process_frame(frame=object()).should_play is False

    def test_re_enabling_mid_sequence_does_not_stack_a_second_motion(self):
        player = FakePlayer()
        use_case, _ = self._seq_case(player)

        use_case.process_frame(frame=object())     # -> pick (sequence active)
        use_case.disable()
        use_case.enable()                          # must not reset the policy

        # A fresh sighting must not queue a second grasp over the running one.
        assert use_case.process_frame(frame=object()).should_play is False
        assert player.played == [("pick", 0)]

        # The in-flight sequence still finishes normally.
        player.complete(success=True)
        assert player.played == [("pick", 0), ("place_left", 1500)]

    def test_stale_completion_after_release_is_ignored(self):
        player = FakePlayer()
        use_case, _ = self._seq_case(player)

        use_case.process_frame(frame=object())
        player.complete(success=True)              # pick -> place
        player.complete(success=True)              # place -> release
        player.complete(success=True)              # stale duplicate: no-op

        assert player.played == [("pick", 0), ("place_left", 1500)]

    def test_sequence_completes_then_cools_down_and_repeats(self):
        player = FakePlayer()
        use_case, clock = self._seq_case(player, cooldown_s=2.0)

        use_case.process_frame(frame=object())
        player.complete(success=True)              # pick
        player.complete(success=True)              # place -> release
        clock.advance(2.0)
        use_case.process_frame(frame=object())     # cooldown elapsed -> pick again

        assert player.played == [
            ("pick", 0), ("place_left", 1500), ("pick", 0),
        ]


class TestWatchdog:
    def _case(self, player, *, timeout=10.0, cooldown_s=0.0, place="place_left"):
        clock = FakeClock()
        use_case = ColorPickUseCase(
            FakeDetector([block("red")]),
            PickPolicy(confirm_frames=1, cooldown_s=cooldown_s),
            StaticColorMotionMap({"red": MotionBinding("pick", 0, place=place)}),
            player,
            clock=clock,
            motion_timeout_s=timeout,
        )
        return use_case, clock

    def test_overrunning_motion_is_cancelled_and_recovered(self):
        player = FakePlayer()
        use_case, clock = self._case(player, timeout=10.0, cooldown_s=5.0)

        use_case.process_frame(frame=object())     # -> pick, deadline at t=10
        clock.advance(11.0)                        # blow through it
        decision = use_case.process_frame(frame=object())

        assert decision.reason == "motion timed out"
        assert player.cancelled == 1

        # A late completion for the cancelled motion is ignored, not chained.
        player.complete(success=True)
        assert player.played == [("pick", 0)]      # place never queued

    def test_disabled_watchdog_never_times_out(self):
        player = FakePlayer()
        clock = FakeClock()
        use_case = ColorPickUseCase(
            FakeDetector([block("red")]),
            PickPolicy(confirm_frames=1, cooldown_s=0.0),
            StaticColorMotionMap({"red": MotionBinding("pick", 0)}),
            player,
            clock=clock,
            motion_timeout_s=None,
        )

        use_case.process_frame(frame=object())
        clock.advance(10_000.0)
        decision = use_case.process_frame(frame=object())

        assert decision.reason != "motion timed out"
        assert getattr(player, "cancelled", 0) == 0

    def test_each_step_gets_its_own_deadline(self):
        player = FakePlayer()
        use_case, clock = self._case(player, timeout=10.0)

        use_case.process_frame(frame=object())     # pick, deadline t=10
        clock.advance(8.0)
        player.complete(success=True)              # pick done -> place, deadline t=18
        assert player.played == [("pick", 0), ("place_left", 0)]

        clock.advance(8.0)                         # t=16, before the place deadline
        use_case.process_frame(frame=object())
        assert getattr(player, "cancelled", 0) == 0  # place not cut short by pick's clock

    def test_watchdog_recovers_even_while_disabled(self):
        """The timer path (check_watchdog) must fire regardless of enable state."""
        player = FakePlayer()
        use_case, clock = self._case(player, timeout=10.0, cooldown_s=5.0)

        use_case.process_frame(frame=object())     # pick, deadline t=10
        use_case.disable()                         # pause triggering
        clock.advance(11.0)

        assert use_case.check_watchdog() is True    # as the node timer would call
        assert player.cancelled == 1
        assert use_case.check_watchdog() is False   # nothing left to recover

    def test_non_positive_timeout_is_rejected(self):
        with pytest.raises(ValueError):
            ColorPickUseCase(
                FakeDetector(), PickPolicy(), StaticColorMotionMap(
                    {"red": MotionBinding("pick")}
                ), FakePlayer(), clock=FakeClock(), motion_timeout_s=0.0,
            )


class TestFailureHandling:
    def test_playback_failure_does_not_wedge_the_pipeline(self):
        player = FakePlayer(fail=True)
        use_case, clock = build(FakeDetector([block("red")]), player, cooldown_s=1.0)

        decision = use_case.process_frame(frame=object())
        assert decision.should_play is False
        assert "playback failed" in decision.reason

        # After cooldown the pipeline must be willing to try again.
        player.fail = False
        clock.advance(1.5)
        use_case.process_frame(frame=object())

        assert player.played == [("pick", 0)]

    def test_unmapped_colour_is_reported_not_raised(self):
        player = FakePlayer()
        use_case, _ = build(
            FakeDetector([block("green")]), player,
            bindings={"red": MotionBinding("pick", 0)},
        )

        decision = use_case.process_frame(frame=object())

        assert decision.should_play is False
        assert "green" in decision.reason
        assert player.played == []

    def test_unmapped_colour_does_not_block_a_later_mapped_one(self):
        player = FakePlayer()
        detector = FakeDetector([block("green")])
        use_case, clock = build(
            detector, player, cooldown_s=0.0,
            bindings={"red": MotionBinding("pick", 0)},
        )

        use_case.process_frame(frame=object())
        detector.detections = [block("red")]
        clock.advance(0.1)
        use_case.process_frame(frame=object())

        assert player.played == [("pick", 0)]

    def test_failed_motion_still_starts_the_cooldown(self):
        player = FakePlayer()
        use_case, clock = build(FakeDetector([block("red")]), player, cooldown_s=5.0)

        use_case.process_frame(frame=object())
        player.complete(success=False)
        clock.advance(1.0)

        assert use_case.process_frame(frame=object()).should_play is False


class TestEnableDisable:
    def test_disabled_pipeline_still_detects_but_never_plays(self):
        player = FakePlayer()
        detector = FakeDetector([block("red")])
        use_case, _ = build(detector, player)
        use_case.disable()

        decision = use_case.process_frame(frame=object())

        assert decision.should_play is False
        assert detector.calls == 1  # detection still ran, for the debug topic
        assert player.played == []

    def test_re_enabling_resumes_triggering(self):
        player = FakePlayer()
        use_case, _ = build(FakeDetector([block("red")]), player)
        use_case.disable()
        use_case.process_frame(frame=object())
        use_case.enable()

        use_case.process_frame(frame=object())

        assert player.played == [("pick", 0)]


class TestDetectionSink:
    def test_sink_sees_every_frames_detections(self):
        seen = []
        detector = FakeDetector([block("red"), block("blue")])
        clock = FakeClock()
        use_case = ColorPickUseCase(
            detector,
            PickPolicy(confirm_frames=10),
            StaticColorMotionMap({"red": MotionBinding("pick", 0)}),
            FakePlayer(),
            clock=clock,
            detection_sink=seen.append,
        )

        use_case.process_frame(frame=object())

        assert len(seen) == 1
        assert {d.color for d in seen[0]} == {"red", "blue"}


class TestMotionMap:
    def test_unknown_colour_raises(self):
        motion_map = StaticColorMotionMap({"red": MotionBinding("pick")})
        with pytest.raises(UnknownColorError):
            motion_map.motion_for("purple")

    def test_empty_map_is_rejected(self):
        with pytest.raises(ValueError):
            StaticColorMotionMap({})

    def test_binding_validates_its_fields(self):
        with pytest.raises(ValueError):
            MotionBinding("")
        with pytest.raises(ValueError):
            MotionBinding("pick", duration_ms=-1)
        with pytest.raises(ValueError):
            MotionBinding("pick", place="")
        with pytest.raises(ValueError):
            MotionBinding("pick", place="place_left", place_duration_ms=-1)
        with pytest.raises(ValueError):
            MotionBinding("pick", place_duration_ms=100)  # budget, nothing to place

    def test_steps_are_pick_then_place(self):
        binding = MotionBinding("pick", 0, place="place_left", place_duration_ms=1500)
        assert [(s.name, s.duration_ms) for s in binding.steps()] == [
            ("pick", 0), ("place_left", 1500),
        ]

    def test_steps_is_pick_only_without_place(self):
        assert [s.name for s in MotionBinding("pick", 0).steps()] == ["pick"]

    def test_steps_for_returns_the_sequence(self):
        motion_map = StaticColorMotionMap(
            {"red": MotionBinding("pick", 0, place="place_left")}
        )
        assert [s.name for s in motion_map.steps_for("red")] == ["pick", "place_left"]


class TestSinkRobustness:
    def test_a_broken_sink_does_not_stop_the_pick(self):
        """A publisher bug is telemetry breakage, not a reason to stop picking."""
        def exploding_sink(_detections):
            raise RuntimeError("publisher blew up")

        player = FakePlayer()
        clock = FakeClock()
        use_case = ColorPickUseCase(
            FakeDetector([block("red")]),
            PickPolicy(confirm_frames=1),
            StaticColorMotionMap({"red": MotionBinding("pick", 0)}),
            player,
            clock=clock,
            detection_sink=exploding_sink,
        )

        decision = use_case.process_frame(frame=object())

        assert decision.should_play is True
        assert player.played == [("pick", 0)]
