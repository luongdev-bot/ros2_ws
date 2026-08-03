"""Closed-loop ROS 2 pick-and-place executor for colour detections."""

import math
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from ...application.base_control import BaseGains, Pose2D
from ...application.grasp_plan import (
    GraspConfig,
    plan_pick,
    plan_pick_and_place,
    plan_place,
)
from ...application.mobile_plan import base_goal_to_center_target
from ...application.target_selection import DetectedBlock, rank_targets
from ..depth_localizer import DepthLocalizer
from .base_driver import BaseDriver
from .trajectory_client import JointTrajectoryActionClient


class GraspExecutorNode(Node):
    """Localize ranked detections and execute one grasp cycle at a time."""

    _BIN_COLORS = ("red", "green", "blue", "yellow")
    _GRIPPER_MOVE_S = 1.0
    _DEFAULT_GRASP_SWEET_SPOT = (0.025, -0.22)
    _DEFAULT_PLACE_SWEET_SPOT = (0.025, -0.32)
    _DEFAULT_HOME_ODOM = (0.0, 0.0, 0.0)

    def __init__(self) -> None:
        super().__init__("grasp_executor")
        self._declare_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._latest_detections: list[DetectedBlock] = []
        self._done_colours: set[str] = set()
        self._cycle_running = False
        self._worker_thread = None

        self._bins = self._load_bins()
        self._grasp_order = self._load_grasp_order()
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._grasp_config = self._load_grasp_config()
        self._auto_grasp = bool(self.get_parameter("auto_grasp").value)
        self._mobile_enabled = bool(
            self.get_parameter("mobile_enabled").value
        )
        self._grasp_sweet_spot = self._load_vector_parameter(
            "grasp_sweet_spot",
            self._DEFAULT_GRASP_SWEET_SPOT,
            2,
        )
        self._place_sweet_spot = self._load_vector_parameter(
            "place_sweet_spot",
            self._DEFAULT_PLACE_SWEET_SPOT,
            2,
        )
        self._home_odom = self._load_vector_parameter(
            "home_odom",
            self._DEFAULT_HOME_ODOM,
            3,
        )
        self._block_pick_z = self._load_scalar_parameter(
            "block_pick_z",
            0.08,
            nonnegative=True,
        )
        self._bin_place_z = self._load_scalar_parameter(
            "bin_place_z",
            0.02,
            nonnegative=True,
        )
        self._base_timeout_s = self._load_scalar_parameter(
            "base_timeout_s",
            12.0,
            nonnegative=True,
        )

        # The base driver owns the /cmd_vel publisher and /odom subscription.
        # It is constructed with the same reentrant group as the other
        # blocking worker-thread adapters so odometry keeps being serviced.
        self._base_driver = BaseDriver(
            self,
            gains=BaseGains(),
            default_callback_group=self._callback_group,
        )

        detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        self._detections_subscription = self.create_subscription(
            Detection2DArray,
            detections_topic,
            self._detections_callback,
            10,
            callback_group=self._callback_group,
        )

        self._depth_localizer = DepthLocalizer(
            self,
            depth_topic=str(self.get_parameter("depth_topic").value),
            camera_info_topic=str(
                self.get_parameter("camera_info_topic").value
            ),
            optical_frame=str(self.get_parameter("optical_frame").value),
            base_frame=str(self.get_parameter("base_frame").value),
            callback_group=self._callback_group,
        )

        self._arm = JointTrajectoryActionClient(
            self,
            str(self.get_parameter("arm_action").value),
            list(self.get_parameter("arm_joints").value),
            self._callback_group,
            float(self.get_parameter("action_timeout_s").value),
        )
        self._gripper = JointTrajectoryActionClient(
            self,
            str(self.get_parameter("gripper_action").value),
            list(self.get_parameter("gripper_joints").value),
            self._callback_group,
            float(self.get_parameter("action_timeout_s").value),
        )

        self._grasp_service = self.create_service(
            Trigger,
            "~/grasp_next",
            self._grasp_next_callback,
            callback_group=self._callback_group,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "detections_topic",
            "/color_pick/detections",
        )
        self.declare_parameter(
            "arm_action",
            "/arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "gripper_action",
            "/gripper_controller/follow_joint_trajectory",
        )
        self.declare_parameter("action_timeout_s", 60.0)
        self.declare_parameter(
            "arm_joints",
            ["joint1", "joint2", "joint3", "joint4", "joint5"],
        )
        self.declare_parameter("gripper_joints", ["r_joint"])
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("optical_frame", "depth_cam_frame")
        self.declare_parameter(
            "depth_topic",
            "/depth_cam/depth_image",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/depth_cam/camera_info",
        )

        self.declare_parameter("bin_green", [-0.055, -0.32, 0.02])
        self.declare_parameter("bin_blue", [0.025, -0.32, 0.02])
        self.declare_parameter("bin_yellow", [0.105, -0.32, 0.02])
        self.declare_parameter("bin_red", [-0.135, -0.32, 0.02])
        self.declare_parameter(
            "grasp_order",
            ["red", "green", "blue", "yellow"],
        )

        self.declare_parameter("mobile_enabled", True)
        self.declare_parameter(
            "grasp_sweet_spot",
            list(self._DEFAULT_GRASP_SWEET_SPOT),
        )
        self.declare_parameter(
            "place_sweet_spot",
            list(self._DEFAULT_PLACE_SWEET_SPOT),
        )
        self.declare_parameter("block_pick_z", 0.08)
        self.declare_parameter("bin_place_z", 0.02)
        self.declare_parameter(
            "home_odom",
            list(self._DEFAULT_HOME_ODOM),
        )
        self.declare_parameter("base_timeout_s", 12.0)

        defaults = GraspConfig()
        self.declare_parameter(
            "approach_height",
            defaults.approach_height,
        )
        self.declare_parameter(
            "grasp_z_offset",
            defaults.grasp_z_offset,
        )
        self.declare_parameter("lift_height", defaults.lift_height)
        self.declare_parameter("place_height", defaults.place_height)
        self.declare_parameter("gripper_open", defaults.gripper_open)
        self.declare_parameter("gripper_closed", defaults.gripper_closed)
        self.declare_parameter("q_home", list(defaults.q_home))
        self.declare_parameter("auto_grasp", False)

    def _load_vector_parameter(
        self,
        name: str,
        default: tuple[float, ...],
        length: int,
    ) -> tuple[float, ...]:
        value = self.get_parameter(name).value
        try:
            vector = tuple(float(component) for component in value)
        except (TypeError, ValueError):
            vector = ()
        if len(vector) != length or not all(
            math.isfinite(component) for component in vector
        ):
            self.get_logger().warning(
                f"invalid {name} parameter; using its default"
            )
            return default
        return vector

    def _load_scalar_parameter(
        self,
        name: str,
        default: float,
        *,
        nonnegative: bool = False,
    ) -> float:
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            value = math.nan
        valid = math.isfinite(value)
        if nonnegative:
            valid = valid and value >= 0.0
        if not valid:
            self.get_logger().warning(
                f"invalid {name} parameter; using its default"
            )
            return default
        return value

    def _load_bins(self) -> dict[str, tuple[float, float, float]]:
        bins = {}
        for color in self._BIN_COLORS:
            value = self.get_parameter(f"bin_{color}").value
            try:
                coordinates = tuple(float(coordinate) for coordinate in value)
            except (TypeError, ValueError):
                coordinates = ()
            if (
                len(coordinates) != 3
                or not all(math.isfinite(value) for value in coordinates)
            ):
                self.get_logger().warning(
                    f"ignoring invalid bin_{color} parameter"
                )
                continue
            bins[color] = coordinates
        return bins

    @staticmethod
    def _coerce_grasp_order(value, logger=None) -> tuple[str, ...]:
        if isinstance(value, str):
            value = [value]
        try:
            return tuple(str(color) for color in value)
        except TypeError:
            if logger is not None:
                logger.warning(
                    "invalid grasp_order parameter; using the default order"
                )
            return GraspExecutorNode._BIN_COLORS

    def _load_grasp_order(self) -> tuple[str, ...]:
        return self._coerce_grasp_order(
            self.get_parameter("grasp_order").value,
            logger=self.get_logger(),
        )

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        for param in params:
            if param.name == "grasp_order":
                self._grasp_order = self._coerce_grasp_order(
                    param.value,
                    logger=self.get_logger(),
                )
        return SetParametersResult(successful=True)

    def _load_grasp_config(self) -> GraspConfig:
        defaults = GraspConfig()
        value = self.get_parameter("q_home").value
        try:
            q_home = tuple(float(joint) for joint in value)
        except (TypeError, ValueError):
            q_home = ()
        if (
            len(q_home) != 5
            or not all(math.isfinite(joint) for joint in q_home)
        ):
            self.get_logger().warning(
                "invalid q_home parameter; using default pick_init pose"
            )
            q_home = defaults.q_home

        return GraspConfig(
            approach_height=float(
                self.get_parameter("approach_height").value
            ),
            grasp_z_offset=float(
                self.get_parameter("grasp_z_offset").value
            ),
            lift_height=float(self.get_parameter("lift_height").value),
            place_height=float(self.get_parameter("place_height").value),
            gripper_open=float(self.get_parameter("gripper_open").value),
            gripper_closed=float(
                self.get_parameter("gripper_closed").value
            ),
            q_home=q_home,
        )

    def _allowed_colours(self) -> set[str]:
        with self._state_lock:
            done = set(self._done_colours)
        return set(self._bins).difference(done)

    def _all_colours_sorted(self) -> bool:
        with self._state_lock:
            return set(self._bins).issubset(self._done_colours)

    def _mark_colour_done(self, colour: str) -> None:
        with self._state_lock:
            self._done_colours.add(colour)

    def _detections_callback(self, msg: Detection2DArray) -> None:
        detections = []
        for detection in msg.detections:
            if not detection.results:
                continue
            detections.append(
                DetectedBlock(
                    color=detection.results[0].hypothesis.class_id,
                    u=float(detection.bbox.center.position.x),
                    v=float(detection.bbox.center.position.y),
                    area=float(detection.bbox.size_x * detection.bbox.size_y),
                )
            )

        with self._state_lock:
            self._latest_detections = detections

        if not self._auto_grasp:
            return
        if self._all_colours_sorted():
            self.get_logger().info("all colours sorted")
            return
        if rank_targets(detections, self._allowed_colours()):
            self._start_cycle()

    def _grasp_next_callback(self, _request, response):
        if not self._start_cycle():
            response.success = False
            response.message = "grasp cycle already running"
            return response

        response.success = True
        response.message = "started"
        return response

    def _start_cycle(self) -> bool:
        with self._state_lock:
            if self._cycle_running:
                return False
            self._cycle_running = True

        worker = threading.Thread(
            target=self._run_cycle,
            name="grasp-executor-cycle",
            daemon=True,
        )
        self._worker_thread = worker
        try:
            worker.start()
        except Exception:
            with self._state_lock:
                self._cycle_running = False
            raise
        return True

    def _run_cycle(self) -> None:
        try:
            with self._state_lock:
                detections = list(self._latest_detections)
            targets = rank_targets(detections, self._allowed_colours())
            if not targets:
                self.get_logger().warning("no allowed target")
                return

            if self._mobile_enabled:
                self._run_mobile_cycle(targets)
            else:
                self._run_fixed_base_cycle(targets)
        except Exception as exc:
            self.get_logger().error(f"grasp cycle failed: {exc}")
            self._attempt_recovery()
        finally:
            try:
                self._base_driver.stop()
            except Exception as exc:
                self.get_logger().warning(
                    f"base stop failed during cycle cleanup: {exc}"
                )
            with self._state_lock:
                self._cycle_running = False

    def _localize_target(self, target: DetectedBlock):
        try:
            block_xyz = self._depth_localizer.locate_pixel(
                target.u,
                target.v,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"could not localize {target.color} target: {exc}"
            )
            return None
        if block_xyz is None:
            self.get_logger().warning(
                f"could not localize {target.color} target; trying next"
            )
        return block_xyz

    def _execute_waypoints(self, waypoints) -> bool:
        """Execute waypoints and report action failures to the caller."""
        for waypoint in waypoints:
            self.get_logger().info(
                f"running grasp waypoint: {waypoint.label}"
            )
            if waypoint.joint_positions is not None:
                try:
                    arm_ok = self._arm.move(
                        list(waypoint.joint_positions),
                        waypoint.settle_time_s,
                    )
                except Exception as exc:
                    self.get_logger().error(
                        f"arm move failed at waypoint "
                        f"{waypoint.label}: {exc}"
                    )
                    return False
                if not arm_ok:
                    self.get_logger().error(
                        f"arm move failed at waypoint {waypoint.label}; "
                        "aborting grasp cycle"
                    )
                    return False

            if waypoint.gripper_position is not None:
                # When both targets are present they run sequentially.
                duration_s = min(
                    waypoint.settle_time_s,
                    self._GRIPPER_MOVE_S,
                )
                try:
                    gripper_ok = self._gripper.move(
                        [waypoint.gripper_position],
                        duration_s,
                    )
                except Exception as exc:
                    self.get_logger().error(
                        f"gripper move failed at waypoint "
                        f"{waypoint.label}: {exc}"
                    )
                    return False
                if not gripper_ok:
                    self.get_logger().error(
                        f"gripper move failed at waypoint {waypoint.label}; "
                        "aborting grasp cycle"
                    )
                    return False
        return True

    def _drive_or_recover(self, goal: Pose2D, description: str) -> bool:
        try:
            reached = self._base_driver.drive_to(
                goal.x,
                goal.y,
                goal.yaw,
                self._base_timeout_s,
            )
        except Exception as exc:
            self.get_logger().error(
                f"driving to {description} failed: {exc}"
            )
            self._attempt_recovery()
            return False
        if reached:
            return True
        self.get_logger().error(f"driving to {description} failed")
        self._attempt_recovery()
        return False

    def _select_mobile_target(
        self,
        targets: list[DetectedBlock],
    ) -> DetectedBlock | None:
        """Select one mobile target in configured order, then by area."""
        with self._state_lock:
            latest_detections = list(self._latest_detections)
            done_colours = set(self._done_colours)

        allowed_colours = set(self._bins).difference(done_colours)
        for colour in self._grasp_order:
            if colour not in allowed_colours:
                continue
            for detection in latest_detections:
                if detection.color == colour:
                    return detection

        ranked = rank_targets(latest_detections, allowed_colours)
        if not ranked:
            # ``targets`` is already the ranked snapshot used to start this
            # cycle; retain it as a defensive fallback if detections changed.
            ranked = rank_targets(targets, allowed_colours)
        return ranked[0] if ranked else None

    def _run_mobile_cycle(self, targets: list[DetectedBlock]) -> None:
        target = self._select_mobile_target(targets)
        if target is None:
            self.get_logger().warning("no allowed mobile target")
            return

        for target in (target,):
            block_base = self._localize_target(target)
            if block_base is None:
                continue

            odom = self._base_driver.current_pose()
            if odom is None:
                self.get_logger().warning(
                    "no odometry yet; cannot drive to block"
                )
                return

            goal = base_goal_to_center_target(
                block_base[:2],
                odom,
                self._grasp_sweet_spot,
                goal_yaw=odom.yaw,
            )
            goal = Pose2D(goal.x, odom.y, odom.yaw)
            self.get_logger().info(
                f"block {target.color}: "
                f"base=({block_base[0]:.3f},{block_base[1]:.3f},"
                f"{block_base[2]:.3f}) "
                f"odom=({odom.x:.3f},{odom.y:.3f},{odom.yaw:.3f}) "
                f"goal=({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f})"
            )
            self.get_logger().info(f"driving to block {target.color}")
            if not self._drive_or_recover(goal, f"block {target.color}"):
                return

            self.get_logger().info(
                f"reached block {target.color}, grasping"
            )
            pick = plan_pick(
                (
                    self._grasp_sweet_spot[0],
                    self._grasp_sweet_spot[1],
                    self._block_pick_z,
                ),
                self._grasp_config,
            )
            if pick is None:
                self.get_logger().error(
                    "block not reachable after centering"
                )
                self._attempt_recovery()
                return
            lift_joints = pick[-1].joint_positions
            if not self._execute_waypoints(pick):
                self._attempt_recovery()
                return

            # For this short demo the bins are fixed points in the odom
            # frame: odom is initialized at the world origin at spawn, so
            # world and odom coordinates are treated as equivalent.
            bin_world = self._bins[target.color]
            goal_yaw = self._home_odom[2]
            cos_goal_yaw = math.cos(goal_yaw)
            sin_goal_yaw = math.sin(goal_yaw)
            place_x, place_y = self._place_sweet_spot
            goal_bin = Pose2D(
                bin_world[0] - (
                    place_x * cos_goal_yaw - place_y * sin_goal_yaw
                ),
                bin_world[1] - (
                    place_x * sin_goal_yaw + place_y * cos_goal_yaw
                ),
                goal_yaw,
            )
            self.get_logger().info(f"driving to bin {target.color}")
            if not self._drive_or_recover(goal_bin, f"bin {target.color}"):
                return

            place = plan_place(
                (
                    self._place_sweet_spot[0],
                    self._place_sweet_spot[1],
                    self._bin_place_z,
                ),
                self._grasp_config,
                q_seed=lift_joints,
            )
            if place is None:
                self.get_logger().error(
                    f"bin {target.color} not reachable after centering"
                )
                self._attempt_recovery()
                return
            if not self._execute_waypoints(place):
                self._attempt_recovery()
                return

            self._mark_colour_done(target.color)
            self.get_logger().info(f"sorted {target.color}")
            try:
                home_reached = self._base_driver.drive_to(
                    *self._home_odom,
                    self._base_timeout_s,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"failed to return home after {target.color}: {exc}"
                )
            else:
                if not home_reached:
                    self.get_logger().warning(
                        f"failed to return home after {target.color}"
                    )
            return

        self.get_logger().warning("no localizable target")

    def _run_fixed_base_cycle(self, targets: list[DetectedBlock]) -> None:
        """Retain the original fixed-base execution path."""
        for target in targets:
            block_xyz = self._localize_target(target)
            if block_xyz is None:
                continue

            try:
                plan = plan_pick_and_place(
                    block_xyz,
                    self._bins[target.color],
                    self._grasp_config,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"failed to plan {target.color} target: {exc}"
                )
                continue
            if plan is None:
                self.get_logger().warning(
                    f"no reachable plan for {target.color} at "
                    f"{block_xyz}; trying next"
                )
                continue
            if not self._execute_waypoints(plan):
                self._attempt_recovery()
                return

            self._mark_colour_done(target.color)
            self.get_logger().info(f"sorted {target.color}")
            self.get_logger().info(
                f"grasp cycle complete for {target.color}"
            )
            return

        self.get_logger().warning("no reachable target")

    def _attempt_recovery(self) -> None:
        """Best-effort stop, release, and return to home."""
        self.get_logger().warning("attempting recovery to home")
        try:
            self._base_driver.stop()
        except Exception as exc:
            self.get_logger().warning(f"recovery base stop failed: {exc}")
        try:
            self._gripper.move(
                [self._grasp_config.gripper_open],
                self._GRIPPER_MOVE_S,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"recovery gripper move failed: {exc}"
            )
        try:
            self._arm.move(
                list(self._grasp_config.q_home),
                self._grasp_config.move_settle_s,
            )
        except Exception as exc:
            self.get_logger().warning(f"recovery arm move failed: {exc}")

    def shutdown_base(self) -> None:
        """Stop the base before the node and its executor are destroyed."""
        self._base_driver.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_base()
        except Exception as exc:
            node.get_logger().warning(
                f"base stop failed during node shutdown: {exc}"
            )
        worker = node._worker_thread
        if worker is not None:
            try:
                worker.join(timeout=2.0)
            except RuntimeError as exc:
                node.get_logger().warning(
                    f"grasp worker join failed during shutdown: {exc}"
                )
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
