"""Closed-loop ROS 2 pick-and-place executor for colour detections."""

import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from ...application.grasp_plan import GraspConfig, plan_pick_and_place
from ...application.target_selection import DetectedBlock, rank_targets
from ..depth_localizer import DepthLocalizer
from .trajectory_client import JointTrajectoryActionClient


class GraspExecutorNode(Node):
    """Localize ranked detections and execute the first viable grasp plan."""

    _BIN_COLORS = ("red", "green", "blue", "yellow")
    _GRIPPER_MOVE_S = 1.0

    def __init__(self) -> None:
        super().__init__("grasp_executor")
        self._declare_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._latest_detections: list[DetectedBlock] = []
        self._cycle_running = False
        self._worker_thread = None

        self._bins = self._load_bins()
        self._grasp_config = self._load_grasp_config()
        self._auto_grasp = bool(self.get_parameter("auto_grasp").value)

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

        if (
            self._auto_grasp
            and rank_targets(detections, set(self._bins))
        ):
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

            targets = rank_targets(detections, set(self._bins))
            for target in targets:
                try:
                    block_xyz = self._depth_localizer.locate_pixel(
                        target.u,
                        target.v,
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"could not localize {target.color} target: {exc}"
                    )
                    continue
                if block_xyz is None:
                    self.get_logger().warning(
                        f"could not localize {target.color} target; "
                        "trying next"
                    )
                    continue

                bin_xyz = self._bins[target.color]
                try:
                    plan = plan_pick_and_place(
                        block_xyz,
                        bin_xyz,
                        self._grasp_config,
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"failed to plan {target.color} target: {exc}"
                    )
                    continue
                if plan is None:
                    self.get_logger().warning(
                        (
                            f"no reachable plan for {target.color} at "
                            f"{block_xyz}; trying next"
                        )
                    )
                    continue

                for waypoint in plan:
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
                            self._attempt_recovery()
                            return
                        if not arm_ok:
                            self.get_logger().error(
                                (
                                    f"arm move failed at waypoint "
                                    f"{waypoint.label}; aborting grasp cycle"
                                )
                            )
                            self._attempt_recovery()
                            return

                    if waypoint.gripper_position is not None:
                        # When both targets are present they run sequentially
                        # in this first version.
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
                            self._attempt_recovery()
                            return
                        if not gripper_ok:
                            self.get_logger().error(
                                (
                                    f"gripper move failed at waypoint "
                                    f"{waypoint.label}; aborting grasp cycle"
                                )
                            )
                            self._attempt_recovery()
                            return

                self.get_logger().info(
                    f"grasp cycle complete for {target.color}"
                )
                return

            self.get_logger().warning("no reachable target")
        except Exception as exc:
            self.get_logger().error(f"grasp cycle failed: {exc}")
        finally:
            with self._state_lock:
                self._cycle_running = False

    def _attempt_recovery(self) -> None:
        """Best-effort release and return to the configured home pose."""
        self.get_logger().warning("attempting recovery to home")
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
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
