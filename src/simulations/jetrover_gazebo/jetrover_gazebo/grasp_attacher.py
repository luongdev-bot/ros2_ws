"""Simulation-only colour-block attacher for the JetRover gripper."""

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from jetrover_gazebo.grasp_logic import (
    GraspAction,
    GraspController,
    Pose,
    TimeRollbackDetector,
    compose_poses,
    is_valid_pose,
    timer_period_from_rate,
)


class GraspAttacher(Node):
    """Carry one nearby Gazebo block while ``r_joint`` is closed."""

    def __init__(self) -> None:
        super().__init__('grasp_attacher')

        # joint_name and the open/closed defaults below intentionally mirror
        # the r_joint detents in src/arm_motion/config/jetrover_arm.yaml. If
        # that profile changes, override all three parameters via launch.
        self.declare_parameter('joint_name', 'r_joint')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'block_pose_topic',
            '/world/color_blocks_world/dynamic_pose/info',
        )
        self.declare_parameter(
            'set_pose_service',
            '/world/color_blocks_world/set_pose',
        )
        self.declare_parameter(
            'entity_command_topic',
            '/grasp/entity_command',
        )
        self.declare_parameter('robot_model_name', 'jetrover')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('gripper_tip_frame', 'end_effector_link')
        self.declare_parameter(
            'block_names',
            ['block_red', 'block_green', 'block_blue', 'block_yellow'],
        )
        self.declare_parameter('block_name_prefix', 'block_')
        self.declare_parameter('open_position', -1.00)
        self.declare_parameter('closed_position', 0.30)
        self.declare_parameter('closed_position_tolerance', 0.05)
        self.declare_parameter('open_position_tolerance', 0.05)
        self.declare_parameter('grasp_radius', 0.045)
        self.declare_parameter('update_rate', 50.0)

        self._joint_name = self.get_parameter('joint_name').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        block_pose_topic = self.get_parameter('block_pose_topic').value
        set_pose_service = self.get_parameter('set_pose_service').value
        entity_command_topic = self.get_parameter(
            'entity_command_topic').value
        self._robot_model_name = self.get_parameter('robot_model_name').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._gripper_tip_frame = self.get_parameter('gripper_tip_frame').value
        self._block_names = set(self.get_parameter('block_names').value)
        self._block_name_prefix = self.get_parameter(
            'block_name_prefix').value
        update_rate = float(self.get_parameter('update_rate').value)

        if not self._block_names and not self._block_name_prefix:
            raise ValueError(
                'Either block_names or block_name_prefix must be set')
        timer_period = timer_period_from_rate(update_rate)

        self._controller = GraspController(
            closed_position=float(self.get_parameter('closed_position').value),
            open_position=float(self.get_parameter('open_position').value),
            closed_tolerance=float(
                self.get_parameter('closed_position_tolerance').value),
            open_tolerance=float(
                self.get_parameter('open_position_tolerance').value),
            grasp_radius=float(self.get_parameter('grasp_radius').value),
        )
        self._joint_position: Optional[float] = None
        self._robot_pose: Optional[Pose] = None
        self._block_poses: Dict[str, Pose] = {}
        self._pose_future = None
        self._service_failure_reported = False
        self._entity_command = None
        self._invalid_pose_names = set()
        self._time_rollback_detector = TimeRollbackDetector()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._set_pose_client = self.create_client(
            SetEntityPose, set_pose_service)
        self._entity_command_publisher = self.create_publisher(
            Entity, entity_command_topic, 10)
        self._joint_subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._pose_subscription = self.create_subscription(
            TFMessage,
            block_pose_topic,
            self._on_gazebo_poses,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(timer_period, self._update)

        self.get_logger().info(
            f'Grasp attacher ready at {update_rate:.1f} Hz; '
            f'tip={self._gripper_tip_frame}, blocks={block_pose_topic}')

    def _on_joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index(self._joint_name)
        except ValueError:
            return
        if index < len(message.position):
            self._joint_position = message.position[index]

    def _on_gazebo_poses(self, message: TFMessage) -> None:
        for transform in message.transforms:
            name = transform.child_frame_id.lstrip('/')
            is_robot = name == self._robot_model_name
            is_block = self._is_configured_block(name)
            if not is_robot and not is_block:
                continue
            pose = self._pose_from_transform(transform.transform)
            if not is_valid_pose(pose):
                self._warn_invalid_pose(name)
                if is_robot:
                    self._robot_pose = None
                if is_block:
                    self._block_poses.pop(name, None)
                continue
            self._invalid_pose_names.discard(name)
            if is_robot:
                self._robot_pose = pose
            if is_block:
                self._block_poses[name] = pose

    def _is_configured_block(self, name: str) -> bool:
        if self._block_names:
            return name in self._block_names
        return name.startswith(self._block_name_prefix) and '::' not in name

    def _warn_invalid_pose(self, name: str) -> None:
        if name in self._invalid_pose_names:
            return
        self.get_logger().warning(
            f'Ignoring malformed pose for {name}: values must be finite and '
            'the quaternion must have a finite, nonzero norm')
        self._invalid_pose_names.add(name)

    @staticmethod
    def _pose_from_transform(transform) -> Pose:
        return Pose(
            position=(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            orientation=(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _gripper_pose_in_world(self) -> Optional[Pose]:
        if self._robot_pose is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_base_frame,
                self._gripper_tip_frame,
                Time(),
            )
        except TransformException:
            return None
        tip_in_robot = self._pose_from_transform(transform.transform)
        transform_name = (
            f'{self._robot_base_frame}->{self._gripper_tip_frame}')
        if not is_valid_pose(tip_in_robot):
            self._warn_invalid_pose(transform_name)
            return None
        try:
            gripper_pose = compose_poses(self._robot_pose, tip_in_robot)
        except ValueError:
            self._warn_invalid_pose(transform_name)
            return None
        if not is_valid_pose(gripper_pose):
            self._warn_invalid_pose(transform_name)
            return None
        self._invalid_pose_names.discard(transform_name)
        return gripper_pose

    def _update(self) -> None:
        current_time = self.get_clock().now().nanoseconds
        if self._time_rollback_detector.observe(current_time):
            self._entity_command = None
            self._controller.clear_held_state()
            self._reset_tf_state()
            self._joint_position = None
            self._robot_pose = None
            self._block_poses = {}
            # Accepted reset limits: a queued MODEL heartbeat or in-flight
            # SetEntityPose may land once; timeout/reopen self-corrects.
            self._pose_future = None
            self.get_logger().warning(
                'Cleared grasp state after simulation time moved backwards')
            return

        self._finish_pose_request()
        self._publish_entity_command()
        if self._joint_position is None:
            return
        self._controller.observe_joint_position(self._joint_position)
        if self._controller.held_block_name is None:
            if not self._controller.is_closed(self._joint_position):
                return
            if not self._block_poses:
                return

        gripper_pose = self._gripper_pose_in_world()
        if gripper_pose is None:
            return
        decision = self._controller.update(
            self._joint_position,
            gripper_pose,
            self._block_poses,
        )

        if decision.action is GraspAction.GRASP:
            self._entity_command = (decision.block_name, True)
            self._publish_entity_command()
            self.get_logger().info(f'Attached {decision.block_name}')
        elif decision.action is GraspAction.RELEASE:
            self._entity_command = (decision.block_name, False)
            self._publish_entity_command()
            self.get_logger().info(f'Released {decision.block_name}')
            return

        if decision.target_pose is None or decision.block_name is None:
            return
        if (
            self._pose_future is not None
            or not self._set_pose_client.service_is_ready()
        ):
            return

        request = SetEntityPose.Request()
        request.entity.name = decision.block_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = decision.target_pose.position[0]
        request.pose.position.y = decision.target_pose.position[1]
        request.pose.position.z = decision.target_pose.position[2]
        request.pose.orientation.x = decision.target_pose.orientation[0]
        request.pose.orientation.y = decision.target_pose.orientation[1]
        request.pose.orientation.z = decision.target_pose.orientation[2]
        request.pose.orientation.w = decision.target_pose.orientation[3]
        self._pose_future = self._set_pose_client.call_async(request)

    def _reset_tf_state(self) -> None:
        """Replace TF state after a simulation clock reset."""

        self._tf_listener.unregister()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _publish_entity_command(self) -> None:
        if self._entity_command is None:
            return
        block_name, held = self._entity_command
        message = Entity()
        message.name = block_name
        message.type = Entity.MODEL if held else Entity.NONE
        self._entity_command_publisher.publish(message)

    def release_held_block(self) -> None:
        """Tell Gazebo to release the held block before this node exits."""
        block_name = self._controller.held_block_name
        if block_name is None:
            return
        self._entity_command = (block_name, False)
        self._publish_entity_command()

    def _finish_pose_request(self) -> None:
        if self._pose_future is None or not self._pose_future.done():
            return

        # Service futures surface Gazebo transport errors at result access.
        try:
            response = self._pose_future.result()
            if response is None or not response.success:
                raise RuntimeError('Gazebo rejected the pose request')
            self._service_failure_reported = False
        except Exception as error:
            if not self._service_failure_reported:
                self.get_logger().error(f'Failed to hold block: {error}')
                self._service_failure_reported = True
        finally:
            self._pose_future = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspAttacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.release_held_block()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
