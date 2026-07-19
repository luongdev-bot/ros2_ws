import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET


class DefaultJointStatePublisher(Node):
    """Publishes all-zero JointState for every movable joint in robot_description.

    Stand-in for joint_state_publisher_gui when that package isn't installed.
    """

    def __init__(self):
        super().__init__(
            'default_joint_state_publisher',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._joint_names = []
        self._timer = self.create_timer(1.0, self._try_init)
        self._publish_timer = None

    def _try_init(self):
        param = self.get_parameter_or('robot_description', None)
        if param is None or not param.value:
            self.get_logger().info('waiting for robot_description parameter...')
            return
        root = ET.fromstring(param.value)
        self._joint_names = [
            j.get('name') for j in root.findall('joint')
            if j.get('type') != 'fixed'
        ]
        self.get_logger().info(f'publishing {len(self._joint_names)} joints at 0.0')
        self._timer.cancel()
        self._publish_timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [0.0] * len(self._joint_names)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = DefaultJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
