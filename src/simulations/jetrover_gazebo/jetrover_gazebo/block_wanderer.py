import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class BlockWanderer(Node):
    def __init__(self) -> None:
        super().__init__('block_wanderer')

        self.declare_parameter(
            'block_names',
            ['block_red', 'block_green', 'block_blue', 'block_yellow'],
        )
        self.declare_parameter('linear_speed_mps', 0.05)
        self.declare_parameter('change_interval_s', 4.0)

        block_names = self.get_parameter('block_names').value
        self._linear_speed_mps = self.get_parameter('linear_speed_mps').value
        change_interval_s = self.get_parameter('change_interval_s').value

        self._publishers = {
            name: self.create_publisher(Twist, f'/model/{name}/cmd_vel', 10)
            for name in block_names
        }
        self._timer = self.create_timer(change_interval_s, self._on_timer)

    def _on_timer(self) -> None:
        for publisher in self._publishers.values():
            message = Twist()
            message.linear.x = random.uniform(
                -self._linear_speed_mps,
                self._linear_speed_mps,
            )
            message.linear.y = random.uniform(
                -self._linear_speed_mps,
                self._linear_speed_mps,
            )
            message.angular.z = 0.0
            publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
