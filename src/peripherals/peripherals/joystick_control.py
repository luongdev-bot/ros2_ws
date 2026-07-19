#!/usr/bin/env python3
# encoding: utf-8
# Gamepad teleop for the JetRover Gazebo simulation.
#
# The Hiwonder `joystick_control` drives the *physical* arm/servos/action-groups
# through the robot's onboard receiver (topic `ros_robot_controller/joy`) and
# cannot run in simulation. This is the sim equivalent: it subscribes to the
# standard sensor_msgs/Joy from `joy_node` and publishes geometry_msgs/Twist to
# /cmd_vel (mecanum: forward/strafe/turn). Axis/button indices are parameters so
# it works with PS4/Xbox/generic pads.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_control')

        # Axis indices (defaults match a typical Xbox/PS4 pad via joy_node).
        self.axis_linear = self.declare_parameter('axis_linear', 1).value    # left stick vertical
        self.axis_lateral = self.declare_parameter('axis_lateral', 0).value   # left stick horizontal
        self.axis_angular = self.declare_parameter('axis_angular', 3).value   # right stick horizontal
        # Optional dead-man / enable button (-1 disables the requirement).
        self.enable_button = self.declare_parameter('enable_button', -1).value

        self.max_linear = self.declare_parameter('max_linear', 0.5).value
        self.max_lateral = self.declare_parameter('max_lateral', 0.5).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value
        self.deadzone = self.declare_parameter('deadzone', 0.08).value

        # Stop the robot if /joy goes quiet (pad unplugged, joy_node died).
        # Without this the drive plugin keeps applying the last velocity.
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value

        cmd_vel_topic = self.declare_parameter('cmd_vel', '/cmd_vel').value
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.last_joy = self.get_clock().now()
        self.moving = False
        self.watchdog = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            'joystick_control ready: /joy -> %s (mecanum forward/strafe/turn)'
            % cmd_vel_topic)

    def stop(self):
        self.pub.publish(Twist())
        self.moving = False

    def _watchdog(self):
        if not self.moving:
            return
        age = (self.get_clock().now() - self.last_joy).nanoseconds / 1e9
        if age > self.joy_timeout:
            self.get_logger().warn(
                'No /joy for %.1fs - stopping the robot.' % age)
            self.stop()

    def _axis(self, axes, idx):
        if idx < 0 or idx >= len(axes):
            return 0.0
        v = axes[idx]
        return 0.0 if abs(v) < self.deadzone else v

    def joy_callback(self, joy: Joy):
        self.last_joy = self.get_clock().now()
        twist = Twist()
        if self.enable_button >= 0:
            if self.enable_button >= len(joy.buttons) or not joy.buttons[self.enable_button]:
                self.stop()  # enable not held -> stop
                return
        twist.linear.x = self._axis(joy.axes, self.axis_linear) * self.max_linear
        twist.linear.y = self._axis(joy.axes, self.axis_lateral) * self.max_lateral
        twist.angular.z = self._axis(joy.axes, self.axis_angular) * self.max_angular
        self.pub.publish(twist)
        self.moving = (twist.linear.x or twist.linear.y or twist.angular.z) != 0.0


def main():
    rclpy.init()
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped, never with the last commanded velocity.
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
