# Copyright 2026 Laura Martinho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Control allocation node mapping (/tau_u, /tau_r) to thruster commands."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AllocationNode(Node):
    """Allocates surge and yaw efforts to left/right actuators and /cmd_vel."""

    def __init__(self):
        """Declare parameters and initialize subscribers and publishers."""
        super().__init__('allocation_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_separation', 0.16),
                ('max_wheel_speed', 0.22),
                ('max_current', 10.0),
                ('cmd_timeout', 0.5),
            ],
        )

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.max_current = self.get_parameter('max_current').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.tau_u = 0.0
        self.tau_r = 0.0
        self.last_cmd_time = None

        self.tau_u_sub = self.create_subscription(
            Float64, '/tau_u', self.tau_u_callback, 10)
        self.tau_r_sub = self.create_subscription(
            Float64, '/tau_r', self.tau_r_callback, 10)

        self.left_pub = self.create_publisher(Float64, '/current_left', 10)
        self.right_pub = self.create_publisher(Float64, '/current_right', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.publish_commands)
        self.get_logger().info('Allocation Node started.')

    def tau_u_callback(self, msg):
        """Store latest surge control effort and update timestamp."""
        self.tau_u = float(msg.data)
        self.last_cmd_time = self.get_clock().now()

    def tau_r_callback(self, msg):
        """Store latest yaw control effort and update timestamp."""
        self.tau_r = float(msg.data)
        self.last_cmd_time = self.get_clock().now()

    def publish_commands(self):
        """Allocate left/right commands and publish the TurtleBot3 Twist."""
        if self.last_cmd_time is not None and self.cmd_timeout > 0.0:
            elapsed = (
                self.get_clock().now() - self.last_cmd_time
            ).nanoseconds * 1e-9
            if elapsed > self.cmd_timeout:
                self.tau_u = 0.0
                self.tau_r = 0.0

        half_l = 0.5 * self.wheel_separation

        # Differential allocation:
        # v_left = tau_u - (L/2) * tau_r
        # v_right = tau_u + (L/2) * tau_r
        v_left = self.tau_u - half_l * self.tau_r
        v_right = self.tau_u + half_l * self.tau_r

        # Saturate wheel speeds symmetrically
        max_mag = max(abs(v_left), abs(v_right))
        if max_mag > self.max_wheel_speed and max_mag > 0.0:
            scale = self.max_wheel_speed / max_mag
            v_left *= scale
            v_right *= scale

        # Publish /current_left and /current_right (scaled actuator commands)
        current_scale = self.max_current / self.max_wheel_speed
        left_msg = Float64()
        left_msg.data = float(v_left * current_scale)
        right_msg = Float64()
        right_msg.data = float(v_right * current_scale)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Convert wheel speeds to a differential-drive Twist (TurtleBot3)
        twist = Twist()
        twist.linear.x = float(0.5 * (v_right + v_left))
        twist.angular.z = float((v_right - v_left) / self.wheel_separation)
        self.cmd_pub.publish(twist)


def main(args=None):
    """Run the control allocation ROS 2 node."""
    rclpy.init(args=args)
    node = AllocationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
