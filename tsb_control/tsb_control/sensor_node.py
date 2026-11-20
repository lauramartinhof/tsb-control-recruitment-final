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

"""Sensor processing node for surge velocity and heading estimation."""

import math

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class SensorProcessingNode(Node):
    """Fuse odometry surge and IMU heading into /state; publish /position."""

    def __init__(self):
        """Initialize subscribers, state publisher, and periodic timer."""
        super().__init__('sensor_processing_node')

        self.current_surge = 0.0
        self.current_heading = 0.0
        self.position = (0.0, 0.0)
        self.has_odom = False
        self.has_imu = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        self.state_pub = self.create_publisher(Vector3, '/state', 10)
        self.position_pub = self.create_publisher(Point, '/position', 10)
        self.timer = self.create_timer(0.05, self.publish_state)
        self.get_logger().info('Sensor Processing Node started (/state).')

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion (x, y, z, w) to yaw angle in [-pi, pi]."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Extract surge velocity and fallback heading from odometry."""
        self.current_surge = msg.twist.twist.linear.x
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if not self.has_imu:
            self.current_heading = self.quaternion_to_yaw(
                msg.pose.pose.orientation)
        self.has_odom = True

    def imu_callback(self, msg):
        """Extract heading (yaw) directly from IMU orientation."""
        self.current_heading = self.quaternion_to_yaw(msg.orientation)
        self.has_imu = True

    def publish_state(self):
        """Publish surge/heading on /state and x, y on /position."""
        if not (self.has_odom or self.has_imu):
            return
        state_msg = Vector3()
        state_msg.x = float(self.current_surge)
        state_msg.y = float(self.current_heading)
        state_msg.z = 0.0
        self.state_pub.publish(state_msg)
        if self.has_odom:
            position_msg = Point()
            position_msg.x = float(self.position[0])
            position_msg.y = float(self.position[1])
            self.position_pub.publish(position_msg)


def main(args=None):
    """Run the sensor processing ROS 2 node."""
    rclpy.init(args=args)
    node = SensorProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
