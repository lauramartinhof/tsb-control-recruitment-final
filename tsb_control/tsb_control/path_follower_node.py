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

"""Path follower node: LOS guidance from waypoints to the PID setpoint."""

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from tsb_control.los import LOSGuidance


class PathFollowerNode(Node):
    """
    Follow a list of waypoints using LOS guidance.

    Subscriptions:
      * ``/waypoints`` (nav_msgs/Path): waypoints (only ``pose.position.x/y``
        are used) expressed in the ``odom`` frame, i.e. relative to the
        start pose of the robot. An empty path cancels the current one (the
        robot stops).
      * ``/position`` (geometry_msgs/Point): measured x, y (sensor node).
      * ``/state`` (geometry_msgs/Vector3): measured surge (x) and
        heading (y).

    Publications:
      * ``/target_setpoint`` (geometry_msgs/Vector3): desired surge velocity
        (x) and heading (y), consumed by ``pid_node``.

    If ``start_from_current`` is true the current position is used as the
    start of the path, so the robot first goes in a straight line to the
    first waypoint.
    """

    def __init__(self):
        """Declare parameters and create the ROS interfaces."""
        super().__init__('path_follower_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate_hz', 20.0),
                ('lookahead', 0.5),
                ('acceptance_radius', 0.15),
                ('cruise_speed', 0.15),
                ('max_speed', 0.22),
                ('slow_gain', 1.0),
                ('start_from_current', True),
            ],
        )
        self.guidance = LOSGuidance(
            lookahead=self.get_parameter('lookahead').value,
            acceptance_radius=self.get_parameter('acceptance_radius').value,
            cruise_speed=self.get_parameter('cruise_speed').value,
            max_speed=self.get_parameter('max_speed').value,
            slow_gain=self.get_parameter('slow_gain').value,
        )
        self.start_from_current = (
            self.get_parameter('start_from_current').value)

        self.position = None     # (x, y)
        self.heading = None      # [rad]
        self.active = False

        self.create_subscription(
            Path, '/waypoints', self.path_callback, 10)
        self.create_subscription(
            Point, '/position', self.position_callback, 10)
        self.create_subscription(
            Vector3, '/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(
            Vector3, '/target_setpoint', 10)

        rate_hz = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / rate_hz, self.control_loop)
        self.get_logger().info(
            'Path follower started. Waiting for /waypoints...')

    def position_callback(self, msg):
        """Store the measured position."""
        self.position = (msg.x, msg.y)

    def state_callback(self, msg):
        """Store the measured heading."""
        self.heading = msg.y

    def publish_setpoint(self, speed, heading):
        """Publish the desired surge velocity and heading for the PID."""
        msg = Vector3()
        msg.x = float(speed)
        msg.y = float(heading)
        msg.z = 0.0
        self.setpoint_pub.publish(msg)

    def path_callback(self, msg):
        """Follow a new path (an empty path cancels the current one)."""
        if self.position is None or self.heading is None:
            self.get_logger().warn('No /position or /state yet; path ignored.')
            return
        if msg.header.frame_id not in ('', 'odom'):
            self.get_logger().warn(
                f"Path frame '{msg.header.frame_id}' is not 'odom'; "
                'no transform is applied, coordinates are used as given.')

        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            self.active = False
            self.publish_setpoint(0.0, self.heading)
            self.get_logger().info('Empty path received: stopping.')
            return
        if self.start_from_current:
            points.insert(0, self.position)

        self.guidance.set_path(points)
        if self.guidance.finished:
            self.active = False
            self.get_logger().warn(
                'Path has fewer than 2 distinct points; ignored.')
            return
        self.active = True
        self.get_logger().info(f'Following a path with {len(points)} points.')

    def control_loop(self):
        """Compute and publish the LOS setpoint (called by the timer)."""
        if not self.active or self.position is None or self.heading is None:
            return
        speed, heading_d, done = self.guidance.update(
            self.position[0], self.position[1], self.heading)
        self.publish_setpoint(speed, heading_d)
        if done:
            self.active = False
            self.get_logger().info(
                'Path completed: stopping and holding heading.')


def main(args=None):
    """Run the path follower ROS 2 node."""
    rclpy.init(args=args)
    node = PathFollowerNode()
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
