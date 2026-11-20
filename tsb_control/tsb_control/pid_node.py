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

"""Discrete PID node with feedforward and conditional anti-windup."""

import math

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PIDController:
    """Discrete PID: feedforward, derivative on measurement, anti-windup."""

    def __init__(
        self,
        kp,
        ki,
        kd,
        kff=0.0,
        integral_limit=1.0,
        output_limit=1.0,
        is_angular=False,
    ):
        """Initialize PID gains, limits, and internal state."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.is_angular = is_angular

        self.integral = 0.0
        self.prev_measurement = None

    @staticmethod
    def normalize_angle(angle):
        """Wrap an angle in radians to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute(self, setpoint, measurement, dt):
        """Compute the saturated control effort for one sampling step."""
        if dt <= 0.0:
            return 0.0

        raw_error = setpoint - measurement
        error = (
            self.normalize_angle(raw_error) if self.is_angular else raw_error
        )

        # Feedforward and proportional terms
        ff_term = self.kff * setpoint
        p_term = self.kp * error

        # Derivative on measurement to avoid setpoint derivative kick
        if self.prev_measurement is None:
            d_meas = 0.0
        else:
            diff = measurement - self.prev_measurement
            if self.is_angular:
                diff = self.normalize_angle(diff)
            d_meas = diff / dt
        self.prev_measurement = measurement
        d_term = -self.kd * d_meas

        # Conditional integration (anti-windup)
        candidate_integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt),
        )
        unsat_output = ff_term + p_term + self.ki * candidate_integral + d_term

        if abs(unsat_output) <= self.output_limit:
            self.integral = candidate_integral
        elif (unsat_output > self.output_limit and error < 0.0) or (
            unsat_output < -self.output_limit and error > 0.0
        ):
            self.integral = candidate_integral

        output = ff_term + p_term + self.ki * self.integral + d_term
        return max(-self.output_limit, min(self.output_limit, output))


class PIDControlNode(Node):
    """Computes generalized surge (/tau_u) and yaw (/tau_r) forces."""

    def __init__(self):
        """Declare parameters, create the PIDs and the ROS interfaces."""
        super().__init__('pid_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('surge.kp', 1.2),
                ('surge.ki', 0.1),
                ('surge.kd', 0.05),
                ('surge.kff', 1.0),
                ('surge.integral_limit', 1.0),
                ('surge.max_tau', 1.0),
                ('heading.kp', 1.8),
                ('heading.ki', 0.05),
                ('heading.kd', 0.15),
                ('heading.kff', 0.0),
                ('heading.integral_limit', 1.0),
                ('heading.max_tau', 1.5),
            ],
        )

        self.surge_pid = PIDController(
            kp=self.get_parameter('surge.kp').value,
            ki=self.get_parameter('surge.ki').value,
            kd=self.get_parameter('surge.kd').value,
            kff=self.get_parameter('surge.kff').value,
            integral_limit=self.get_parameter('surge.integral_limit').value,
            output_limit=self.get_parameter('surge.max_tau').value,
            is_angular=False,
        )

        self.heading_pid = PIDController(
            kp=self.get_parameter('heading.kp').value,
            ki=self.get_parameter('heading.ki').value,
            kd=self.get_parameter('heading.kd').value,
            kff=self.get_parameter('heading.kff').value,
            integral_limit=self.get_parameter('heading.integral_limit').value,
            output_limit=self.get_parameter('heading.max_tau').value,
            is_angular=True,
        )

        self.desired_surge = 0.0
        self.desired_heading = 0.0
        self.current_surge = 0.0
        self.current_heading = 0.0
        self.state_received = False
        self.target_received = False
        self.last_time = None

        self.target_sub = self.create_subscription(
            Vector3, '/target_setpoint', self.target_callback, 10)
        self.state_sub = self.create_subscription(
            Vector3, '/state', self.state_callback, 10)

        self.tau_u_pub = self.create_publisher(Float64, '/tau_u', 10)
        self.tau_r_pub = self.create_publisher(Float64, '/tau_r', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            'PID Control Node started. '
            'Waiting for /state and /target_setpoint...'
        )

    def target_callback(self, msg):
        """Update desired surge and heading setpoints."""
        self.desired_surge = float(msg.x)
        self.desired_heading = float(msg.y)
        self.target_received = True

    def state_callback(self, msg):
        """Update current surge and heading state."""
        self.current_surge = float(msg.x)
        self.current_heading = float(msg.y)
        self.state_received = True

    def publish_taus(self, tau_u, tau_r):
        """Publish generalized surge and yaw forces."""
        msg_u = Float64()
        msg_u.data = float(tau_u)
        msg_r = Float64()
        msg_r.data = float(tau_r)
        self.tau_u_pub.publish(msg_u)
        self.tau_r_pub.publish(msg_r)

    def control_loop(self):
        """Run PID control step using simulation clock dt."""
        if not (self.state_received and self.target_received):
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        tau_u = self.surge_pid.compute(
            self.desired_surge, self.current_surge, dt)
        tau_r = self.heading_pid.compute(
            self.desired_heading, self.current_heading, dt)

        self.publish_taus(tau_u, tau_r)


def main(args=None):
    """Run the PID control ROS 2 node."""
    rclpy.init(args=args)
    node = PIDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_taus(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
