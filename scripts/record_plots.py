#!/usr/bin/env python3
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

"""Record the validation plots of the control pipeline in Gazebo.

Needs the simulator and ``ros2 launch tsb_control control.launch.py``
running, the workspace sourced and ``python3-matplotlib`` installed.

    python3 scripts/record_plots.py --experiment both --out docs

Experiments (the robot moves: put it in a free area first):
  * step: surge step to ``--speed`` and heading step of ``--turn-deg``
    degrees for ``--step-duration`` s, then stop (3 s).
    -> step_response.png
  * los: follow ``--waypoints`` with LOS, starting from the current pose.
    Coordinates are in the ``odom`` frame, which in the TurtleBot3 Gazebo
    simulation is the world frame. The default is a 1.4 m x 1.0 m rectangle
    in the free area of ``turtlebot3_world``.
    -> los_trajectory.png
"""

import argparse
import math
import os
import time

from geometry_msgs.msg import Point, PoseStamped, Vector3
import matplotlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

matplotlib.use('Agg')  # no display needed (Docker)


def wrap(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def step_metrics(times, values, final):
    """Return ``(rise_time_10_90, overshoot_percent)`` of a step response."""
    if abs(final) < 1e-9 or not times:
        return None, None
    sign = 1.0 if final > 0 else -1.0
    norm = [sign * v / abs(final) for v in values]
    t10 = next((t for t, v in zip(times, norm) if v >= 0.1), None)
    t90 = next((t for t, v in zip(times, norm) if v >= 0.9), None)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    overshoot = max(0.0, (max(norm) - 1.0) * 100.0)
    return rise, overshoot


def segment_distance(px, py, a, b):
    """Distance from point (px, py) to the segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return math.hypot(px - a[0], py - a[1])
    t = ((px - a[0]) * dx + (py - a[1]) * dy) / length2
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def polyline_distance(px, py, points):
    """Distance from a point to a polyline (list of ``(x, y)``)."""
    return min(segment_distance(px, py, points[i], points[i + 1])
               for i in range(len(points) - 1))


def parse_waypoints(text):
    """Parse ``"x1,y1;x2,y2;..."`` into a list of ``(x, y)`` floats."""
    points = []
    for item in text.split(';'):
        item = item.strip()
        if item:
            x_str, y_str = item.split(',')
            points.append((float(x_str), float(y_str)))
    if not points:
        raise ValueError('at least one waypoint is needed')
    return points


def fmt(value, unit):
    """Format an optional metric for a plot title."""
    return 'n/a' if value is None else f'{value:.2f} {unit}'


def plot_step(samples, turn_deg, out_file):
    """Plot surge and heading step responses (``samples`` from Recorder)."""
    t0 = samples[0][0]
    t = [s[0] - t0 for s in samples]
    surge = [s[1] for s in samples]
    sp_surge = [s[3] for s in samples]
    h0 = samples[0][2]
    heading = [math.degrees(wrap(s[2] - h0)) for s in samples]
    sp_heading = [math.degrees(wrap(s[4] - h0)) for s in samples]

    final_surge = max(sp_surge)
    rise_s, over_s = step_metrics(t, surge, final_surge)
    rise_h, over_h = step_metrics(t, heading, turn_deg)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax1.plot(t, sp_surge, 'k--', label='setpoint')
    ax1.plot(t, surge, 'b', label='measured')
    ax1.set_ylabel('surge [m/s]')
    ax1.set_title(f'Surge: rise {fmt(rise_s, "s")}, '
                  f'overshoot {fmt(over_s, "%")}')
    ax2.plot(t, sp_heading, 'k--', label='setpoint')
    ax2.plot(t, heading, 'b', label='measured')
    ax2.set_ylabel('heading change [deg]')
    ax2.set_xlabel('time [s]')
    ax2.set_title(f'Heading: rise {fmt(rise_h, "s")}, '
                  f'overshoot {fmt(over_h, "%")}')
    for ax in (ax1, ax2):
        ax.grid(True)
        ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return rise_s, over_s, rise_h, over_h


def plot_los(trajectory, planned, out_file):
    """Plot the planned polyline and the trajectory; return (max, mean) err."""
    errors = [polyline_distance(x, y, planned) for x, y in trajectory]
    max_err = max(errors)
    mean_err = sum(errors) / len(errors)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot([p[0] for p in planned], [p[1] for p in planned], 'k--o',
            label='planned path')
    for i, (px, py) in enumerate(planned):
        ax.annotate('start' if i == 0 else f'WP{i}', (px, py),
                    textcoords='offset points', xytext=(6, 6))
    ax.plot([p[0] for p in trajectory], [p[1] for p in trajectory], 'b',
            label='trajectory')
    ax.plot(*trajectory[0], 'go', label='start')
    ax.plot(*trajectory[-1], 'rs', label='end')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m] (odom)')
    ax.set_ylabel('y [m] (odom)')
    ax.set_title(f'LOS: max cross-track {max_err:.2f} m, '
                 f'mean {mean_err:.2f} m')
    ax.grid(True)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return max_err, mean_err


class Recorder(Node):
    """Run the experiments and record /state, /position, /target_setpoint."""

    def __init__(self, use_sim_time=True):
        """Create subscriptions and publishers."""
        super().__init__(
            'record_plots',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
            ])
        self.surge = None
        self.heading = None
        self.position = None
        self.setpoint = (0.0, 0.0)
        self.recording = False
        self.samples = []       # (t, surge, heading, sp_surge, sp_heading)
        self.trajectory = []    # (x, y)

        self.create_subscription(Vector3, '/state', self.on_state, 10)
        self.create_subscription(Point, '/position', self.on_position, 10)
        self.create_subscription(
            Vector3, '/target_setpoint', self.on_setpoint, 10)
        self.setpoint_pub = self.create_publisher(
            Vector3, '/target_setpoint', 10)
        self.path_pub = self.create_publisher(Path, '/waypoints', 10)

    def now(self):
        """Return the node clock time in seconds (simulation time)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        """Store the state and record a sample."""
        self.surge, self.heading = msg.x, msg.y
        if self.recording:
            self.samples.append((self.now(), msg.x, msg.y,
                                 self.setpoint[0], self.setpoint[1]))

    def on_position(self, msg):
        """Store the position and record the trajectory."""
        self.position = (msg.x, msg.y)
        if self.recording:
            self.trajectory.append(self.position)

    def on_setpoint(self, msg):
        """Remember the last setpoint seen on the bus."""
        self.setpoint = (msg.x, msg.y)

    def spin_for(self, seconds):
        """Spin for ``seconds`` of node time (with a wall-time guard)."""
        end = self.now() + seconds
        guard = time.monotonic() + max(10.0, 20.0 * seconds)
        while rclpy.ok() and self.now() < end:
            if time.monotonic() > guard:
                self.get_logger().warn('Clock is not advancing (/clock?).')
                break
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_data(self, timeout=15.0):
        """Wait for the first /state and /position messages."""
        guard = time.monotonic() + timeout
        while rclpy.ok() and (self.heading is None or self.position is None):
            if time.monotonic() > guard:
                raise RuntimeError(
                    'No /state or /position received: are the simulator and '
                    'control.launch.py running?')
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_setpoint(self, speed, heading):
        """Publish a (surge, heading) setpoint."""
        msg = Vector3()
        msg.x, msg.y, msg.z = float(speed), float(heading), 0.0
        self.setpoint_pub.publish(msg)
        self.setpoint = (msg.x, msg.y)

    def run_step(self, speed, turn_deg, duration, settle=3.0):
        """Apply a surge and heading step, then stop. Return the samples."""
        self.wait_for_data()
        target = wrap(self.heading + math.radians(turn_deg))
        self.setpoint = (0.0, self.heading)
        self.samples = []
        self.recording = True
        for _ in range(int(duration * 10)):
            self.publish_setpoint(speed, target)
            self.spin_for(0.1)
        for _ in range(int(settle * 10)):
            self.publish_setpoint(0.0, target)
            self.spin_for(0.1)
        self.recording = False
        return list(self.samples)

    def cancel_path(self):
        """Cancel any path the follower may still be running."""
        path = Path()
        path.header.frame_id = 'odom'
        self.path_pub.publish(path)
        self.spin_for(0.3)

    def run_los(self, waypoints, timeout):
        """Follow the waypoints. Return (trajectory, planned)."""
        self.wait_for_data()
        x0, y0 = self.position
        planned = [(x0, y0)] + list(waypoints)
        last = waypoints[-1]

        guard = time.monotonic() + 10.0
        while self.path_pub.get_subscription_count() == 0:
            if time.monotonic() > guard:
                raise RuntimeError('path_follower_node is not running.')
            rclpy.spin_once(self, timeout_sec=0.1)

        path = Path()
        path.header.frame_id = 'odom'
        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.trajectory = []
        self.recording = True
        self.path_pub.publish(path)
        start = self.now()
        left_start = False
        while rclpy.ok() and self.now() - start < timeout:
            self.spin_for(0.1)
            left_start = left_start or math.hypot(
                self.position[0] - x0, self.position[1] - y0) > 0.3
            dist_last = math.hypot(self.position[0] - last[0],
                                   self.position[1] - last[1])
            if left_start and dist_last < 0.2:
                break
        else:
            self.get_logger().warn('Timeout before the path was completed.')
        self.spin_for(2.0)
        self.recording = False
        return list(self.trajectory), planned

    def stop(self):
        """Command zero speed holding the current heading."""
        if self.heading is not None:
            self.publish_setpoint(0.0, self.heading)


def main():
    """Parse the arguments, run the experiments and save the plots."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--experiment', choices=['step', 'los', 'both'],
                        default='both')
    parser.add_argument('--out', default='docs', help='output directory')
    parser.add_argument('--speed', type=float, default=0.15)
    parser.add_argument('--turn-deg', type=float, default=90.0)
    parser.add_argument('--step-duration', type=float, default=4.0)
    parser.add_argument(
        '--waypoints', default='-0.55,0.5;-0.55,-0.5;-1.95,-0.5;-1.95,0.5',
        help='"x,y;x,y;..." in the odom frame (default: free area of '
             'turtlebot3_world)')
    parser.add_argument('--los-timeout', type=float, default=120.0)
    parser.add_argument('--wall-time', action='store_true',
                        help='do not use simulation time (real robot)')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = Recorder(use_sim_time=not args.wall_time)
    try:
        if args.experiment in ('step', 'both'):
            node.get_logger().info('Running the step experiment...')
            node.cancel_path()
            samples = node.run_step(args.speed, args.turn_deg,
                                    args.step_duration)
            out = os.path.join(args.out, 'step_response.png')
            rise_s, over_s, rise_h, over_h = plot_step(
                samples, args.turn_deg, out)
            node.get_logger().info(
                f'Saved {out} (surge rise {fmt(rise_s, "s")}, overshoot '
                f'{fmt(over_s, "%")}; heading rise {fmt(rise_h, "s")}, '
                f'overshoot {fmt(over_h, "%")})')
        if args.experiment in ('los', 'both'):
            node.get_logger().info('Running the LOS experiment...')
            trajectory, planned = node.run_los(
                parse_waypoints(args.waypoints), args.los_timeout)
            out = os.path.join(args.out, 'los_trajectory.png')
            max_err, mean_err = plot_los(trajectory, planned, out)
            node.get_logger().info(
                f'Saved {out} (cross-track max {max_err:.2f} m, '
                f'mean {mean_err:.2f} m)')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
