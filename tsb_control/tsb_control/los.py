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

"""Line-of-Sight (LOS) path guidance in the plane (no ROS dependency)."""

import math


def wrap_angle(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LOSGuidance:
    """
    Lookahead-based LOS guidance along a piecewise-linear path.

    For the active segment A -> B, with path angle ``alpha``, the vehicle
    position is expressed in the path frame:

        s = along-track distance from A
        e = cross-track error (positive to the left of the path)

    and the desired heading is::

        psi_d = alpha + atan2(-e, lookahead)

    i.e. the vehicle aims at a point ``lookahead`` metres ahead on the path,
    which drives the cross-track error to zero. The desired surge speed is::

        v_d = cruise_speed * max(0, cos(psi_d - psi))   (turn first, then go)
        v_d = min(v_d, slow_gain * distance_to_goal)    (last segment only)

    Waypoint switching: the next segment becomes active when the vehicle is
    inside the acceptance radius of B, or has passed ``L - acceptance_radius``
    along the segment. The path ends when the vehicle is inside the
    acceptance radius of the last waypoint (speed 0, heading kept along the
    last segment). On the last segment, if the goal is closer than the
    lookahead, the vehicle steers directly towards it, which guarantees
    convergence even after overshooting.
    """

    def __init__(self, lookahead=0.5, acceptance_radius=0.15,
                 cruise_speed=0.15, max_speed=0.22, slow_gain=1.0):
        """Store the guidance parameters."""
        self.lookahead = lookahead
        self.acceptance_radius = acceptance_radius
        self.cruise_speed = cruise_speed
        self.max_speed = max_speed
        self.slow_gain = slow_gain

        self.points = []
        self.index = 0
        self.finished = True
        self.cross_track = 0.0
        self.hold_heading = 0.0

    def set_path(self, points):
        """Set a new path (list of ``(x, y)``), dropping repeated points."""
        cleaned = []
        for px, py in points:
            if cleaned:
                gap = math.hypot(px - cleaned[-1][0], py - cleaned[-1][1])
                if gap <= 1e-6:
                    continue
            cleaned.append((float(px), float(py)))
        self.points = cleaned
        self.index = 0
        self.cross_track = 0.0
        self.finished = len(cleaned) < 2
        if not self.finished:
            (ax, ay), (bx, by) = cleaned[0], cleaned[1]
            self.hold_heading = math.atan2(by - ay, bx - ax)

    def update(self, x, y, heading):
        """
        Return ``(desired_speed, desired_heading, finished)``.

        ``heading`` is the measured heading [rad]. Once finished, the speed
        is 0 and the desired heading is kept constant.
        """
        if self.finished:
            return 0.0, self.hold_heading, True

        last_index = len(self.points) - 2
        radius = self.acceptance_radius
        while True:
            ax, ay = self.points[self.index]
            bx, by = self.points[self.index + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            alpha = math.atan2(by - ay, bx - ax)
            rx, ry = x - ax, y - ay
            along = rx * math.cos(alpha) + ry * math.sin(alpha)
            cross = -rx * math.sin(alpha) + ry * math.cos(alpha)
            dist_b = math.hypot(x - bx, y - by)
            is_last = self.index == last_index

            if is_last:
                if dist_b < radius:
                    self.finished = True
                    self.hold_heading = alpha
                    return 0.0, alpha, True
                break
            if dist_b < radius or along >= seg_len - radius:
                self.index += 1
                continue
            break

        self.cross_track = cross
        if is_last and dist_b < self.lookahead:
            desired = math.atan2(by - y, bx - x)
        else:
            desired = wrap_angle(alpha + math.atan2(-cross, self.lookahead))
        self.hold_heading = alpha

        error = wrap_angle(desired - heading)
        speed = self.cruise_speed * max(0.0, math.cos(error))
        if is_last:
            speed = min(speed, self.slow_gain * dist_b)
        speed = max(0.0, min(self.max_speed, speed))
        return speed, desired, False
