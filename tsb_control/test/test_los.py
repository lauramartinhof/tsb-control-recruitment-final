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

"""Unit tests for the LOS guidance (pure Python, no ROS needed)."""

import math

import pytest
from tsb_control.los import LOSGuidance, wrap_angle


def simulate(path, start, steps=4000, dt=0.05, **kwargs):
    """Drive a kinematic unicycle (P heading control) along a path."""
    guidance = LOSGuidance(**kwargs)
    guidance.set_path(path)
    x, y, psi = start
    max_cross = 0.0
    trace = []
    for k in range(steps):
        speed, psi_d, done = guidance.update(x, y, psi)
        trace.append((x, y, guidance.index, guidance.cross_track))
        if done:
            return x, y, psi, k * dt, trace
        rate = max(-1.5, min(1.5, 2.0 * wrap_angle(psi_d - psi)))
        psi = wrap_angle(psi + rate * dt)
        x += speed * math.cos(psi) * dt
        y += speed * math.sin(psi) * dt
        max_cross = max(max_cross, abs(guidance.cross_track))
    return x, y, psi, None, trace


def test_heading_points_along_the_path_when_on_it():
    g = LOSGuidance()
    g.set_path([(0.0, 0.0), (5.0, 0.0)])
    _, psi_d, done = g.update(1.0, 0.0, 0.0)
    assert psi_d == pytest.approx(0.0) and not done


def test_heading_points_back_to_the_path():
    g = LOSGuidance(lookahead=0.5)
    g.set_path([(0.0, 0.0), (5.0, 0.0)])
    _, psi_d, _ = g.update(1.0, 1.0, 0.0)      # 1 m to the left of the path
    assert psi_d == pytest.approx(math.atan2(-1.0, 0.5))
    assert psi_d < 0.0                          # steer to the right


def test_speed_is_zero_when_pointing_away():
    g = LOSGuidance(cruise_speed=0.15)
    g.set_path([(0.0, 0.0), (5.0, 0.0)])
    speed, _, _ = g.update(1.0, 0.0, math.pi)  # facing backwards
    assert speed == pytest.approx(0.0)


def test_waypoint_switching():
    g = LOSGuidance(acceptance_radius=0.15)
    g.set_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    g.update(1.0, 0.0, 0.0)
    assert g.index == 0
    g.update(1.95, 0.0, 0.0)                    # inside the acceptance radius
    assert g.index == 1


def test_stops_at_last_waypoint_and_holds_heading():
    g = LOSGuidance(acceptance_radius=0.15)
    g.set_path([(0.0, 0.0), (2.0, 0.0)])
    speed, psi_d, done = g.update(1.9, 0.0, 0.0)
    assert done and speed == 0.0 and psi_d == pytest.approx(0.0)
    assert g.update(1.9, 0.0, 0.0) == (0.0, pytest.approx(0.0), True)


def test_final_approach_slows_down():
    g = LOSGuidance(cruise_speed=0.15, slow_gain=1.0, acceptance_radius=0.05)
    g.set_path([(0.0, 0.0), (5.0, 0.0)])
    speed, _, _ = g.update(4.9, 0.0, 0.0)       # 0.1 m from the goal
    assert speed == pytest.approx(0.1)


def test_duplicate_and_short_paths():
    g = LOSGuidance()
    g.set_path([(1.0, 1.0), (1.0, 1.0)])
    assert g.finished
    g.set_path([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)])
    assert not g.finished and len(g.points) == 2


def test_follows_square_path():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    x, y, _, t_end, trace = simulate(path, start=(0.0, -0.3, 0.0))
    assert t_end is not None                    # path completed
    assert math.hypot(x - 0.0, y - 2.0) < 0.15  # ended at the last waypoint
    # in the middle of the straight segments the cross-track error is small
    mid = [abs(c) for (_, _, i, c) in trace[len(trace) // 2:] if i >= 1]
    assert max(mid) < 0.25


def test_recovers_from_large_initial_offset():
    x, y, _, t_end, _ = simulate(
        [(0.0, 0.0), (4.0, 0.0)], start=(0.0, 1.0, math.pi / 2)
    )
    assert t_end is not None and math.hypot(x - 4.0, y) < 0.15
