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

"""Unit tests for PIDController logic, anti-windup and angle wrapping."""

import math

import pytest
from tsb_control.pid_node import PIDController


def test_pid_step_and_feedforward():
    """Verify proportional response and feedforward steady-state output."""
    pid = PIDController(kp=1.2, ki=0.0, kd=0.0, kff=1.0, output_limit=2.0)
    # At steady state (measurement == setpoint) the output is kff * setpoint
    out = pid.compute(setpoint=0.15, measurement=0.15, dt=0.05)
    assert out == pytest.approx(0.15, abs=1e-6)


def test_pid_saturation_and_anti_windup():
    """Verify output clamping and conditional anti-windup."""
    pid = PIDController(
        kp=2.0, ki=1.0, kd=0.0, kff=0.0, integral_limit=1.0, output_limit=1.0
    )
    for _ in range(50):
        out = pid.compute(setpoint=5.0, measurement=0.0, dt=0.1)
    assert out == pytest.approx(1.0, abs=1e-6)
    # Integral must not wind up while saturated
    assert pid.integral <= 1.0


def test_normalize_angle():
    """Verify angle wrapping into [-pi, pi]."""
    wrapped = PIDController.normalize_angle(3.0 * math.pi / 2.0)
    assert wrapped == pytest.approx(-math.pi / 2.0, abs=1e-6)


def test_closed_loop_reaches_setpoint_with_model_mismatch():
    """Plant gain 0.8 (feedforward alone gives 0.12): integral must fix it."""
    pid = PIDController(
        kp=1.2, ki=0.1, kd=0.05, kff=1.0, integral_limit=1.0, output_limit=0.22
    )
    v, dt = 0.0, 0.05
    for _ in range(3000):
        u = pid.compute(setpoint=0.15, measurement=v, dt=dt)
        v += dt * (0.8 * u - v) / 0.2
    assert v == pytest.approx(0.15, abs=3e-3)


def test_anti_windup_when_target_is_unreachable():
    """Output saturated for a long time: the integral must not wind up."""
    pid = PIDController(
        kp=1.2, ki=0.1, kd=0.0, kff=1.0, integral_limit=1.0, output_limit=0.22
    )
    for _ in range(2000):
        pid.compute(setpoint=0.30, measurement=0.22, dt=0.05)
    assert pid.integral < 0.5


def test_heading_error_takes_shortest_way():
    """Setpoint +3.1 rad, measurement -3.1 rad: the short way is negative."""
    pid = PIDController(
        kp=1.0, ki=0.0, kd=0.0, output_limit=5.0, is_angular=True
    )
    out = pid.compute(setpoint=3.1, measurement=-3.1, dt=0.05)
    assert out == pytest.approx(6.2 - 2.0 * math.pi, abs=1e-6)
