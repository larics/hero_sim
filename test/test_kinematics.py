import math

import pytest

from hero_sim.kinematics import (
    integrate_ground, limit_ground_cmd, quat_to_yaw, wrap_angle, yaw_to_quat,
)


@pytest.mark.parametrize('yaw', [0.0, 0.5, -2.0, math.pi / 2, 3.0])
def test_quat_roundtrip(yaw):
    assert quat_to_yaw(*yaw_to_quat(yaw)) == pytest.approx(yaw)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_omni_keeps_lateral_and_scales_speed():
    vx, vy, wz = limit_ground_cmd(3.0, 4.0, 0.5, 'omni', max_speed=2.0, max_yaw_rate=2.0)
    assert (vx, vy) == pytest.approx((1.2, 1.6))  # direction kept, |v| = 2
    assert wz == 0.5


def test_diff_drops_lateral():
    vx, vy, _ = limit_ground_cmd(1.0, 1.0, 0.0, 'diff', max_speed=2.0, max_yaw_rate=2.0)
    assert (vx, vy) == (1.0, 0.0)


def test_diff_clamps_reverse_speed():
    vx, _, _ = limit_ground_cmd(-5.0, 0.0, 0.0, 'diff', max_speed=2.0, max_yaw_rate=2.0)
    assert vx == pytest.approx(-2.0)


def test_yaw_rate_clamped_both_ways():
    assert limit_ground_cmd(0, 0, 9.0, 'omni', 2.0, 1.5)[2] == 1.5
    assert limit_ground_cmd(0, 0, -9.0, 'diff', 2.0, 1.5)[2] == -1.5


def test_integrate_is_body_frame():
    # Facing +y, "forward" moves along world +y and "left" along world -x.
    x, y, yaw = integrate_ground(0.0, 0.0, math.pi / 2, vx=1.0, vy=0.0, wz=0.0, dt=0.5)
    assert (x, y) == pytest.approx((0.0, 0.5))
    x, y, _ = integrate_ground(0.0, 0.0, math.pi / 2, vx=0.0, vy=1.0, wz=0.0, dt=0.5)
    assert (x, y) == pytest.approx((-0.5, 0.0))


def test_integrate_full_turn_returns_to_start():
    x, y, yaw = 0.0, 0.0, 0.0
    dt, steps = 0.01, 628  # ~2*pi at 1 rad/s
    for _ in range(steps):
        x, y, yaw = integrate_ground(x, y, yaw, 1.0, 0.0, 1.0, dt)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=0.02)
