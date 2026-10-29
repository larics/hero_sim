import math

import pytest

from hero_sim.kinematics import (
    GroundGoTo, integrate_ground, limit_ground_cmd, quat_to_yaw, wrap_angle, yaw_to_quat,
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


def test_go_to_omni_goes_straight_in_the_duration():
    g = GroundGoTo((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), 'omni', 4.0, 2.0, 2.0)
    assert g.total == 4.0
    assert g.at(2.0) == pytest.approx((0.5, 0.5, 0.5))
    assert g.at(4.0) == pytest.approx((1.0, 1.0, 1.0))
    assert g.at(9.0) == pytest.approx((1.0, 1.0, 1.0))


def test_go_to_held_to_the_speed_limits():
    assert GroundGoTo((0, 0, 0), (4, 0, 0), 'omni', 1.0, 2.0, 2.0).total == pytest.approx(2.0)
    assert GroundGoTo((0, 0, 0), (0, 0, 3), 'omni', 0.0, 2.0, 1.0).total == pytest.approx(3.0)


def test_go_to_turns_the_short_way():
    g = GroundGoTo((0.0, 0.0, 3.0), (0.0, 0.0, -3.0), 'omni', 0.0, 2.0, 1.0)
    assert g.total == pytest.approx(2 * math.pi - 6.0)  # through pi, not 6 rad back
    assert abs(g.at(g.total / 2)[2]) == pytest.approx(math.pi)


def test_go_to_diff_turns_drives_then_turns():
    # Facing +x, goal 2 m up +y facing -x: 90 deg in place, 2 m straight, 90 deg in place.
    g = GroundGoTo((0.0, 0.0, 0.0), (0.0, 2.0, math.pi), 'diff', 0.0,
                   max_speed=1.0, max_yaw_rate=math.pi / 2)
    assert g.total == pytest.approx(4.0)
    assert g.at(0.5) == pytest.approx((0.0, 0.0, math.pi / 4))
    assert g.at(2.0) == pytest.approx((0.0, 1.0, math.pi / 2))
    assert g.at(3.5) == pytest.approx((0.0, 2.0, 3 * math.pi / 4))
    assert g.at(4.0) == pytest.approx((0.0, 2.0, math.pi))


def test_go_to_diff_stretches_every_phase_alike():
    g = GroundGoTo((0.0, 0.0, 0.0), (0.0, 2.0, math.pi), 'diff', 8.0, 1.0, math.pi / 2)
    assert g.total == 8.0
    assert g.at(1.0) == pytest.approx((0.0, 0.0, math.pi / 4))  # half the yaw rate
    assert g.at(4.0) == pytest.approx((0.0, 1.0, math.pi / 2))  # half the speed


def test_go_to_diff_barely_moving_only_turns():
    g = GroundGoTo((1.0, 1.0, 0.0), (1.005, 1.0, 1.0), 'diff', 0.0, 1.0, 2.0)
    assert g.total == pytest.approx(0.5)
    assert g.at(0.25)[2] == pytest.approx(0.5)
    assert g.at(0.5) == pytest.approx((1.005, 1.0, 1.0))


def test_go_to_nowhere_is_done_at_once():
    g = GroundGoTo((1.0, 2.0, 0.5), (1.0, 2.0, 0.5), 'omni', 0.0, 2.0, 2.0)
    assert g.total == 0.0
    assert g.at(0.0) == pytest.approx((1.0, 2.0, 0.5))
