"""SimCrazyflie behaviour. Needs a ROS 2 environment with crazyflie_interfaces."""
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('crazyflie_interfaces')

from conftest import duration  # noqa: E402
from crazyflie_interfaces.msg import FullState, Hover, Position, Status, VelocityWorld  # noqa: E402
from crazyflie_interfaces.srv import Arm, GoTo, Land, NotifySetpointsStop, Takeoff  # noqa: E402
from geometry_msgs.msg import Point, Quaternion  # noqa: E402

from hero_sim.crazyflie import SimCrazyflie  # noqa: E402
from hero_sim.kinematics import yaw_to_quat  # noqa: E402


@pytest.fixture()
def cf(node):
    return SimCrazyflie(node, 'cf_test', (0.0, 0.0, 0.0), dt=0.1, battery_drain=0.025)


def takeoff(cf, height=1.0, sec=2, group_mask=0):
    req = Takeoff.Request(height=height, duration=duration(sec), group_mask=group_mask)
    cf._takeoff_cb(req, Takeoff.Response())


def land(cf, height=0.0, sec=2):
    cf._land_cb(Land.Request(height=height, duration=duration(sec)), Land.Response())


def go_to(cf, x, y, z, yaw=0.0, sec=2, relative=False):
    req = GoTo.Request(goal=Point(x=float(x), y=float(y), z=float(z)), yaw=float(yaw),
                       duration=duration(sec), relative=relative)
    cf._go_to_cb(req, GoTo.Response())


def run(cf, ticks):
    for _ in range(ticks):
        cf.update()


def test_initial_state(cf):
    np.testing.assert_allclose(cf.position, [0, 0, 0])
    assert cf.armed and not cf.flying and not cf.emergency
    assert cf.battery_voltage == pytest.approx(4.2)


def test_takeoff_goto_land_cycle(cf):
    takeoff(cf, height=1.0, sec=2)
    assert cf.flying
    cf.update()
    assert cf.position[2] == pytest.approx(0.05)  # linear: 1 tick of a 2 s climb
    run(cf, 25)
    np.testing.assert_allclose(cf.position, [0, 0, 1.0], atol=1e-9)
    assert not cf.motion_active

    go_to(cf, 3, 4, 1, yaw=1.0, sec=3)
    run(cf, 35)
    np.testing.assert_allclose(cf.position, [3, 4, 1], atol=1e-9)
    assert cf.yaw == pytest.approx(1.0)

    land(cf, height=0.0, sec=2)
    run(cf, 25)
    assert cf.position[2] == pytest.approx(0.0)
    assert not cf.flying


def test_takeoff_rejected_when_disarmed(cf):
    cf._arm_cb(Arm.Request(arm=False), Arm.Response())
    takeoff(cf)
    assert not cf.flying and not cf.motion_active


def test_disarm_stops_everything(cf):
    takeoff(cf)
    cf.hover_cmd = Hover()
    cf._arm_cb(Arm.Request(arm=False), Arm.Response())
    assert not cf.armed and not cf.flying and not cf.motion_active and cf.hover_cmd is None


def test_goto_relative(cf):
    cf.position = np.array([1.0, 2.0, 0.5])
    go_to(cf, 1, 1, 0.5, relative=True)
    np.testing.assert_allclose(cf.goal, [2, 3, 1])


def test_group_mask(cf):
    cf.group_mask = 0b0101
    assert cf._matches_group(0) and cf._matches_group(0b0001)
    assert not cf._matches_group(0b0010)
    takeoff(cf, group_mask=0b0010)
    assert not cf.flying


def test_emergency_falls_and_ignores_commands(cf):
    cf.position = np.array([0.0, 0.0, 2.0])
    cf.flying = True
    cf.emergency = True
    go_to(cf, 5, 5, 5)
    run(cf, 15)  # 2 m at 2 m/s
    assert cf.position[2] == 0.0
    assert not cf.flying


def test_battery_drains_per_second_while_flying(cf):
    cf.flying = True
    run(cf, 100)  # 10 s
    assert cf.battery_voltage == pytest.approx(4.2 - 0.25)


def test_battery_same_drain_per_second_at_other_rate(node):
    fast = SimCrazyflie(node, 'cf_fast', dt=0.02, battery_drain=0.025)
    fast.flying = True
    run(fast, 500)  # 10 s
    assert fast.battery_voltage == pytest.approx(4.2 - 0.25)


def test_battery_floor_and_low_power(cf):
    cf.flying = True
    cf.battery_voltage = 3.25
    cf.update()
    assert cf.pm_state == Status.PM_STATE_LOW_POWER
    cf.battery_voltage = 3.0001
    cf.battery_drain = 10.0
    cf.update()
    assert cf.battery_voltage == 3.0


def test_battery_steady_on_ground(cf):
    run(cf, 100)
    assert cf.battery_voltage == pytest.approx(4.2)


def test_hover_moves_in_body_frame_and_holds(cf):
    cf.position = np.array([0.0, 0.0, 1.0])
    cf.yaw = np.pi / 2
    cf._cmd_hover_cb(Hover(vx=1.0, vy=0.0, yaw_rate=0.0, z_distance=1.0))
    run(cf, 10)  # 1 s, and the command stays active without new messages
    np.testing.assert_allclose(cf.position, [0.0, 1.0, 1.0], atol=1e-9)


def test_hover_approaches_height_at_limited_speed(cf):
    cf._cmd_hover_cb(Hover(z_distance=1.0))
    cf.update()
    assert cf.position[2] == pytest.approx(0.1)


def test_hover_cancels_goto_and_notify_stop_clears(cf):
    go_to(cf, 5, 5, 5)
    cf._cmd_hover_cb(Hover(z_distance=1.0))
    assert not cf.motion_active
    cf.last_low_level_time = cf.node.get_clock().now()
    cf._notify_setpoints_stop_cb(NotifySetpointsStop.Request(), NotifySetpointsStop.Response())
    assert cf.hover_cmd is None and cf.last_low_level_time is None


def test_cmd_position_sets_pose_and_blocks_high_level(cf):
    go_to(cf, 5, 5, 5)
    cf._cmd_position_cb(Position(x=4.0, y=5.0, z=6.0, yaw=1.0))
    run(cf, 3)  # within LOW_LEVEL_TIMEOUT: stays where the setpoint put it
    np.testing.assert_allclose(cf.position, [4, 5, 6])
    assert cf.yaw == pytest.approx(1.0)


def test_cmd_full_state_reads_yaw(cf):
    msg = FullState()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1.0, 2.0, 3.0
    qx, qy, qz, qw = yaw_to_quat(0.7)
    msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    cf._cmd_full_state_cb(msg)
    np.testing.assert_allclose(cf.position, [1, 2, 3])
    assert cf.yaw == pytest.approx(0.7)


def test_cmd_velocity_world_steps_one_tick(cf):
    cf.position = np.array([0.0, 0.0, 1.0])
    msg = VelocityWorld()
    msg.vel.x = 1.0
    cf._cmd_velocity_world_cb(msg)
    assert cf.position[0] == pytest.approx(0.1)


def test_set_position_teleports(cf):
    go_to(cf, 9, 9, 9)
    req = GoTo.Request(goal=Point(x=10.0, y=20.0, z=30.0), yaw=1.5)
    cf._set_position_cb(req, GoTo.Response())
    np.testing.assert_allclose(cf.position, [10, 20, 30])
    assert cf.yaw == 1.5 and not cf.motion_active


def test_update_returns_world_transform(cf):
    cf.position = np.array([1.0, 2.0, 3.0])
    t = cf.update()
    assert (t.header.frame_id, t.child_frame_id) == ('world', 'cf_test')
    assert (t.transform.translation.x, t.transform.translation.y,
            t.transform.translation.z) == (1.0, 2.0, 3.0)
