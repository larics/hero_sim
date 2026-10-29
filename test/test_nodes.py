"""Node-level tests: each node built from a scenario file via HERO_SIM_CONFIG."""
import math

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('crazyflie_interfaces')

from conftest import duration  # noqa: E402
from crazyflie_interfaces.srv import Arm, GoTo, Takeoff  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped, Twist  # noqa: E402

from hero_sim import config  # noqa: E402
from hero_sim.kinematics import yaw_to_quat  # noqa: E402

SCENARIO = """
ground_robots:
  - name: scout_1
    pose: {x: 1.0, y: 0.0, yaw: 1.5707963267948966}
    model: scout
  - name: herover_1
    pose: {x: 0.0, y: -1.0, yaw: 0.0}  # clear of scout_1's path in the cmd_vel test
    drive: diff
crazyflies:
  - pose: {x: 0.5, y: 0.5, z: 0.03}
    gripper: true
  - pose: {x: -0.5, y: 0.5, z: 0.03}
sim: {ground_rate: 10.0, cf_rate: 20.0, cf_battery_drain: 0.1}
"""


@pytest.fixture()
def make(rclpy_context, scenario_file):
    scenario_file(SCENARIO)
    nodes = []

    def build(cls):
        n = cls()
        nodes.append(n)
        return n
    yield build
    for n in nodes:
        n.destroy_node()


def twist(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
    return msg


def test_crazyflie_server_builds_drones_from_config(make):
    from hero_sim.crazyflie_server import CrazyflieServer
    server = make(CrazyflieServer)
    assert server.get_name() == 'crazyflie_server'
    assert list(server.drones) == ['cf_1', 'cf_2']
    assert list(server.drones['cf_2'].position) == [-0.5, 0.5, 0.03]
    assert server.drones['cf_1'].dt == pytest.approx(0.05)
    assert server.drones['cf_1'].battery_drain == 0.1


def test_crazyflie_server_broadcast_respects_group_mask(make):
    from hero_sim.crazyflie_server import CrazyflieServer
    server = make(CrazyflieServer)
    server.drones['cf_1'].group_mask = 0b01
    server.drones['cf_2'].group_mask = 0b10
    req = Takeoff.Request(height=1.0, duration=duration(1), group_mask=0b01)
    server._broadcast('takeoff', '_takeoff_cb', req, Takeoff.Response())
    assert server.drones['cf_1'].flying and not server.drones['cf_2'].flying

    server._broadcast('arm', '_arm_cb', Arm.Request(arm=False), Arm.Response())
    assert not any(d.armed for d in server.drones.values())


def test_crazyflie_server_update_all(make):
    from hero_sim.crazyflie_server import CrazyflieServer
    server = make(CrazyflieServer)
    req = Takeoff.Request(height=1.0, duration=duration(1))
    server._broadcast('takeoff', '_takeoff_cb', req, Takeoff.Response())
    server._update_all()
    assert all(d.position[2] > 0.03 for d in server.drones.values())


def test_ground_robots_integrate_cmd_vel(make):
    from hero_sim.ground_robots import GroundRobotSim
    sim = make(GroundRobotSim)
    scout, herover = sim._robots
    assert (scout.name, scout.drive, herover.drive) == ('scout_1', 'omni', 'diff')

    sim._on_cmd_vel(scout, twist(vx=1.0, vy=1.0))
    sim._on_cmd_vel(herover, twist(vx=1.0, vy=1.0, wz=9.0))
    assert (herover.vy, herover.wz) == (0.0, 2.0)  # diff drops vy; yaw rate clamped

    for _ in range(10):  # 1 s at 10 Hz
        sim._tick()
    # scout faces +y: forward -> +y, left -> -x
    assert (scout.x, scout.y) == pytest.approx((0.0, 1.0))


def test_ground_robots_publish_pose(make):
    from hero_sim.ground_robots import GroundRobotSim
    sim = make(GroundRobotSim)
    got = []
    sub_node = make(lambda: rclpy.create_node('pose_listener'))
    sub_node.create_subscription(PoseStamped, '/scout_1/pose', got.append, 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sim)
    executor.add_node(sub_node)
    for _ in range(50):
        executor.spin_once(timeout_sec=0.05)
        if got:
            break
    executor.shutdown()
    assert got, 'no pose received on /scout_1/pose'
    assert got[-1].header.frame_id == 'world'
    assert got[-1].pose.position.x == pytest.approx(1.0)
    assert got[-1].pose.position.z == pytest.approx(config.MARKER_HEIGHTS['scout'])


def test_ground_robots_stop_at_map_obstacles(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    # Heading +x into the icuas26_1 trunk whose west face is at x = -3.3.
    scenario_file('ground_robots: [{name: r, pose: {x: -4.5, y: 0.15}, radius: 0.3}]\n'
                  'sim: {ground_rate: 10.0}\n')
    sim = make(GroundRobotSim)
    robot = sim._robots[0]
    sim._on_cmd_vel(robot, twist(vx=1.0))
    for _ in range(30):  # 3 s at 1 m/s would carry it through the trunk
        sim._tick()
    assert robot.x == pytest.approx(-3.3 - robot.radius, abs=0.15)
    assert sim._obstacles.clearance(robot.x, robot.y) >= robot.radius

    sim._on_cmd_vel(robot, twist(vx=-1.0))  # backing away is never blocked
    sim._tick()
    assert robot.x == pytest.approx(-3.3 - robot.radius - 0.1, abs=0.15)


PILLAR_SCENARIO = """
ground_robots:
  - name: r
    pose: {x: 0.0, y: 0.0}
    radius: 0.3
crazyflies:
  - pose: {x: 0.0, y: 1.0, z: 1.0}
obstacles:
  - {x: 1.0, y: 0.0, radius: 0.2}
  - {x: 1.0, y: 1.0, radius: 0.2}
sim: {map: null, ground_rate: 10.0, cf_rate: 10.0}
"""


def test_ground_robots_stop_at_pillars(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(PILLAR_SCENARIO)
    sim = make(GroundRobotSim)
    robot = sim._robots[0]
    sim._on_cmd_vel(robot, twist(vx=1.0))
    for _ in range(20):  # 2 s at 1 m/s would carry it through the pillar
        sim._tick()
    assert 0.5 - 0.05 <= robot.x <= 0.5  # pillar edge at 0.8, minus the radius

    sim._on_cmd_vel(robot, twist(vy=1.0))  # sideways past it is free
    sim._tick()
    assert robot.y == pytest.approx(0.1)


def test_crazyflies_stop_at_pillars_at_any_height(make, scenario_file):
    from hero_sim.crazyflie_server import CrazyflieServer
    scenario_file(PILLAR_SCENARIO)
    server = make(CrazyflieServer)
    cf = server.drones['cf_1']
    req = GoTo.Request(goal=Point(x=2.0, y=1.0, z=1.5), duration=duration(1))
    cf._go_to_cb(req, GoTo.Response())
    for _ in range(20):
        server._update_all()
    assert 0.7 - 0.05 <= cf.position[0] <= 0.7  # pillar edge at 0.8, minus 0.1
    assert cf.position[2] == pytest.approx(1.5)  # height still goes through


def test_collision_off_ignores_pillars(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(PILLAR_SCENARIO.replace('map: null', 'map: null, collision: false'))
    sim = make(GroundRobotSim)
    assert sim._obstacles is None
    robot = sim._robots[0]
    sim._on_cmd_vel(robot, twist(vx=1.0))
    for _ in range(20):
        sim._tick()
    assert robot.x == pytest.approx(2.0)


GO_TO_SCENARIO = """
ground_robots:
  - name: omni_1
    pose: {x: 0.0, y: 0.0}
  - name: diff_1
    pose: {x: 0.0, y: 2.0}
    drive: diff
obstacles:
  - {x: 3.0, y: 0.0, radius: 0.2}
sim: {map: null, ground_rate: 10.0, ground_max_speed: 1.0, ground_max_yaw_rate: 2.0}
"""


def go_to(x, y, yaw=0.0, sec=0, relative=False):
    return GoTo.Request(goal=Point(x=x, y=y, z=5.0), yaw=yaw, duration=duration(sec),
                        relative=relative)


@pytest.fixture()
def ground(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(GO_TO_SCENARIO)
    return make(GroundRobotSim)


def test_ground_go_to_reaches_the_goal(ground):
    omni, _ = ground._robots
    ground._on_go_to(omni, go_to(1.0, 0.5, yaw=0.5, sec=2), GoTo.Response())
    assert omni.go_to.total == 2.0
    for _ in range(10):
        ground._tick()
    assert (omni.x, omni.y, omni.yaw) == pytest.approx((0.5, 0.25, 0.25))
    for _ in range(15):
        ground._tick()
    assert (omni.x, omni.y, omni.yaw) == pytest.approx((1.0, 0.5, 0.5))
    assert omni.go_to is None  # done: the robot holds still
    ground._tick()
    assert (omni.x, omni.y) == pytest.approx((1.0, 0.5))


def test_ground_go_to_held_to_max_speed(ground):
    omni, _ = ground._robots
    ground._on_go_to(omni, go_to(2.0, 0.0), GoTo.Response())  # duration 0
    assert omni.go_to.total == pytest.approx(2.0)  # ground_max_speed 1 m/s
    for _ in range(10):
        ground._tick()
    assert omni.x == pytest.approx(1.0)


def test_ground_go_to_relative_and_diff(ground):
    _, diff = ground._robots
    ground._on_go_to(diff, go_to(1.0, 0.0, relative=True), GoTo.Response())
    for _ in range(15):
        ground._tick()
    assert (diff.x, diff.y, diff.yaw) == pytest.approx((1.0, 2.0, 0.0))


def test_cmd_vel_cancels_a_go_to(ground):
    omni, _ = ground._robots
    ground._on_go_to(omni, go_to(2.0, 0.0, sec=4), GoTo.Response())
    for _ in range(5):
        ground._tick()
    ground._on_cmd_vel(omni, twist())
    assert omni.go_to is None
    stopped = omni.x
    for _ in range(5):
        ground._tick()
    assert omni.x == stopped == pytest.approx(0.25)


def test_ground_go_to_stops_at_a_pillar(ground):
    omni, _ = ground._robots
    ground._on_go_to(omni, go_to(5.0, 0.0), GoTo.Response())
    for _ in range(60):
        ground._tick()
    assert 2.5 - 0.05 <= omni.x <= 2.5  # pillar edge at 2.8, minus the radius 0.3
    assert omni.go_to is not None  # past its 5 s, still waiting for the way to clear
    ground._on_cmd_vel(omni, twist())
    assert omni.go_to is None


def test_ground_go_to_service_call(ground, make):
    client = make(lambda: rclpy.create_node('go_to_client'))
    cli = client.create_client(GoTo, '/diff_1/go_to')
    assert cli.wait_for_service(timeout_sec=2.0)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ground)
    executor.add_node(client)
    future = cli.call_async(go_to(1.0, 3.0, sec=2))
    executor.spin_until_future_complete(future, timeout_sec=2.0)
    executor.shutdown()
    assert future.done()
    goal = ground._robots[1].go_to.goal
    assert goal == pytest.approx((1.0, 3.0, 0.0))


TWO_SCENARIO = """
ground_robots:
  - name: a
    pose: {x: 0.0, y: 0.0}
    radius: 0.3
  - name: b
    pose: {x: 2.0, y: 0.0, yaw: 3.141592653589793}
    radius: 0.2
sim: {map: null, ground_rate: 10.0, ground_max_speed: 1.0}
"""


def gap(a, b):
    """Distance between the edges of two robots' footprints; < 0 when they overlap."""
    return math.hypot(a.x - b.x, a.y - b.y) - a.radius - b.radius


def test_ground_robots_stop_at_each_other(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(TWO_SCENARIO)
    sim = make(GroundRobotSim)
    a, b = sim._robots
    sim._on_cmd_vel(a, twist(vx=1.0))
    sim._on_cmd_vel(b, twist(vx=1.0))  # b faces -x: head-on
    for _ in range(20):
        sim._tick()
        assert gap(a, b) >= 0.0
    assert gap(a, b) <= 0.05  # touching, within a path sample each
    assert (a.y, b.y) == pytest.approx((0.0, 0.0))

    sim._on_cmd_vel(a, twist(vx=-1.0))  # backing away is free, b still pushing
    sim._on_cmd_vel(b, twist())
    sim._tick()
    assert gap(a, b) == pytest.approx(0.1, abs=0.05)


@pytest.mark.parametrize('change', ['radius: 0.2|radius: 0.0',
                                    'map: null|map: null, collision: false'])
def test_ground_robots_pass_through_without_collisions(make, scenario_file, change):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(TWO_SCENARIO.replace(*change.split('|')))
    sim = make(GroundRobotSim)
    a, _ = sim._robots
    sim._on_cmd_vel(a, twist(vx=1.0))
    for _ in range(30):
        sim._tick()
    assert a.x == pytest.approx(3.0)


def test_blocked_go_to_waits_then_carries_on(make, scenario_file):
    from hero_sim.ground_robots import GroundRobotSim
    scenario_file(TWO_SCENARIO)
    sim = make(GroundRobotSim)
    a, b = sim._robots
    sim._on_go_to(a, go_to(4.0, 0.0), GoTo.Response())  # 4 s at 1 m/s, b in the way
    for _ in range(50):
        sim._tick()
    assert 1.5 - 0.05 <= a.x <= 1.5 and a.go_to is not None  # waiting at b

    sim._on_cmd_vel(b, twist(vy=1.0))  # b faces -x: its left is -y
    for i in range(50):
        if i == 10:
            sim._on_cmd_vel(b, twist())  # b parked at (2, -1)
        before = a.x
        sim._tick()
        assert gap(a, b) >= 0.0
        assert a.x - before <= 0.1 + 0.03  # resumes at its speed, no jump to the schedule
    assert b.y == pytest.approx(-1.0)
    assert (a.x, a.y) == pytest.approx((4.0, 0.0)) and a.go_to is None


def _latched(make, topic, publisher_cls):
    """The first MarkerArray a late TRANSIENT_LOCAL subscriber gets on `topic`."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import MarkerArray
    publisher = make(publisher_cls)  # publishes once, before anyone subscribes
    got = []
    listener = make(lambda: rclpy.create_node('rviz_stand_in'))
    listener.create_subscription(
        MarkerArray, topic, got.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(listener)
    for _ in range(50):
        executor.spin_once(timeout_sec=0.05)
        if got:
            break
    executor.shutdown()
    assert got, f'late subscriber did not get the latched MarkerArray on {topic}'
    return got[0]


def test_pillars_drawn_latched_even_with_collision_off(make, scenario_file):
    from visualization_msgs.msg import Marker
    from hero_sim.ground_robots import OBSTACLE_TOPIC, PILLAR_HEIGHT, GroundRobotSim

    scenario_file(PILLAR_SCENARIO.replace('map: null', 'map: null, collision: false'))
    markers = _latched(make, OBSTACLE_TOPIC, GroundRobotSim).markers
    assert markers[0].action == Marker.DELETEALL
    pillars = markers[1:]
    assert [(m.ns, m.id, m.type, m.header.frame_id) for m in pillars] == [
        ('obstacles', 0, Marker.CYLINDER, 'world'), ('obstacles', 1, Marker.CYLINDER, 'world')]
    assert [(m.pose.position.x, m.pose.position.y) for m in pillars] == [(1.0, 0.0), (1.0, 1.0)]
    assert all(m.scale.x == m.scale.y == pytest.approx(0.4) for m in pillars)
    assert all(m.scale.z == PILLAR_HEIGHT for m in pillars)


def test_robot_models_latched_and_frame_locked(make):
    from visualization_msgs.msg import Marker
    from hero_sim.robot_models import TOPIC, RobotModels

    markers = _latched(make, TOPIC, RobotModels).markers
    # herover_1 has no model -> TF only; scout_1 and both CFs get meshes
    assert [(m.header.frame_id, m.ns) for m in markers] == [
        ('cf_1', 'crazyflie'), ('cf_2', 'crazyflie'), ('scout_1', 'scout')]
    assert all(m.type == Marker.MESH_RESOURCE and m.frame_locked
               and m.mesh_use_embedded_materials for m in markers)
    assert markers[2].mesh_resource == 'package://hero_sim/models/scout/scout.glb'
    # CF meshes sit on their frame; the scout's drops from its marker height to the floor
    assert [m.pose.position.z for m in markers] == pytest.approx(
        [0.0, 0.0, -config.MARKER_HEIGHTS['scout']])
    assert len({(m.ns, m.id) for m in markers}) == len(markers)


class _Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _pose(x, y, z, yaw):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = yaw_to_quat(yaw)
    return msg


def test_gripper_box_faces_point_out():
    from hero_sim.gripper import box
    center = (0.1, -0.2, 0.3)
    tris = box(center, (0.0, 0.6, 0.8), (0.0, -0.8, 0.6), (0.5, 0.0, 0.0))  # rotated
    assert len(tris) == 36
    for a, b, c in zip(tris[0::3], tris[1::3], tris[2::3]):
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        normal = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        out = [(a[i] + b[i] + c[i]) / 3 - center[i] for i in range(3)]
        assert sum(n * o for n, o in zip(normal, out)) > 0


def test_gripper_claws_shut_around_the_bar():
    from hero_sim.gripper import BAR_Z, _claw, OPEN_ANGLE
    shut, opened = _claw(0.0), _claw(OPEN_ANGLE)
    assert shut[0] == opened[0]  # same pivot
    tip_y, tip_z = shut[-1]
    assert 0.0 < tip_y < 0.005 and tip_z > BAR_Z + 0.015  # curled over a 3 cm bar
    assert opened[-1][0] > 0.03  # open: tips 6 cm+ apart, the bar fits between


def test_grippers_lifecycle(rclpy_context, node):
    from visualization_msgs.msg import Marker
    from hero_sim.gripper import CLOSED, OPEN, Grippers

    cfs = (config.Crazyflie('cf_1', 0, 0, 0, gripper=True), config.Crazyflie('cf_2', 0, 0, 0))
    grippers = Grippers(node, cfs)
    grippers._pub = capture = _Capture()
    assert (grippers.state('cf_1'), grippers.state('cf_2')) == (OPEN, None)

    assert not grippers.close('cf_2') and not grippers.close('scout_1')  # nothing to shut
    assert grippers.close('cf_1') and grippers.state('cf_1') == CLOSED
    markers = capture.msgs[-1].markers
    assert markers[0].action == Marker.DELETEALL
    assert [(m.header.frame_id, m.ns, m.frame_locked) for m in markers[1:]] == [
        ('cf_1', 'gripper', True)]

    assert grippers.release('cf_1', 1.0, 2.0, 0.65, math.pi / 2)
    assert grippers.state('cf_1') is None and grippers.left == [(1.0, 2.0, 0.65, math.pi / 2)]
    left = capture.msgs[-1].markers[1]
    assert (left.header.frame_id, left.ns, left.frame_locked) == ('world', 'gripper_left', False)
    assert left.pose.position.z == 0.65 and left.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert not grippers.release('cf_1', 0.0, 0.0, 0.0, 0.0)  # already gone

    resp = grippers._on_add('cf_1', Trigger.Request(), Trigger.Response())
    assert resp.success and grippers.state('cf_1') == OPEN
    assert len(capture.msgs[-1].markers) == 3  # DELETEALL, cf_1's new one, the one left behind
    published = len(capture.msgs)
    resp = grippers._on_add('cf_1', Trigger.Request(), Trigger.Response())
    assert resp.success and 'no-op' in resp.message and len(capture.msgs) == published


def test_grippers_not_drawn_without_models(rclpy_context, node):
    from hero_sim.gripper import TOPIC, Grippers

    grippers = Grippers(node, (config.Crazyflie('cf_1', 0, 0, 0, gripper=True),), draw=False)
    assert node.count_publishers(TOPIC) == 0
    assert grippers.close('cf_1') and grippers.release('cf_1', 0.0, 0.0, 0.5, 0.0)
    assert grippers.state('cf_1') is None and len(grippers.left) == 1  # state still kept


def test_attach_follows_parent_in_body_frame(make):
    pytest.importorskip('hero_mrs_msgs')
    from hero_mrs_msgs.srv import AttachAgent
    from hero_sim.attach import SimAttachNode

    node = make(SimAttachNode)
    node._poses['herover_1'] = _pose(1.0, 0.0, 0.0, math.pi / 2)
    node._poses['cf_1'] = _pose(1.0, 1.0, 0.2, math.pi / 2)  # 1 m ahead of the parent
    req = AttachAgent.Request(child_id='cf_1', parent_id='herover_1', attach=True)
    resp = node._on_request(req, AttachAgent.Response())
    assert resp.success, resp.message

    couple = node._couples['cf_1']
    real_publisher, capture = couple.publisher, _Capture()
    couple.publisher = capture
    node._poses['herover_1'] = _pose(2.0, 0.0, 0.0, 0.0)  # parent turned to face +x
    node._follow_tick()
    couple.publisher = real_publisher  # detach must destroy a real publisher
    msg = capture.msgs[-1]
    assert (msg.x, msg.y, msg.z, msg.yaw) == pytest.approx((3.0, 0.0, 0.2, 0.0))

    req.attach = False
    assert node._on_request(req, AttachAgent.Response()).success
    assert 'cf_1' not in node._couples
    assert node._on_request(req, AttachAgent.Response()).success  # idempotent
    assert node._grippers.state('cf_1') == 'open'  # docking leaves the gripper alone


def test_attach_without_parent_locks_in_place(make):
    pytest.importorskip('hero_mrs_msgs')
    from hero_mrs_msgs.srv import AttachAgent
    from hero_sim.attach import SimAttachNode

    node = make(SimAttachNode)
    node._poses['cf_1'] = _pose(1.0, 2.0, 0.65, -math.pi / 2)  # hanging on a branch
    req = AttachAgent.Request(child_id='cf_1', parent_id='', attach=True)
    resp = node._on_request(req, AttachAgent.Response())
    assert resp.success, resp.message
    assert node._grippers.state('cf_1') == 'closed'

    couple = node._couples['cf_1']
    real_publisher, capture = couple.publisher, _Capture()
    couple.publisher = capture
    node._poses['cf_1'] = _pose(1.0, 2.0, 0.0, 0.0)  # disarmed: whatever it reports
    node._follow_tick()
    couple.publisher = real_publisher
    msg = capture.msgs[-1]
    assert (msg.x, msg.y, msg.z, msg.yaw) == pytest.approx((1.0, 2.0, 0.65, -math.pi / 2))

    req.attach = False
    resp = node._on_request(req, AttachAgent.Response())
    assert resp.success and 'lock' in resp.message and 'gripper left' in resp.message
    assert 'cf_1' not in node._couples
    assert node._grippers.state('cf_1') is None
    assert node._grippers.left == [pytest.approx((1.0, 2.0, 0.65, -math.pi / 2))]


def test_attach_fails_without_pose(make):
    pytest.importorskip('hero_mrs_msgs')
    from hero_mrs_msgs.srv import AttachAgent
    from hero_sim.attach import SimAttachNode

    node = make(SimAttachNode)
    node._pose_wait_timeout = 0.1
    req = AttachAgent.Request(child_id='cf_9', parent_id='ghost', attach=True)
    resp = node._on_request(req, AttachAgent.Response())
    assert not resp.success and 'ghost' in resp.message


TETHER_SCENARIO = """
ground_robots:
  - name: ugv
    pose: {x: -1.0, y: 0.0}
crazyflies:
  - pose: {x: 1.0, y: 0.0, z: 1.0}
tether:
  winch: {x: -2.0, y: 0.0, z: 0.0}
  object: {x: 3.0, y: 0.0}
  object_size: 0.2
  goal: {x: 1.0, y: 0.0, radius: 0.3}
  slack: 0.0
  max_reel_speed: 1.0
sim: {ground_rate: 10.0, map: null}
"""


def pose_msg(x, y, z):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    return msg


def test_tether_sim_reels_the_object_in(rclpy_context, scenario_file):
    from std_msgs.msg import Float64
    from hero_sim.tether_sim import TetherSim

    scenario_file(TETHER_SCENARIO)
    node = TetherSim()
    try:
        node._tick()  # no poses yet: waits
        assert node._state is None

        node._on_ugv(pose_msg(-1.0, 0.0, 0.0))
        node._on_uav(pose_msg(1.0, 0.0, 1.0))
        node._tick()
        start = node._length
        assert node._state.obj == pytest.approx((3.0, 0.0))
        assert node._state.taut  # slack 0: taut from the start

        node._on_rate(Float64(data=-5.0))  # clamped to max_reel_speed 1.0
        node._tick()
        assert node._length == pytest.approx(start - 0.1)  # 1 m/s at 10 Hz
        assert 1.0 < node._state.obj[0] < 3.0
        assert not node._reached

        for _ in range(30):
            node._tick()
        assert node._state.obj == pytest.approx((1.0, 0.0))  # under the UAV, never lifted
        assert node._reached
    finally:
        node.destroy_node()


def test_tether_sim_object_jams_on_a_pillar(rclpy_context, scenario_file):
    from std_msgs.msg import Float64
    from hero_sim.tether_sim import TetherSim

    # A pillar between the object (3, 0) and the point under the UAV (1, 0).
    scenario_file(TETHER_SCENARIO + 'obstacles: [{x: 2.0, y: 0.0, radius: 0.2}]\n')
    node = TetherSim()
    try:
        node._on_ugv(pose_msg(-1.0, 0.0, 0.0))
        node._on_uav(pose_msg(1.0, 0.0, 1.0))
        node._on_rate(Float64(data=-1.0))
        for _ in range(30):
            node._tick()
        assert node._state.stuck and not node._reached
        assert 2.3 <= node._state.obj[0] <= 2.35  # pillar 0.2 + object 0.1
    finally:
        node.destroy_node()


def test_tether_sim_needs_a_tether_section(make):
    from hero_sim.tether_sim import TetherSim
    with pytest.raises(RuntimeError, match='no tether'):
        make(TetherSim)
