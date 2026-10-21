"""Node-level tests: each node built from a scenario file via HERO_SIM_CONFIG."""
import math

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('crazyflie_interfaces')

from conftest import duration  # noqa: E402
from crazyflie_interfaces.srv import Arm, Takeoff  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402

from hero_sim.kinematics import yaw_to_quat  # noqa: E402

SCENARIO = """
ground_robots:
  - name: scout_1
    pose: {x: 1.0, y: 0.0, yaw: 1.5707963267948966}
    model: scout
  - name: herover_1
    pose: {x: 0.0, y: 0.0, yaw: 0.0}
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


def test_robot_models_latched_and_frame_locked(make):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray
    from hero_sim.robot_models import TOPIC, RobotModels

    publisher = make(RobotModels)  # publishes once, before anyone subscribes
    got = []
    listener = make(lambda: rclpy.create_node('rviz_stand_in'))
    listener.create_subscription(
        MarkerArray, TOPIC, got.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(listener)
    for _ in range(50):
        executor.spin_once(timeout_sec=0.05)
        if got:
            break
    executor.shutdown()
    assert got, 'late subscriber did not get the latched MarkerArray'

    markers = got[0].markers
    # herover_1 has no model -> TF only; scout_1 and both CFs get meshes
    assert [(m.header.frame_id, m.ns) for m in markers] == [
        ('cf_1', 'crazyflie'), ('cf_2', 'crazyflie'), ('scout_1', 'scout')]
    assert all(m.type == Marker.MESH_RESOURCE and m.frame_locked
               and m.mesh_use_embedded_materials for m in markers)
    assert markers[2].mesh_resource == 'package://hero_sim/models/scout/scout.glb'
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
    from hero_sim import config
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
    from hero_sim import config
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
