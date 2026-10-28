"""Kinematic ground robots: integrate /<name>/cmd_vel, publish /<name>/pose and TF.

cmd_vel is a body-frame Twist held until the next message. omni robots honor
linear.x, linear.y and angular.z; diff robots ignore linear.y. Poses are
published in 'world' at the robot's pose z (its marker height, see
config.MARKER_HEIGHTS), along with a world -> <name> transform.

/<name>/go_to (crazyflie_interfaces/srv/GoTo, as for the Crazyflies) drives
to goal x, y and yaw in `duration`, or slower to stay within the speed limits
(kinematics.GroundGoTo). relative makes x, y an offset in the world frame;
goal.z and group_mask are ignored. Any cmd_vel message cancels it.

With sim.collision on, a robot stops where its footprint would hit the map or
a pillar (hero_sim/obstacles.py) and resumes when cmd_vel points away.

This node also draws the scenario's pillars (its obstacles: list), latched on
/hero_sim/obstacles: one CYLINDER per pillar, at its x, y, scale.x = diameter.
"""
from crazyflie_interfaces.srv import GoTo
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from hero_sim import config, obstacles
from hero_sim.kinematics import GroundGoTo, integrate_ground, limit_ground_cmd, yaw_to_quat

FRAME = 'world'
OBSTACLE_TOPIC = '/hero_sim/obstacles'
PILLAR_HEIGHT = 2.0  # m, drawn only: pillars block at any height


class _Robot:
    __slots__ = ('name', 'drive', 'radius', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'wz', 'pose_pub',
                 'go_to', 'go_to_t')

    def __init__(self, spec, pose_pub):
        self.name = spec.name
        self.drive = spec.drive
        self.radius = spec.radius
        self.x, self.y, self.z, self.yaw = spec.x, spec.y, spec.z, spec.yaw
        self.vx = self.vy = self.wz = 0.0
        self.pose_pub = pose_pub
        self.go_to = None  # GroundGoTo while one runs
        self.go_to_t = 0.0  # s into it


class GroundRobotSim(Node):

    def __init__(self):
        super().__init__('ground_robot_sim')
        self.declare_parameter('config_path', '')
        config_path = config.resolve_config(self.get_parameter('config_path').value)
        scenario = config.load(config_path)
        sim = scenario.sim
        self._max_speed = sim['ground_max_speed']
        self._max_yaw_rate = sim['ground_max_yaw_rate']
        self._dt = 1.0 / sim['ground_rate']

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        self._robots = []
        for spec in scenario.ground_robots:
            robot = _Robot(spec, self.create_publisher(PoseStamped, f'/{spec.name}/pose', qos))
            self.create_subscription(Twist, f'/{spec.name}/cmd_vel',
                                     lambda msg, r=robot: self._on_cmd_vel(r, msg), qos)
            self.create_service(GoTo, f'/{spec.name}/go_to',
                                lambda req, resp, r=robot: self._on_go_to(r, req, resp))
            self._robots.append(robot)

        self._obstacles = obstacles.load(sim, [r.radius for r in self._robots],
                                         scenario.obstacles)
        if self._obstacles is not None:
            for r in self._robots:
                if self._obstacles.clearance(r.x, r.y) < r.radius:
                    self.get_logger().warn(f'{r.name} starts overlapping an obstacle; '
                                           'it can only move away from it')
        self._publish_pillars(scenario.obstacles)

        self._tf = TransformBroadcaster(self)
        self.create_timer(self._dt, self._tick)

        summary = ', '.join(f'{r.name}({r.drive})' for r in self._robots) or 'none'
        collisions = 'off' if self._obstacles is None else 'on'
        self.get_logger().info(
            f'ground robots at {sim["ground_rate"]:g} Hz, max {self._max_speed:g} m/s, '
            f'{self._max_yaw_rate:g} rad/s, collisions {collisions}, '
            f'{len(scenario.obstacles)} pillar(s): {summary}')

    def _publish_pillars(self, pillars):
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pillar_pub = self.create_publisher(MarkerArray, OBSTACLE_TOPIC, latched)
        clear = Marker()
        clear.header.frame_id = FRAME
        clear.action = Marker.DELETEALL
        markers = [clear]
        for i, (x, y, radius) in enumerate(pillars):
            m = Marker()
            m.header.frame_id = FRAME
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, PILLAR_HEIGHT / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * radius
            m.scale.z = PILLAR_HEIGHT
            m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.45, 0.4, 1.0
            markers.append(m)
        self._pillar_pub.publish(MarkerArray(markers=markers))

    def _on_cmd_vel(self, robot, msg):
        robot.go_to = None
        robot.vx, robot.vy, robot.wz = limit_ground_cmd(
            msg.linear.x, msg.linear.y, msg.angular.z,
            robot.drive, self._max_speed, self._max_yaw_rate)

    def _on_go_to(self, robot, request, response):
        x, y = request.goal.x, request.goal.y
        if request.relative:
            x, y = robot.x + x, robot.y + y
        duration = request.duration.sec + request.duration.nanosec * 1e-9
        robot.go_to = GroundGoTo((robot.x, robot.y, robot.yaw), (x, y, request.yaw), robot.drive,
                                 duration, self._max_speed, self._max_yaw_rate)
        robot.go_to_t = 0.0
        robot.vx = robot.vy = robot.wz = 0.0
        total = robot.go_to.total
        slower = f' (asked {duration:.2f}s, held to the speed limits)' if total > duration else ''
        self.get_logger().info(
            f'[{robot.name}] GO_TO ({x:.2f}, {y:.2f}) yaw={request.yaw:.2f} '
            f'in {total:.2f}s{slower}')
        return response

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for r in self._robots:
            if r.go_to is not None:
                r.go_to_t += self._dt
                x, y, r.yaw = r.go_to.at(r.go_to_t)
                if r.go_to_t >= r.go_to.total:
                    r.go_to = None
            else:
                x, y, r.yaw = integrate_ground(r.x, r.y, r.yaw, r.vx, r.vy, r.wz, self._dt)
            if self._obstacles is not None:
                x, y = self._obstacles.move(r.x, r.y, x, y, r.radius)
            r.x, r.y = x, y
            qx, qy, qz, qw = yaw_to_quat(r.yaw)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = FRAME
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = r.x, r.y, r.z
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            r.pose_pub.publish(pose)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = FRAME
            t.child_frame_id = r.name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
                r.x, r.y, r.z)
            t.transform.rotation = o
            transforms.append(t)
        if transforms:
            self._tf.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = GroundRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
