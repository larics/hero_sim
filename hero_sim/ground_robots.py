"""Kinematic ground robots: integrate /<name>/cmd_vel, publish /<name>/pose and TF.

cmd_vel is a body-frame Twist held until the next message. omni robots honor
linear.x, linear.y and angular.z; diff robots ignore linear.y. Poses are
published in 'world' at z=0, along with a world -> <name> transform.
"""
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from hero_sim import config
from hero_sim.kinematics import integrate_ground, limit_ground_cmd, yaw_to_quat

FRAME = 'world'


class _Robot:
    __slots__ = ('name', 'drive', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'pose_pub')

    def __init__(self, spec, pose_pub):
        self.name = spec.name
        self.drive = spec.drive
        self.x, self.y, self.yaw = spec.x, spec.y, spec.yaw
        self.vx = self.vy = self.wz = 0.0
        self.pose_pub = pose_pub


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
            self._robots.append(robot)

        self._tf = TransformBroadcaster(self)
        self.create_timer(self._dt, self._tick)

        summary = ', '.join(f'{r.name}({r.drive})' for r in self._robots) or 'none'
        self.get_logger().info(
            f'ground robots at {sim["ground_rate"]:g} Hz, max {self._max_speed:g} m/s, '
            f'{self._max_yaw_rate:g} rad/s: {summary}')

    def _on_cmd_vel(self, robot, msg):
        robot.vx, robot.vy, robot.wz = limit_ground_cmd(
            msg.linear.x, msg.linear.y, msg.angular.z,
            robot.drive, self._max_speed, self._max_yaw_rate)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for r in self._robots:
            r.x, r.y, r.yaw = integrate_ground(r.x, r.y, r.yaw, r.vx, r.vy, r.wz, self._dt)
            qx, qy, qz, qw = yaw_to_quat(r.yaw)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = FRAME
            pose.pose.position.x, pose.pose.position.y = r.x, r.y
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            r.pose_pub.publish(pose)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = FRAME
            t.child_frame_id = r.name
            t.transform.translation.x, t.transform.translation.y = r.x, r.y
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
