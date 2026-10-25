"""Winch, tether and object for the scenario's tether: section (model in hero_sim/tether.py).

sim.launch.py starts this node only when the scenario has a tether: section.
It follows the UGV's and UAV's /pose and steps the model at sim.ground_rate.

Topics:
  /winch/cmd_rate       std_msgs/Float64  in, m/s: + pays out, - reels in. Held until
                                          the next msg, clamped to tether.max_reel_speed
  /winch/length         std_msgs/Float64  L, tether paid out
  /tether/free_length   std_msgs/Float64  l, from the UAV ring to the object
  /tether/taut          std_msgs/Bool
  /tether/stuck         std_msgs/Bool     object jammed against an obstacle
  /tether/object_pose   geometry_msgs/PoseStamped  object center
  /tether/goal          geometry_msgs/PoseStamped  goal center (latched)
  /tether/goal_reached  std_msgs/Bool     object center inside the goal circle
  /hero_sim/tether      visualization_msgs/MarkerArray
"""
from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64
from visualization_msgs.msg import Marker, MarkerArray

from hero_sim import config, tether

FRAME = 'world'
MARKER_TOPIC = '/hero_sim/tether'
OBSTACLE_HEIGHT = 0.6  # m, drawn only: the tether passes over obstacles


def _color(m, r, g, b, a=1.0):
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a


class TetherSim(Node):

    def __init__(self):
        super().__init__('tether_sim')
        self.declare_parameter('config_path', '')
        scenario = config.load(config.resolve_config(self.get_parameter('config_path').value))
        if scenario.tether is None:
            raise RuntimeError('scenario has no tether: section')
        t = self._t = scenario.tether
        self._dt = 1.0 / scenario.sim['ground_rate']
        # Winch and pillars never change: resend them once a second, not every tick.
        self._static_every = max(1, round(scenario.sim['ground_rate']))
        self._ticks = 0

        self._attach_z = t.object_size / 2.0
        self._length = None  # set from the first poses: taut + t.slack
        self._state = None
        self._reached = False
        self._rate = 0.0
        self._ugv = self._uav = None

        self.create_subscription(PoseStamped, f'/{t.ugv}/pose', self._on_ugv, 10)
        self.create_subscription(PoseStamped, f'/{t.uav}/pose', self._on_uav, 10)
        self.create_subscription(Float64, '/winch/cmd_rate', self._on_rate, 10)

        self._length_pub = self.create_publisher(Float64, '/winch/length', 10)
        self._free_pub = self.create_publisher(Float64, '/tether/free_length', 10)
        self._taut_pub = self.create_publisher(Bool, '/tether/taut', 10)
        self._stuck_pub = self.create_publisher(Bool, '/tether/stuck', 10)
        self._obj_pub = self.create_publisher(PoseStamped, '/tether/object_pose', 10)
        self._reached_pub = self.create_publisher(Bool, '/tether/goal_reached', 10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        goal = self._pose(t.goal[0], t.goal[1], 0.0)
        self.create_publisher(PoseStamped, '/tether/goal', latched).publish(goal)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'tether: winch {t.winch[:2]} -> {t.ugv} -> {t.uav} -> object {t.object}, '
            f'goal {t.goal[:2]} r={t.goal[2]:g}, {len(t.obstacles)} obstacle(s); '
            f'waiting for {t.ugv} and {t.uav} poses')

    def _on_ugv(self, msg):
        p = msg.pose.position
        self._ugv = (p.x, p.y, p.z)

    def _on_uav(self, msg):
        p = msg.pose.position
        self._uav = (p.x, p.y, p.z)

    def _on_rate(self, msg):
        limit = self._t.max_reel_speed
        self._rate = max(-limit, min(limit, msg.data))

    def _pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        return pose

    def _tick(self):
        if self._ugv is None or self._uav is None:
            return
        t = self._t
        if self._state is None:
            self._length = tether.initial_length(t.winch, self._ugv, self._uav, t.object,
                                                  self._attach_z, t.slack)
            self.get_logger().info(f'tether ready: L = {self._length:.2f} m')
        obj = t.object if self._state is None else self._state.obj

        was_stuck = self._state is not None and self._state.stuck
        s = self._state = tether.step(
            max(self._length + self._rate * self._dt, 0.0), t.winch, self._ugv, self._uav,
            obj, self._attach_z, t.obstacles, t.object_radius)
        self._length = s.length
        if s.stuck and not was_stuck:
            self.get_logger().info('object jammed against an obstacle')

        reached = tether.reached(s.obj, t.goal)
        if reached and not self._reached:
            self.get_logger().info('object reached the goal!')
        self._reached = reached

        self._length_pub.publish(Float64(data=s.length))
        self._free_pub.publish(Float64(data=s.free))
        self._taut_pub.publish(Bool(data=s.taut))
        self._stuck_pub.publish(Bool(data=s.stuck))
        self._obj_pub.publish(self._pose(*s.obj, self._attach_z))
        self._reached_pub.publish(Bool(data=reached))

        markers = self._dynamic_markers()
        if self._ticks % self._static_every == 0:
            markers += self._static_markers()
        self._ticks += 1
        self._marker_pub.publish(MarkerArray(markers=markers))

    def _marker(self, ns, marker_id, marker_type, x=0.0, y=0.0, z=0.0):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        return m

    def _static_markers(self):
        t = self._t
        wx, wy, wz = t.winch
        base = self._marker('winch', 0, Marker.CUBE, wx, wy, wz / 2.0)
        base.scale.x, base.scale.y, base.scale.z = 0.4, 0.4, wz
        _color(base, 0.3, 0.33, 0.36)
        drum = self._marker('winch', 1, Marker.CYLINDER, wx, wy, wz)
        drum.scale.x, drum.scale.y, drum.scale.z = 0.3, 0.3, 0.12
        _color(drum, 0.91, 0.35, 0.05)

        pillars = []
        for i, (x, y, r) in enumerate(t.obstacles):
            m = self._marker('obstacles', i, Marker.CYLINDER, x, y, OBSTACLE_HEIGHT / 2.0)
            m.scale.x = m.scale.y = 2.0 * r
            m.scale.z = OBSTACLE_HEIGHT
            _color(m, 0.55, 0.45, 0.4)
            pillars.append(m)
        return [base, drum, *pillars]

    def _dynamic_markers(self):
        t = self._t
        s = self._state
        wx, wy, wz = t.winch
        gx, gy, gr = t.goal
        ox, oy = s.obj
        size = t.object_size

        line = self._marker('tether', 0, Marker.LINE_STRIP)
        line.scale.x = 0.03
        line.points = [Point(x=p[0], y=p[1], z=p[2])
                       for p in (t.winch, self._ugv, self._uav, (ox, oy, self._attach_z))]
        if s.taut:
            _color(line, 0.91, 0.35, 0.05)
        else:
            _color(line, 0.98, 0.7, 0.45, 0.8)

        pulleys = []
        for i, p in enumerate((self._ugv, self._uav)):
            m = self._marker('pulleys', i, Marker.SPHERE, *p)
            m.scale.x = m.scale.y = m.scale.z = 0.08
            _color(m, 1.0, 1.0, 1.0)
            pulleys.append(m)

        box = self._marker('object', 0, Marker.CUBE, ox, oy, size / 2.0)
        box.scale.x = box.scale.y = box.scale.z = size
        _color(box, 0.69, 0.54, 0.41)

        goal = self._marker('goal', 0, Marker.CYLINDER, gx, gy, 0.01)
        goal.scale.x = goal.scale.y = 2.0 * gr
        goal.scale.z = 0.02
        if self._reached:
            _color(goal, 0.25, 0.75, 0.34, 0.9)
        else:
            _color(goal, 0.17, 0.54, 0.24, 0.35)
        label = self._marker('goal', 1, Marker.TEXT_VIEW_FACING, gx, gy, 0.5)
        label.scale.z = 0.25
        label.text = 'GOAL REACHED!' if self._reached else 'GOAL'
        _color(label, 0.4, 0.9, 0.45)

        status = self._marker('status', 0, Marker.TEXT_VIEW_FACING, wx, wy, wz + 0.5)
        status.scale.z = 0.18
        state = 'STUCK' if s.stuck else 'TAUT' if s.taut else 'slack'
        status.text = f'L = {s.length:.2f} m   l = {s.free:.2f} m   {state}'
        if s.stuck:
            _color(status, 1.0, 0.4, 0.4)
        else:
            _color(status, 1.0, 1.0, 1.0)

        return [line, *pulleys, box, goal, label, status]


def main(args=None):
    rclpy.init(args=args)
    node = TetherSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
