"""Sim backend for the AttachAgent hook (dock ride-along, branch hang).

Serves /sim/attach_agent (hero_mrs_msgs/srv/AttachAgent). When a UAV docks on
a ground robot the coordinator calls attach=true; on undock, attach=false.

On attach we freeze the child's pose in the parent's body frame once, then a
timer drives the child to (parent pose (+) that offset) at follow_rate Hz by
publishing /<child>/cmd_position, which SimCrazyflie applies as an
instantaneous setpoint. Detach stops driving it. Several couples can run at
once, keyed by child_id.

An empty parent_id locks the child where it is: its current world pose is
frozen and held until detach. hero_mrs_control_stack's gripper_perch_node uses
this for a UAV hanging from a branch perch — the sim has no gripper physics,
so without the lock a disarmed UAV would not stay on the branch.

The lock also drives the drawn gripper (hero_sim/gripper.py): locking shuts
the UAV's gripper, and detaching leaves it on the branch at the lock pose.
This node serves /sim/<cf>/add_gripper to fit a fresh one.

hero_mrs_msgs (from hero_mrs_control_stack) is an optional dependency of
hero_sim; sim.launch.py only starts this node when it is installed.
"""
import math
import threading
import time

from crazyflie_interfaces.msg import Position
from geometry_msgs.msg import PoseStamped
from hero_mrs_msgs.srv import AttachAgent
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hero_sim import config
from hero_sim.gripper import Grippers
from hero_sim.kinematics import quat_to_yaw, wrap_angle


def _planar(pose_msg):
    p, q = pose_msg.pose.position, pose_msg.pose.orientation
    return p.x, p.y, p.z, quat_to_yaw(q.x, q.y, q.z, q.w)


class _Couple:
    """Frozen child pose in the parent's body frame, plus the child's setpoint publisher.

    ``parent_id`` None is a lock in place: the frozen pose is in the world frame.
    """

    __slots__ = ('parent_id', 'local_x', 'local_y', 'dz', 'dyaw', 'publisher')

    def __init__(self, parent_id, local_x, local_y, dz, dyaw, publisher):
        self.parent_id = parent_id
        self.local_x = local_x
        self.local_y = local_y
        self.dz = dz
        self.dyaw = dyaw
        self.publisher = publisher


class SimAttachNode(Node):

    def __init__(self):
        super().__init__('sim_attach_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('follow_rate', 50.0)
        # Wait budget (s) for an agent's first pose sample on attach.
        self.declare_parameter('pose_wait_timeout', 3.0)
        follow_rate = float(self.get_parameter('follow_rate').value)
        self._pose_wait_timeout = float(self.get_parameter('pose_wait_timeout').value)

        # Reentrant so pose subscriptions keep updating while the service
        # callback blocks waiting for a first pose sample.
        self._cbg = ReentrantCallbackGroup()

        self._couples = {}  # child_id -> _Couple, guarded by _lock
        self._lock = threading.Lock()
        self._poses = {}  # agent_id -> latest PoseStamped
        self._pose_subs = {}

        self._qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_service(AttachAgent, '/sim/attach_agent', self._on_request,
                            callback_group=self._cbg)
        self.create_timer(1.0 / follow_rate if follow_rate > 0 else 0.02,
                          self._follow_tick, callback_group=self._cbg)

        scenario = config.load(config.resolve_config(self.get_parameter('config_path').value))
        # Drawn only alongside the robot meshes (robot_models' launch condition).
        self._grippers = Grippers(self, scenario.crazyflies,
                                  draw=scenario.sim['rviz'] and scenario.sim['models'])

        fitted = [cf.name for cf in scenario.crazyflies if cf.gripper]
        self.get_logger().info(
            f'sim_attach up: service=/sim/attach_agent follow_rate={follow_rate:g}Hz '
            f'grippers on: {", ".join(fitted) or "none"}')

    # ------------------------------------------------------------------ poses

    def _ensure_pose_sub(self, agent_id):
        if agent_id in self._pose_subs:
            return
        self._pose_subs[agent_id] = self.create_subscription(
            PoseStamped, f'/{agent_id}/pose',
            lambda msg, a=agent_id: self._poses.__setitem__(a, msg),
            self._qos, callback_group=self._cbg)

    def _wait_for_pose(self, agent_id):
        """Latest pose for an agent, waiting briefly for the first sample. None on timeout."""
        self._ensure_pose_sub(agent_id)
        deadline = time.monotonic() + self._pose_wait_timeout
        while agent_id not in self._poses:
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)  # subscriptions run on the executor's other threads
        return self._poses[agent_id]

    # ---------------------------------------------------------------- service

    def _on_request(self, request, response):
        if not request.child_id:
            response.success = False
            response.message = 'child_id is empty'
            return response
        if request.attach:
            return self._do_attach(request.child_id, request.parent_id, response)
        return self._do_detach(request.child_id, response)

    def _do_attach(self, child, parent, response):
        if not parent:
            return self._do_lock(child, response)
        with self._lock:
            already = self._couples.get(child)
        if already is not None:
            self.get_logger().warn(
                f'{child} already attached to {already.parent_id}; re-snapshotting against {parent}')

        parent_pose = self._wait_for_pose(parent)
        child_pose = self._wait_for_pose(child)
        for agent, pose in ((parent, parent_pose), (child, child_pose)):
            if pose is None:
                response.success = False
                response.message = f"no pose for '{agent}' on /{agent}/pose"
                return response

        px, py, pz, pyaw = _planar(parent_pose)
        cx, cy, cz, cyaw = _planar(child_pose)

        # Child pose expressed in the parent's body frame.
        dx, dy = cx - px, cy - py
        c, s = math.cos(pyaw), math.sin(pyaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        publisher = self.create_publisher(Position, f'/{child}/cmd_position', self._qos)
        with self._lock:
            old = self._couples.get(child)
            self._couples[child] = _Couple(parent, local_x, local_y, cz - pz,
                                           wrap_angle(cyaw - pyaw), publisher)
            if old is not None:
                self.destroy_publisher(old.publisher)

        self.get_logger().info(
            f'ATTACH {child} -> {parent} offset(local)=({local_x:.3f}, {local_y:.3f}, '
            f'{cz - pz:.3f}) dyaw={wrap_angle(cyaw - pyaw):.3f}')
        response.success = True
        response.message = f'attached {child} to {parent}'
        return response

    def _do_lock(self, child, response):
        """Freeze ``child`` at its current world pose (attach with no parent)."""
        child_pose = self._wait_for_pose(child)
        if child_pose is None:
            response.success = False
            response.message = f"no pose for '{child}' on /{child}/pose"
            return response
        cx, cy, cz, cyaw = _planar(child_pose)
        publisher = self.create_publisher(Position, f'/{child}/cmd_position', self._qos)
        with self._lock:
            old = self._couples.get(child)
            # parent_id None: the "offset" is the world pose itself.
            self._couples[child] = _Couple(None, cx, cy, cz, cyaw, publisher)
            if old is not None:
                self.destroy_publisher(old.publisher)
        shut = self._grippers.close(child)
        self.get_logger().info(
            f'LOCK {child} at ({cx:.3f}, {cy:.3f}, {cz:.3f}) yaw={cyaw:.3f}'
            + (', gripper shut' if shut else ''))
        response.success = True
        response.message = f'locked {child} in place'
        return response

    def _do_detach(self, child, response):
        with self._lock:
            couple = self._couples.pop(child, None)
            if couple is not None:
                self.destroy_publisher(couple.publisher)
        if couple is None:
            response.success = True  # idempotent
            response.message = f'{child} was not attached (no-op)'
            return response
        where = couple.parent_id or 'its lock'
        left = couple.parent_id is None and self._grippers.release(
            child, couple.local_x, couple.local_y, couple.dz, couple.dyaw)
        note = ', gripper left behind' if left else ''
        self.get_logger().info(f'DETACH {child} <- {where}{note}')
        response.success = True
        response.message = f'detached {child} from {where}{note}'
        return response

    # -------------------------------------------------------------- follow loop

    def _follow_tick(self):
        stamp = self.get_clock().now().to_msg()
        # Publish under the lock so attach/detach can't destroy a publisher mid-use.
        with self._lock:
            for couple in self._couples.values():
                if couple.parent_id is None:
                    px = py = pz = pyaw = 0.0   # locked: offset is the world pose
                else:
                    parent_pose = self._poses.get(couple.parent_id)
                    if parent_pose is None:
                        continue
                    px, py, pz, pyaw = _planar(parent_pose)
                c, s = math.cos(pyaw), math.sin(pyaw)

                msg = Position()
                msg.header.stamp = stamp
                msg.header.frame_id = 'world'
                msg.x = px + c * couple.local_x - s * couple.local_y
                msg.y = py + s * couple.local_x + c * couple.local_y
                msg.z = pz + couple.dz
                msg.yaw = wrap_angle(pyaw + couple.dyaw)
                couple.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimAttachNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
