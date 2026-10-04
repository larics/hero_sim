"""Two-claw grippers on top of the Crazyflies, drawn in RViz (visualization only).

A Crazyflie with ``gripper: true`` in the scenario starts with an open gripper.
sim_attach drives the rest: locking a UAV in place (AttachAgent with an empty
parent_id: hanging from a branch) shuts its gripper, and detaching it from
that lock leaves the gripper on the branch, where the UAV hung, while the UAV
flies off without one. /sim/<cf>/add_gripper (std_srvs/Trigger) fits a fresh,
open gripper on a Crazyflie that has none.

Each gripper is one TRIANGLE_LIST marker on /hero_sim/grippers (transient
local, so a restarted RViz still gets them). A fitted gripper is frame-locked
to its Crazyflie's TF frame, so RViz moves it by itself; one left on a branch
sits in the world frame. The whole set is republished only when a gripper
changes, never per tick.
"""
from functools import partial
import math
import threading

from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from hero_sim.kinematics import yaw_to_quat

TOPIC = '/hero_sim/grippers'
OPEN, CLOSED = 'open', 'closed'

# Geometry (m) in the Crazyflie's frame: x forward, z up. The claws face each
# other across y, so they shut around a bar lying along x, the way a hanging
# UAV faces along its branch. Shut, they wrap a 3 cm bar 8 cm above the
# Crazyflie's origin, which is where hero_mrs_control_stack's visualizer
# draws a branch above the hang pose.
BAR_Z = 0.08
PLATE_TOP = 0.037          # mounting plate, on the Crazyflie's top deck
CLAW_RADIUS = 0.020        # shut claws' centerline around the bar
CLAW_PIVOT_Y = 0.016
CLAW_WIDTH = 0.016         # along x
CLAW_THICKNESS = 0.004
OPEN_ANGLE = math.radians(35.0)  # each claw swings out this far about its pivot

BODY_RGB = (0.2, 0.2, 0.2)
CLAW_RGB = (1.0, 0.6, 0.1)

# Unit-cube faces as corner indices (i + 2j + 4k for corner (+-1, +-1, +-1)),
# counter-clockwise seen from outside, so RViz's normals face out.
_CUBE_FACES = ((0, 4, 6, 2), (1, 3, 7, 5), (0, 1, 5, 4),
               (2, 6, 7, 3), (0, 2, 3, 1), (4, 5, 7, 6))


def box(center, ax, ay, az):
    """12 triangles of the box center +- ax +- ay +- az (half-axes, right-handed)."""
    corners = [tuple(c + i * a + j * b + k * d for c, a, b, d in zip(center, ax, ay, az))
               for k in (-1, 1) for j in (-1, 1) for i in (-1, 1)]
    tris = []
    for a, b, c, d in _CUBE_FACES:
        tris += [corners[a], corners[b], corners[c], corners[a], corners[c], corners[d]]
    return tris


def _segment(a, b):
    """Claw segment from a to b, both (y, z): a bar, overlapping its neighbours at the joints."""
    (ya, za), (yb, zb) = a, b
    length = math.hypot(yb - ya, zb - za)
    uy, uz = (yb - ya) / length, (zb - za) / length
    half = (length + CLAW_THICKNESS) / 2
    t = CLAW_THICKNESS / 2
    return box((0.0, (ya + yb) / 2, (za + zb) / 2),
               (CLAW_WIDTH / 2, 0.0, 0.0), (0.0, uy * half, uz * half), (0.0, -uz * t, uy * t))


def _claw(angle):
    """Centerline (y, z) of the +y claw swung out by ``angle``: up from its pivot, curled over."""
    shut = [(CLAW_PIVOT_Y, PLATE_TOP)] + [
        (CLAW_RADIUS * math.cos(a), BAR_Z + CLAW_RADIUS * math.sin(a))
        for a in map(math.radians, (0.0, 45.0, 80.0))]
    (py, pz), c, s = shut[0], math.cos(angle), math.sin(angle)
    return [(py + (y - py) * c + (z - pz) * s, pz - (y - py) * s + (z - pz) * c)
            for y, z in shut]


def shape(angle):
    """(points, colors) for a TRIANGLE_LIST of the gripper, claws swung out by ``angle``."""
    parts = [(box((0.0, 0.0, PLATE_TOP - 0.003), (0.012, 0, 0), (0, 0.020, 0), (0, 0, 0.003)),
              BODY_RGB),
             (box((0.0, 0.0, PLATE_TOP + 0.009), (0.005, 0, 0), (0, 0.005, 0), (0, 0, 0.009)),
              BODY_RGB)]  # plate, and the trigger post the bar presses on
    for side in (1, -1):
        claw = [(side * y, z) for y, z in _claw(angle)]
        parts += [(_segment(a, b), CLAW_RGB) for a, b in zip(claw, claw[1:])]
    points, colors = [], []
    for tris, (r, g, b) in parts:
        points += [Point(x=x, y=y, z=z) for x, y, z in tris]
        colors += [ColorRGBA(r=r, g=g, b=b, a=1.0)] * len(tris)
    return points, colors


_SHAPES = {OPEN: shape(OPEN_ANGLE), CLOSED: shape(0.0)}


def gripper_marker(frame, ns, marker_id, state):
    m = Marker()
    m.header.frame_id = frame  # stamp 0: RViz uses the latest transform
    m.ns = ns
    m.id = marker_id
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color.r = m.color.g = m.color.b = m.color.a = 1.0  # RViz scales vertex alpha by color.a
    m.points, m.colors = _SHAPES[state]
    return m


class Grippers:
    """Every Crazyflie's gripper (open, closed or none), the ones left on branches,
    their RViz markers and the /sim/<cf>/add_gripper services."""

    def __init__(self, node, crazyflies):
        self._node = node
        self._names = [cf.name for cf in crazyflies]
        self._fitted = {cf.name: OPEN if cf.gripper else None for cf in crazyflies}
        self._left = []  # world (x, y, z, yaw) of grippers left on branches
        self._lock = threading.Lock()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = node.create_publisher(MarkerArray, TOPIC, qos)
        for name in self._names:
            node.create_service(Trigger, f'/sim/{name}/add_gripper', partial(self._on_add, name))
        with self._lock:
            self._publish()

    def state(self, cf):
        """OPEN, CLOSED, or None for no gripper."""
        return self._fitted.get(cf)

    @property
    def left(self):
        return list(self._left)

    def close(self, cf):
        """``cf`` locked in place (hanging from a branch): an open gripper shuts."""
        with self._lock:
            if self._fitted.get(cf) != OPEN:
                return False
            self._fitted[cf] = CLOSED
            self._publish()
        return True

    def release(self, cf, x, y, z, yaw):
        """``cf`` left its lock at world (x, y, z, yaw): a shut gripper stays there."""
        with self._lock:
            if self._fitted.get(cf) != CLOSED:
                return False
            self._fitted[cf] = None
            self._left.append((x, y, z, yaw))
            self._publish()
        return True

    def _on_add(self, cf, request, response):
        with self._lock:
            if self._fitted[cf] is not None:
                response.success = True  # idempotent
                response.message = f'{cf} already has a gripper (no-op)'
                return response
            self._fitted[cf] = OPEN
            self._publish()
        self._node.get_logger().info(f'GRIPPER fitted on {cf}')
        response.success = True
        response.message = f'fitted a fresh gripper on {cf}'
        return response

    def _publish(self):
        """Republish every gripper; call with _lock held."""
        markers = [Marker(action=Marker.DELETEALL)]
        for i, name in enumerate(self._names):
            if self._fitted[name] is not None:
                m = gripper_marker(name, 'gripper', i, self._fitted[name])
                m.frame_locked = True
                markers.append(m)
        for i, (x, y, z, yaw) in enumerate(self._left):
            m = gripper_marker('world', 'gripper_left', i, CLOSED)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            o = m.pose.orientation
            o.x, o.y, o.z, o.w = yaw_to_quat(yaw)
            markers.append(m)
        self._pub.publish(MarkerArray(markers=markers))
