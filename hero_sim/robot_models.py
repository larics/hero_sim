"""Robot meshes for RViz, attached to each robot's TF frame.

Publishes one MarkerArray, once, on /hero_sim/robot_models with transient-local
durability (late subscribers such as a restarted RViz still get it). Every
marker is frame-locked to its robot's frame, so RViz moves the meshes with TF
by itself: nothing is republished while the sim runs.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from hero_sim import config

TOPIC = '/hero_sim/robot_models'


def mesh_marker(frame, model, marker_id, z=0.0):
    """Mesh marker for `model` drawn at height `z` in TF frame `frame`."""
    m = Marker()
    m.header.frame_id = frame  # stamp 0: RViz uses the latest transform
    m.ns = model
    m.id = marker_id
    m.type = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.mesh_resource = f'package://hero_sim/models/{config.MODEL_MESHES[model]}'
    m.mesh_use_embedded_materials = True  # color left at 0,0,0,0 so RViz doesn't tint
    m.frame_locked = True
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    return m


def scenario_markers(scenario):
    markers = [mesh_marker(cf.name, 'crazyflie', i)
               for i, cf in enumerate(scenario.crazyflies)]
    # A ground robot's frame is at its marker height: drop the mesh back to the floor.
    markers += [mesh_marker(r.name, r.model, i, -r.z)
                for i, r in enumerate(scenario.ground_robots) if r.model]
    return markers


class RobotModels(Node):

    def __init__(self):
        super().__init__('robot_models')
        self.declare_parameter('config_path', '')
        scenario = config.load(config.resolve_config(self.get_parameter('config_path').value))
        markers = scenario_markers(scenario)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, TOPIC, qos)
        self._pub.publish(MarkerArray(markers=markers))
        self.get_logger().info(
            f'{len(markers)} robot meshes on {TOPIC}: '
            f'{", ".join(f"{m.header.frame_id}({m.ns})" for m in markers) or "none"}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotModels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
