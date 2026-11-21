"""Meshes for RViz: the robots, attached to their TF frames, and the map's own 3D model.

Publishes each MarkerArray once, with transient-local durability (late
subscribers such as a restarted RViz still get it):

  /hero_sim/robot_models  with sim.models. Every marker is frame-locked to
                          its robot's frame, so RViz moves the meshes with TF
                          by itself: nothing is republished while the sim runs
  /hero_sim/map_model     with sim.map_model: the map's furniture or its own
                          mesh (gazebo.map_visuals), in 'world', in their own
                          colors. Drawn over the octomap, which stays
"""
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from hero_sim import config, gazebo

TOPIC = '/hero_sim/robot_models'
MAP_TOPIC = '/hero_sim/map_model'


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


_SHAPES = {'box': Marker.CUBE, 'cylinder': Marker.CYLINDER, 'sphere': Marker.SPHERE}


def map_markers(visuals):
    """Markers in 'world' for gazebo.Visuals: meshes by file, shapes by type."""
    markers = []
    for i, v in enumerate(visuals):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = 'map_model'
        m.id = i
        m.action = Marker.ADD
        if v.shape == 'mesh':
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = Path(v.mesh).resolve().as_uri()
            m.mesh_use_embedded_materials = v.color is None
        else:
            m.type = _SHAPES[v.shape]
        if v.color is not None:
            m.color.r, m.color.g, m.color.b, m.color.a = v.color
        m.pose.position.x, m.pose.position.y, m.pose.position.z = v.position
        o = m.pose.orientation
        o.x, o.y, o.z, o.w = v.orientation
        m.scale.x, m.scale.y, m.scale.z = v.size
        markers.append(m)
    return markers


class RobotModels(Node):

    def __init__(self):
        super().__init__('robot_models')
        self.declare_parameter('config_path', '')
        scenario = config.load(config.resolve_config(self.get_parameter('config_path').value))
        sim = scenario.sim
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        if sim['models']:
            markers = scenario_markers(scenario)
            self._pub = self.create_publisher(MarkerArray, TOPIC, qos)
            self._pub.publish(MarkerArray(markers=markers))
            self.get_logger().info(
                f'{len(markers)} robot meshes on {TOPIC}: '
                f'{", ".join(f"{m.header.frame_id}({m.ns})" for m in markers) or "none"}')

        if sim['map_model']:
            visuals = gazebo.map_visuals(sim)
            self._map_pub = self.create_publisher(MarkerArray, MAP_TOPIC, qos)
            self._map_pub.publish(MarkerArray(markers=map_markers(visuals)))
            if visuals:
                self.get_logger().info(f'map model on {MAP_TOPIC}: {len(visuals)} pieces')
            else:
                self.get_logger().warn(f'sim.map_model: map {sim["map"]!r} has no 3D model of '
                                       'its own (a scenery .sdf or an octomap .stl); RViz shows '
                                       'its octomap only')


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
