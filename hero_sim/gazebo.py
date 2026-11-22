"""Gazebo as a camera renderer: the world it draws and the camera geometry. No ROS imports.

Gazebo simulates nothing here. Every robot is a static model that
gazebo_bridge moves to wherever the kinematic sim has it (gz-sim's
set_pose_vector service), so ground_robot_sim and crazyflie_server stay the
only source of motion: no wheels, gravity or contacts. Gazebo only renders,
so that a camera on a ground robot (config.Camera) sees the floor, the map
and the other robots.

write_world builds the world from a scenario:

  floor   a grey plane, FLOOR_Z under z = 0 so it doesn't flicker against a
          map's own floor
  map     the map's own Gazebo scenery if it has one: <map name>.sdf next to
          its .bt or .yaml, whose models and lights go in as they are
          (maps/small_house/, see map_scenery). Else a mesh: an octomap's
          own .stl next to it (maps/icuas26_1.stl), or else its occupied
          voxels; an image map's walls, sim.wall_height tall. Both
          as write_obj meshes. Pillars (a scenario's obstacles:) are cylinders
  robots  their RViz meshes (config.MODEL_MESHES), each in a model named
          after the robot. A model's origin is the robot's TF frame, at its
          pose height as in RViz, and a camera sits there at its Camera's x,
          y, z and pitch. Robots without a model are invisible, as in RViz

The world loads gz-sim's Physics system because that is what applies
set_pose_vector's poses, static models included. With no gravity, joints or
collisions it moves nothing by itself.

map_visuals reads the same map model for RViz (sim.map_model), without Gazebo.
"""
import math
from pathlib import Path
from typing import NamedTuple
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape, quoteattr

import numpy as np
import yaml

from hero_sim import config, obstacles

WORLD = 'hero_sim'  # services and topics are under /world/hero_sim/
SET_POSE_SERVICE = f'/world/{WORLD}/set_pose_vector'
STEP = 0.01  # s, physics step: a pose is applied within one step
FLOOR_Z = -0.01  # m
FLOOR_SIZE = 100.0  # m, edge of the floor plane
FLOOR_COLOR = (0.5, 0.5, 0.5)
MAP_COLOR = (0.6, 0.57, 0.52)
PILLAR_COLOR = (0.55, 0.45, 0.4)  # as in RViz (ground_robots._publish_pillars)
CAMERA_CLIP = (0.05, 50.0)  # m, nearest and farthest a camera sees
SCENERY_TAGS = ('model', 'light', 'include')  # what a map's scenery puts in the world

# camera_link (x forward, z up) -> camera_optical (z forward, x right, y down): x, y, z, w
OPTICAL_ROTATION = (-0.5, 0.5, -0.5, 0.5)


def camera_topic(robot):
    """Gazebo topic of a robot's camera images (gz-transport, not ROS)."""
    return f'/hero_sim/{robot}/camera'


def camera_frames(robot):
    """TF frames of a robot's camera: (camera_link, camera_optical)."""
    return f'{robot}/camera_link', f'{robot}/camera_optical'


def intrinsics(camera):
    """fx, fy, cx, cy (px) of a config.Camera: square pixels, centered, Gazebo's pinhole model."""
    fx = camera.width / 2.0 / math.tan(camera.hfov / 2.0)
    return fx, fx, camera.width / 2.0, camera.height / 2.0


# ------------------------------------------------------------------ meshes

# For each axis a, the two in-plane axes (b, c) with e_b x e_c = e_a: a quad
# p, p + e_b, p + e_b + e_c, p + e_c then faces +a.
_PLANES = ((0, 1, 2), (1, 2, 0), (2, 0, 1))


def grid_quads(filled):
    """The outer faces of a 3D grid's filled cells, as quads in cell units.

    filled: bool array [x, y, z]. A face between two filled cells is inside
    and left out; faces next to each other along a row are joined into one
    long quad, so flat floors and walls stay cheap. Returns (corners,
    normals): float arrays (n, 4, 3), wound counter-clockwise seen from
    outside, and (n, 3) unit normals.
    """
    padded = np.pad(np.asarray(filled, bool), 1)
    corners, normals = [], []
    for a, b, c in _PLANES:
        e_a, e_b, e_c = np.eye(3)[a], np.eye(3)[b], np.eye(3)[c]
        for sign in (1, -1):
            exposed = padded & ~np.roll(padded, -sign, axis=a)
            # Runs along b: with b last, starts and ends come in the same order.
            runs = np.moveaxis(exposed, b, -1)
            starts = np.argwhere(runs & ~np.roll(runs, 1, axis=-1))
            ends = np.argwhere(runs & ~np.roll(runs, -1, axis=-1))
            if not len(starts):
                continue
            others = [ax for ax in range(3) if ax != b]
            cell = np.empty((len(starts), 3))
            cell[:, others] = starts[:, :2]
            cell[:, b] = starts[:, 2]
            cell -= 1  # the padding
            length = (ends[:, 2] - starts[:, 2] + 1)[:, None]
            p = cell + (e_a if sign > 0 else 0.0)
            quad = [p, p + length * e_b, p + length * e_b + e_c, p + e_c]
            if sign < 0:
                quad = [quad[0], quad[3], quad[2], quad[1]]
            corners.append(np.stack(quad, axis=1))
            normals.append(np.tile(sign * e_a, (len(starts), 1)))
    if not corners:
        return np.zeros((0, 4, 3)), np.zeros((0, 3))
    return np.concatenate(corners), np.concatenate(normals)


def write_obj(path, filled, origin, cell):
    """Write a 3D grid's filled cells as a Wavefront OBJ mesh (grid_quads, two triangles each).

    Cell (i, j, k) of filled [x, y, z] spans origin + (i, j, k) * cell to
    one cell further, in m. cell is the edge length along x, y and z.
    """
    corners, normals = grid_quads(filled)
    verts = np.asarray(origin, float) + corners.reshape(-1, 3) * np.asarray(cell, float)
    first = 1 + 4 * np.arange(len(corners))[:, None]  # OBJ counts from 1
    normal_ids = 1 + np.arange(len(corners))[:, None]
    tris = np.hstack([first, normal_ids, first + 1, normal_ids, first + 2, normal_ids,
                      first, normal_ids, first + 2, normal_ids, first + 3, normal_ids])
    with open(path, 'w') as f:
        f.write('# written by hero_sim/gazebo.py\n')
        np.savetxt(f, verts, fmt='v %.5f %.5f %.5f')
        np.savetxt(f, normals, fmt='vn %g %g %g')
        np.savetxt(f, tris.reshape(-1, 6), fmt='f %d//%d %d//%d %d//%d')
    return str(path)


def octomap_grid(path):
    """An octomap .bt's occupied voxels as (filled [x, y, z], origin, cell), for write_obj."""
    res, boxes = obstacles.read_bt(path)
    if not len(boxes):
        return np.zeros((0, 0, 0), bool), (0.0, 0.0, 0.0), (res,) * 3
    lo = boxes[:, :3].min(axis=0)
    hi = (boxes[:, :3] + boxes[:, 3:]).max(axis=0)
    filled = np.zeros(tuple(hi - lo), bool)
    for kx, ky, kz, size in boxes - (*lo, 0):
        filled[kx:kx + size, ky:ky + size, kz:kz + size] = True
    origin = (lo - obstacles._KEY_ORIGIN) * res  # octomap keys -> m, as obstacles.read_bt
    return filled, tuple(float(v) for v in origin), (res,) * 3


def map_mesh(sim, obj_path):
    """The mesh file Gazebo draws for sim.map, or None for no map.

    An octomap's own mesh when one sits next to it (same name, .stl), else
    one built from its voxels, or from an image map's walls, written to
    obj_path.
    """
    map_file = config.resolve_map(sim['map'])
    if map_file is None:
        return None
    if config.is_image_map(map_file):
        occupied, x0, y0, res = obstacles.read_image_map(map_file)
        filled = occupied.T[:, :, None]  # [row = y, col = x] -> [x, y, z]
        origin = (x0, y0, 0.0)
        cell = (res, res, sim.get('wall_height', obstacles.MAP_WALL_HEIGHT))
    else:
        mesh = Path(map_file).with_suffix('.stl')
        if mesh.is_file():
            return str(mesh)
        filled, origin, cell = octomap_grid(map_file)
    if not filled.any():
        return None
    Path(obj_path).parent.mkdir(parents=True, exist_ok=True)
    return write_obj(obj_path, filled, origin, cell)


def map_scenery(sim):
    """sim.map's own Gazebo scenery, or None: <map name>.sdf next to its .bt or .yaml.

    An SDF world whose models, lights and includes (SCENERY_TAGS) Gazebo
    draws in place of a mesh made from the map, and with its own lights
    instead of hero_sim's sun. Its model:// URIs are looked up in a models/
    folder next to it (resource_paths). The map itself still decides what
    RViz shows and what robots collide with, so the two must line up.
    """
    map_file = config.resolve_map(sim['map'])
    if map_file is None:
        return None
    sdf = Path(map_file).with_suffix('.sdf')
    return str(sdf) if sdf.is_file() else None


def resource_paths(sim):
    """Folders Gazebo must search for model:// URIs (GZ_SIM_RESOURCE_PATH): the scenery's models/."""
    scenery = map_scenery(sim)
    models = Path(scenery).parent / 'models' if scenery else None
    return [str(models)] if models is not None and models.is_dir() else []


def scenery_elements(path):
    """The SCENERY_TAGS elements of an SDF world file, as SDF text for world_sdf."""
    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise ValueError(f'{path}: no <world> in it')
    return ''.join(f'    {ET.tostring(el, encoding="unicode").strip()}\n'
                   for el in world if el.tag in SCENERY_TAGS)


# ------------------------------------------------------- the map's 3D model for RViz

class Visual(NamedTuple):
    """One piece of a map's 3D model, placed in the world frame (robot_models draws it in RViz)."""
    name: str          # the scenery model it belongs to
    shape: str         # 'mesh', 'box', 'cylinder' or 'sphere'
    size: tuple        # mesh: its scale; box: x, y, z; cylinder: 2 r, 2 r, length; sphere: 2 r x 3 (m)
    position: tuple    # x, y, z (m)
    orientation: tuple  # quaternion x, y, z, w
    mesh: str = ''     # mesh file, for shape 'mesh'
    color: tuple = None  # r, g, b, a; None: the mesh's own colors


def _floats(text, n):
    values = [float(v) for v in (text or '').split()]
    if len(values) != n:
        raise ValueError(f'expected {n} numbers, got {text!r}')
    return values


def _pose(el):
    """The 4x4 transform of an element's <pose> (x y z roll pitch yaw); identity without one."""
    m = np.eye(4)
    pose = el.find('pose')
    if pose is None:
        return m
    x, y, z, roll, pitch, yaw = _floats(pose.text, 6)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],  # Rz Ry Rx, as SDF
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = x, y, z
    return m


def _quaternion(r):
    """Quaternion x, y, z, w of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return x, y, z, w


def _model_dir(uri, models_dir):
    """The folder of an included model: model://<name> in models_dir, or a path."""
    if uri.startswith('model://'):
        return Path(models_dir) / uri[len('model://'):].strip('/')
    return Path(uri[len('file://'):] if uri.startswith('file://') else uri)


def _mesh_file(uri, models_dir):
    if uri.startswith('model://'):
        return Path(models_dir) / uri[len('model://'):]
    return Path(uri[len('file://'):] if uri.startswith('file://') else uri)


def _visual(el, m, name, models_dir):
    """A Visual from an SDF <visual> placed at transform m, or None for a shape RViz can't draw."""
    geometry = el.find('geometry')
    material = el.find('material/diffuse')
    color = tuple(_floats(material.text, 4)) if material is not None else (0.7, 0.7, 0.7, 1.0)
    where = dict(position=tuple(float(v) for v in m[:3, 3]), orientation=_quaternion(m[:3, :3]))
    mesh = geometry.find('mesh')
    if mesh is not None:
        scale = mesh.find('scale')
        return Visual(name, 'mesh', tuple(_floats(scale.text, 3)) if scale is not None else (1.0,) * 3,
                      mesh=str(_mesh_file(mesh.findtext('uri').strip(), models_dir)), **where)
    if geometry.find('box') is not None:
        return Visual(name, 'box', tuple(_floats(geometry.findtext('box/size'), 3)),
                      color=color, **where)
    if geometry.find('cylinder') is not None:
        r, length = float(geometry.findtext('cylinder/radius')), float(geometry.findtext('cylinder/length'))
        return Visual(name, 'cylinder', (2 * r, 2 * r, length), color=color, **where)
    if geometry.find('sphere') is not None:
        d = 2 * float(geometry.findtext('sphere/radius'))
        return Visual(name, 'sphere', (d, d, d), color=color, **where)
    return None  # a plane, heightmap, ...


def scenery_visuals(path):
    """Every visual of a scenery world (map_scenery), placed in the world frame.

    Follows its models, the models they include (model:// URIs in the models/
    folder next to it) and their links, composing their poses.
    """
    models_dir = Path(path).parent / 'models'
    visuals = []

    def walk(el, m, name):
        for child in el:
            if child.tag == 'model':
                walk(child, m @ _pose(child), child.get('name', name))
            elif child.tag == 'include':
                folder = _model_dir(child.findtext('uri').strip(), models_dir)
                sdf = folder / 'model.sdf'
                if (folder / 'model.config').is_file():  # it may name another file
                    sdf = folder / (ET.parse(folder / 'model.config').getroot().findtext('sdf')
                                    or 'model.sdf').strip()
                model = ET.parse(sdf).getroot().find('model')
                # An include's own <pose> places the model instead of the model's <pose>.
                pose = _pose(child) if child.find('pose') is not None else _pose(model)
                walk(model, m @ pose, child.findtext('name') or name)
            elif child.tag == 'link':
                for visual in child.findall('visual'):
                    v = _visual(visual, m @ _pose(child) @ _pose(visual), name, models_dir)
                    if v is not None:
                        visuals.append(v)

    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise ValueError(f'{path}: no <world> in it')
    walk(world, np.eye(4), '')
    return visuals


def map_visuals(sim):
    """sim.map's own 3D model, for RViz (sim.map_model), as Visuals; [] if it has none.

    A map with a scenery: its furniture (scenery_visuals). An octomap with its
    own mesh (maps/icuas26_1.stl): that mesh, in MAP_COLOR.
    """
    scenery = map_scenery(sim)
    if scenery:
        return scenery_visuals(scenery)
    map_file = config.resolve_map(sim['map'])
    if map_file is None or config.is_image_map(map_file):
        return []
    mesh = Path(map_file).with_suffix('.stl')
    if not mesh.is_file():
        return []
    return [Visual('map', 'mesh', (1.0, 1.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                   mesh=str(mesh), color=(*MAP_COLOR, 1.0))]


# ------------------------------------------------------------------- world

def _num(*values):
    return ' '.join(f'{v:.6g}' for v in values)


def _material(color):
    rgb = _num(*color)
    return f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>'


def _static(name, pose, visual):
    return (f'    <model name={quoteattr(name)}>\n'
            f'      <static>true</static>\n'
            f'      <pose>{_num(*pose)}</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="visual">{visual}</visual>\n'
            f'      </link>\n'
            f'    </model>\n')


def _mesh(path):
    return f'<geometry><mesh><uri>{escape(Path(path).resolve().as_uri())}</uri></mesh></geometry>'


def _camera_sensor(robot, camera):
    return (f'        <sensor name="camera" type="camera">\n'
            f'          <pose>{_num(camera.x, camera.y, camera.z, 0, camera.pitch, 0)}</pose>\n'
            f'          <always_on>true</always_on>\n'
            f'          <update_rate>{camera.rate:g}</update_rate>\n'
            f'          <topic>{escape(camera_topic(robot))}</topic>\n'
            f'          <camera>\n'
            f'            <horizontal_fov>{camera.hfov:g}</horizontal_fov>\n'
            f'            <image><width>{camera.width}</width><height>{camera.height}</height>'
            f'<format>R8G8B8</format></image>\n'
            f'            <clip><near>{CAMERA_CLIP[0]:g}</near><far>{CAMERA_CLIP[1]:g}</far></clip>\n'
            f'          </camera>\n'
            f'        </sensor>\n')


def _robot(name, pose, mesh=None, mesh_z=0.0, camera=None):
    """A robot's model: static, moved by gazebo_bridge. pose: x, y, z, yaw of its TF frame."""
    x, y, z, yaw = pose
    visual = (f'        <visual name="mesh"><pose>{_num(0, 0, mesh_z, 0, 0, 0)}</pose>'
              f'{_mesh(mesh)}</visual>\n' if mesh else '')
    sensor = _camera_sensor(name, camera) if camera else ''
    return (f'    <model name={quoteattr(name)}>\n'
            f'      <static>true</static>\n'
            f'      <pose>{_num(x, y, z, 0, 0, yaw)}</pose>\n'
            f'      <link name="base">\n{visual}{sensor}      </link>\n'
            f'    </model>\n')


_HEADER = '''<?xml version="1.0"?>
<!-- Written by hero_sim/gazebo.py from a scenario. Gazebo only renders the cameras:
     gazebo_bridge moves the robots to where the kinematic sim has them. -->
<sdf version="1.9">
  <world name="{world}">
    <physics name="hero_sim" type="dart">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <grid>false</grid>
    </scene>
'''

_SUN = '''    <light type="directional" name="hero_sim_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>
'''


def world_sdf(scenario, models_dir, map_mesh_file=None, scenery_file=None):
    """The scenario's world as SDF text. models_dir: where config.MODEL_MESHES are.

    scenery_file (map_scenery) brings the map's models and its own lights;
    without it the map is map_mesh_file, lit by hero_sim's sun.
    """
    models_dir = Path(models_dir)
    floor = (f'<geometry><plane><normal>0 0 1</normal><size>{_num(FLOOR_SIZE, FLOOR_SIZE)}</size>'
             f'</plane></geometry>{_material(FLOOR_COLOR)}')
    parts = [_HEADER.format(world=WORLD, step=STEP),
             _static('hero_sim_floor', (0, 0, FLOOR_Z, 0, 0, 0), floor)]
    if scenery_file:
        parts.append(scenery_elements(scenery_file))
    else:
        parts.append(_SUN)
        if map_mesh_file:
            parts.append(_static('hero_sim_map', (0,) * 6,
                                 _mesh(map_mesh_file) + _material(MAP_COLOR)))
    for i, p in enumerate(scenario.obstacles):
        cylinder = (f'<geometry><cylinder><radius>{p.radius:g}</radius><length>{p.height:g}'
                    f'</length></cylinder></geometry>{_material(PILLAR_COLOR)}')
        parts.append(_static(f'hero_sim_pillar_{i}', (p.x, p.y, p.height / 2.0, 0, 0, 0),
                             cylinder))
    for r in scenario.ground_robots:
        mesh = models_dir / config.MODEL_MESHES[r.model] if r.model else None
        # The frame is at the pose height: drop the mesh back to the floor, as robot_models does.
        parts.append(_robot(r.name, (r.x, r.y, r.z, r.yaw), mesh, -r.z, r.camera))
    for cf in scenario.crazyflies:
        parts.append(_robot(cf.name, (cf.x, cf.y, cf.z, 0.0),
                            models_dir / config.MODEL_MESHES['crazyflie']))
    parts.append('  </world>\n</sdf>\n')
    return ''.join(parts)


def write_world(path, scenario, models_dir):
    """Write the scenario's Gazebo world to `path` (dirs made as needed). Returns path.

    The map's scenery goes in if it has one (map_scenery). Else a map mesh
    built for it goes next to it, as <path stem>_map.obj. Gazebo then needs
    resource_paths in GZ_SIM_RESOURCE_PATH.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    scenery = map_scenery(scenario.sim)
    mesh = None if scenery else map_mesh(scenario.sim, path.with_name(f'{path.stem}_map.obj'))
    path.write_text(world_sdf(scenario, models_dir, mesh, scenery))
    return str(path)


# -------------------------------------------------------------------- RViz

def rviz_config(text, scenario):
    """An RViz config (text of config/sim.rviz) with an Image display for each camera added."""
    rviz = yaml.safe_load(text)
    displays = rviz['Visualization Manager']['Displays']
    for r in scenario.ground_robots:
        if r.camera:
            displays.append({
                'Class': 'rviz_default_plugins/Image',
                'Enabled': True,
                'Name': f'{r.name} camera',
                'Normalize Range': True,
                'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                          'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                          'Value': f'/{r.name}/camera/image_raw'},
                'Value': True,
            })
    return yaml.safe_dump(rviz, sort_keys=False)
