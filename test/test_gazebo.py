"""Cameras and the Gazebo world: config keys, world SDF, map meshes, RViz panel, bridge helpers.

None of this needs Gazebo installed. The bridge helpers need ROS messages and skip without them.
"""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from conftest import CONFIG_DIR
from hero_sim import config, gazebo, obstacles

MAPS = CONFIG_DIR.parent / 'maps'
MODELS = CONFIG_DIR.parent / 'models'


def parse(text):
    return config.parse(yaml.safe_load(text))


# ------------------------------------------------------------------ config

def test_camera_defaults_by_model():
    s = parse('''
ground_robots:
  - {name: h, model: herover, camera: true}
  - {name: s, model: scout, camera: true}
  - {name: r, camera: true}
  - {name: n, model: herover}
  - {name: f, model: herover, camera: false}
''')
    h, sc, r, n, f = s.ground_robots
    assert h.camera == config.Camera(*config.CAMERA_MOUNTS['herover'])
    assert (sc.camera.x, sc.camera.z) == config.CAMERA_MOUNTS['scout'][::2]
    assert (r.camera.x, r.camera.y, r.camera.z) == config.DEFAULT_CAMERA_MOUNT
    assert n.camera is None and f.camera is None
    assert (h.camera.width, h.camera.height, h.camera.rate) == (640, 480, 15.0)


def test_camera_overrides_keep_other_defaults():
    s = parse('ground_robots: [{name: h, model: herover, camera: '
              '{width: 320, height: 240, rate: 30, pose: {z: 0.1, pitch: 0.3}}}]')
    cam = s.ground_robots[0].camera
    assert (cam.width, cam.height, cam.rate) == (320, 240, 30.0)
    assert (cam.x, cam.z, cam.pitch) == (config.CAMERA_MOUNTS['herover'][0], 0.1, 0.3)
    assert cam.hfov == config.Camera.hfov


def test_sim_gazebo_defaults():
    s = parse('ground_robots: [{name: h}]')
    assert s.sim['gazebo'] == 'auto' and s.sim['gazebo_gui'] is False
    assert s.sim['map_model'] is False


@pytest.mark.parametrize('text, match', [
    ('ground_robots: [{name: a, camera: yes please}]', r'\(a\)\.camera must be true, false or a mapping'),
    ('ground_robots: [{name: a, camera: {fps: 5}}]', r"\(a\)\.camera: unknown key.*'fps'"),
    ('ground_robots: [{name: a, camera: {pose: {roll: 1}}}]', r"camera\.pose: unknown key.*'roll'"),
    ('ground_robots: [{name: a, camera: {width: 0}}]', 'width must be a whole number'),
    ('ground_robots: [{name: a, camera: {height: 240.5}}]', 'height must be a whole number'),
    ('ground_robots: [{name: a, camera: {hfov: 3.2}}]', 'hfov must be between 0 and pi'),
    ('ground_robots: [{name: a, camera: {rate: 0}}]', 'rate must be > 0'),
    ('ground_robots: [{name: a, camera: true}]\nsim: {gazebo: false}',
     'a camera needs Gazebo, but sim.gazebo is false'),
    ('sim: {gazebo: maybe}', 'gazebo must be auto, true or false'),
    ('sim: {gazebo_gui: 1}', 'gazebo_gui must be true or false'),
    ('sim: {map_model: yes please}', 'map_model must be true or false'),
])
def test_rejects_bad_camera_config(text, match):
    with pytest.raises(ValueError, match=match):
        parse(text)


def test_gazebo_enabled(monkeypatch):
    monkeypatch.setattr(config, 'gazebo_missing', lambda: [])
    assert not config.gazebo_enabled(parse('ground_robots: [{name: a}]'))
    assert config.gazebo_enabled(parse('ground_robots: [{name: a, camera: true}]'))
    assert config.gazebo_enabled(parse('sim: {gazebo: true}'))
    assert not config.gazebo_enabled(parse('sim: {gazebo: false}'))


def test_gazebo_needed_but_missing(monkeypatch):
    monkeypatch.setattr(config, 'gazebo_missing', lambda: ['gz-harmonic'])
    assert not config.gazebo_enabled(parse('ground_robots: [{name: a}]'))  # never asks
    with pytest.raises(RuntimeError, match='a ground robot has a camera.*missing: gz-harmonic'):
        config.gazebo_enabled(parse('ground_robots: [{name: a, camera: true}]'))
    with pytest.raises(RuntimeError, match='sim.gazebo is true'):
        config.gazebo_enabled(parse('sim: {gazebo: true}'))


# ------------------------------------------------------------------ meshes

def _check_quads(corners, normals):
    """Every quad is planar, wound counter-clockwise around its outward normal. Returns total area."""
    area = 0.0
    for quad, n in zip(corners, normals):
        cross = np.cross(quad[1] - quad[0], quad[3] - quad[0])
        assert np.allclose(np.cross(cross, n), 0) and cross @ n > 0
        assert np.isclose((quad[2] - quad[0]) @ n, 0)
        area += np.linalg.norm(cross)
    return area


def test_grid_quads_one_cube():
    corners, normals = gazebo.grid_quads(np.ones((1, 1, 1), bool))
    assert len(corners) == 6
    assert _check_quads(corners, normals) == pytest.approx(6.0)
    assert {tuple(n) for n in normals.astype(int)} == {
        (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)}
    # The +x face is at x = 1, the -x face at x = 0.
    for quad, n in zip(corners, normals):
        axis = int(np.argmax(abs(n)))
        assert np.allclose(quad[:, axis], 1.0 if n[axis] > 0 else 0.0)


def test_grid_quads_leave_out_inner_faces_and_merge_rows():
    two = np.ones((2, 1, 1), bool)
    corners, normals = gazebo.grid_quads(two)
    assert _check_quads(corners, normals) == pytest.approx(10.0)  # 12 faces, 2 inside
    slab = np.ones((10, 10, 1), bool)
    corners, normals = gazebo.grid_quads(slab)
    assert _check_quads(corners, normals) == pytest.approx(2 * 100 + 4 * 10)
    assert len(corners) < 2 * 100 + 4 * 10  # rows joined into strips


def test_grid_quads_empty():
    corners, normals = gazebo.grid_quads(np.zeros((3, 3, 3), bool))
    assert corners.shape == (0, 4, 3) and normals.shape == (0, 3)


def _read_obj(path):
    lines = open(path).read().splitlines()
    verts = np.array([[float(v) for v in l.split()[1:]] for l in lines if l.startswith('v ')])
    faces = [l for l in lines if l.startswith('f ')]
    return verts, faces


def test_write_obj_scales_cells(tmp_path):
    path = gazebo.write_obj(tmp_path / 'm.obj', np.ones((1, 1, 1), bool), (1.0, 2.0, 0.0),
                            (0.5, 0.5, 2.0))
    verts, faces = _read_obj(path)
    assert len(faces) == 12  # two triangles a face
    assert np.allclose(verts.min(axis=0), (1.0, 2.0, 0.0))
    assert np.allclose(verts.max(axis=0), (1.5, 2.5, 2.0))


def test_octomap_grid_matches_the_octomap():
    res, boxes = obstacles.read_bt(MAPS / 'icuas26_1.bt')
    filled, origin, cell = gazebo.octomap_grid(MAPS / 'icuas26_1.bt')
    assert cell == (res, res, res)
    assert filled.sum() == int((boxes[:, 3] ** 3).sum())
    lo = (boxes[:, :3].min(axis=0) - obstacles._KEY_ORIGIN) * res
    assert np.allclose(origin, lo)


def test_map_mesh_uses_the_octomaps_own_mesh(tmp_path):
    out = tmp_path / 'map.obj'
    assert gazebo.map_mesh({'map': str(MAPS / 'icuas26_1.bt')}, out) == str(MAPS / 'icuas26_1.stl')
    assert not out.exists()
    assert gazebo.map_mesh({'map': None}, out) is None


def test_map_mesh_from_voxels_without_a_mesh(tmp_path):
    bt = obstacles.write_pillar_map(tmp_path / 'p.bt', [config.Pillar(1.0, 0.0, 0.2, 0.5)])
    mesh = gazebo.map_mesh({'map': str(bt)}, tmp_path / 'p.obj')
    verts, _ = _read_obj(mesh)
    assert np.allclose(verts.min(axis=0), (0.8, -0.2, 0.0), atol=obstacles.PILLAR_RES)
    assert np.allclose(verts.max(axis=0), (1.2, 0.2, 0.5), atol=obstacles.PILLAR_RES)


def test_map_mesh_image_map_walls(tmp_path):
    yaml_path = MAPS / 'simple_maze_4x4' / 'simple_maze_4x4.yaml'
    mesh = gazebo.map_mesh({'map': str(yaml_path)}, tmp_path / 'maze.obj')
    verts, faces = _read_obj(mesh)
    occupied, x0, y0, res = obstacles.read_image_map(yaml_path)
    rows, cols = np.nonzero(occupied)
    assert np.allclose(verts.min(axis=0), (x0 + cols.min() * res, y0 + rows.min() * res, 0.0))
    assert np.allclose(verts.max(axis=0), (x0 + (cols.max() + 1) * res,
                                           y0 + (rows.max() + 1) * res,
                                           obstacles.MAP_WALL_HEIGHT))
    assert faces


# ------------------------------------------------------------------- world

SCENARIO = '''
ground_robots:
  - {name: herover_1, pose: {x: 1.0, y: 2.0, yaw: 0.5}, model: herover,
     camera: {width: 320, height: 240, pose: {pitch: 0.2}}}
  - {name: plain}
crazyflies: [{pose: {x: 3.0, z: 0.5}}]
obstacles: [{x: 0.0, y: 3.0, radius: 0.3, height: 1.5}]
'''


def _world(text=SCENARIO, mesh=None):
    return ET.fromstring(gazebo.world_sdf(parse(text), MODELS, mesh)).find('world')


def test_world_has_scenery_and_every_robot():
    world = _world()
    assert world.get('name') == gazebo.WORLD
    models = {m.get('name'): m for m in world.findall('model')}
    assert set(models) == {'hero_sim_floor', 'hero_sim_pillar_0', 'herover_1', 'plain', 'cf_1'}
    assert all(m.findtext('static') == 'true' for m in models.values())
    plugins = {p.get('name') for p in world.findall('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
            'gz::sim::systems::Sensors'} <= plugins  # Physics applies set_pose_vector
    pillar = models['hero_sim_pillar_0']
    assert pillar.findtext('pose').split()[:3] == ['0', '3', '0.75']
    assert pillar.find('.//cylinder/length').text == '1.5'


def test_world_robot_frames_meshes_and_camera():
    models = {m.get('name'): m for m in _world().findall('model')}
    h = models['herover_1']
    z = config.MARKER_HEIGHTS['herover']
    assert [float(v) for v in h.findtext('pose').split()] == pytest.approx([1, 2, z, 0, 0, 0.5])
    visual = h.find('link/visual')
    assert visual.findtext('pose').split()[2] == f'{-z:.6g}'  # mesh back on the floor
    uri = visual.findtext('geometry/mesh/uri')
    assert uri.startswith('file:///') and uri.endswith('models/herover/herover.glb')
    sensor = h.find('link/sensor')
    assert sensor.get('type') == 'camera'
    assert sensor.findtext('topic') == gazebo.camera_topic('herover_1')
    assert sensor.findtext('camera/image/width') == '320'
    x, _, cz = config.CAMERA_MOUNTS['herover']
    assert [float(v) for v in sensor.findtext('pose').split()] == pytest.approx([x, 0, cz, 0, 0.2, 0])
    assert models['plain'].find('link/visual') is None  # no model: nothing to see, as in RViz
    assert models['plain'].find('link/sensor') is None
    assert models['cf_1'].findtext('link/visual/geometry/mesh/uri').endswith('crazyflie.dae')


def test_world_map_mesh(tmp_path):
    world = _world('ground_robots: [{name: a}]', MAPS / 'icuas26_1.stl')
    visual = world.find("model[@name='hero_sim_map']/link/visual")
    assert visual.findtext('geometry/mesh/uri').endswith('icuas26_1.stl')
    assert visual.find('material') is not None  # an .stl has no colors of its own
    assert world.find("light[@name='hero_sim_sun']") is not None


SCENERY = """<?xml version="1.0"?>
<sdf version="1.6"><world name="room">
  <gui><camera name="c"/></gui>
  <physics name="p" type="ode"/>
  <model name="Sofa_01_001"><static>true</static>
    <include><uri>model://sofa</uri></include><pose>1 2 0 0 0 0</pose></model>
  <light name="sun" type="directional"><direction>0 0 -1</direction></light>
</world></sdf>
"""


def _scenery_map(tmp_path):
    """A map with its own scenery: room/room.yaml and image, room.sdf, models/sofa/."""
    from PIL import Image
    d = tmp_path / 'room'
    (d / 'models' / 'sofa').mkdir(parents=True)
    Image.new('L', (20, 20), 255).save(d / 'room.png')
    (d / 'room.yaml').write_text('image: room.png\nresolution: 0.1\norigin: [-1.0, -1.0, 0.0]\n')
    (d / 'room.sdf').write_text(SCENERY)
    return d


def test_world_with_a_maps_scenery(tmp_path):
    d = _scenery_map(tmp_path)
    sim = {'map': str(d / 'room.yaml')}
    assert gazebo.map_scenery(sim) == str(d / 'room.sdf')
    assert gazebo.resource_paths(sim) == [str(d / 'models')]
    s = parse(f'ground_robots: [{{name: a, camera: true}}]\nsim: {{map: {d / "room.yaml"}}}')
    world = ET.parse(gazebo.write_world(tmp_path / 'w.sdf', s, MODELS)).getroot().find('world')
    names = {m.get('name') for m in world.findall('model')}
    assert {'Sofa_01_001', 'a', 'hero_sim_floor'} <= names and 'hero_sim_map' not in names
    assert world.find("model[@name='Sofa_01_001']/include/uri").text == 'model://sofa'
    lights = [light.get('name') for light in world.findall('light')]
    assert lights == ['sun']  # the scenery's own lights, not hero_sim's sun as well
    assert len(world.findall('physics')) == 1 and world.find('gui') is None  # only its scenery
    assert not (tmp_path / 'w_map.obj').exists()


def test_no_scenery_without_an_sdf(tmp_path):
    d = _scenery_map(tmp_path)
    (d / 'room.sdf').unlink()
    sim = {'map': str(d / 'room.yaml')}
    assert gazebo.map_scenery(sim) is None and gazebo.resource_paths(sim) == []
    assert gazebo.map_scenery({'map': None}) is None


def test_write_world_builds_the_map_mesh_next_to_it(tmp_path):
    s = parse('ground_robots: [{name: a, camera: true}]\nsim: {map: '
              f'{MAPS / "simple_maze_4x4" / "simple_maze_4x4.yaml"}}}')
    path = gazebo.write_world(tmp_path / 'out' / 'x_world.sdf', s, MODELS)
    mesh = tmp_path / 'out' / 'x_world_map.obj'
    assert mesh.is_file()
    world = ET.parse(path).getroot().find('world')
    uris = [u.text for u in world.iter('uri')]
    assert mesh.as_uri() in uris


def test_rviz_config_adds_an_image_panel_per_camera():
    base = (CONFIG_DIR / 'sim.rviz').read_text()
    before = yaml.safe_load(base)['Visualization Manager']['Displays']
    s = parse('ground_robots: [{name: a, camera: true}, {name: b}, {name: c, camera: true}]')
    after = yaml.safe_load(gazebo.rviz_config(base, s))['Visualization Manager']['Displays']
    added = after[len(before):]
    assert after[:len(before)] == before
    assert [d['Topic']['Value'] for d in added] == ['/a/camera/image_raw', '/c/camera/image_raw']
    assert all(d['Class'] == 'rviz_default_plugins/Image' for d in added)


# ------------------------------------------------------------- small_house

HOUSE = MAPS / 'small_house'


def test_small_house_scenery_models_are_all_there():
    text = (HOUSE / 'small_house.sdf').read_text()
    elements = ET.fromstring(f'<w>{gazebo.scenery_elements(HOUSE / "small_house.sdf")}</w>')
    assert len(elements.findall('model')) > 50 and elements.findall('light')
    models = set(re.findall(r'model://(\w+)', text))
    for model in models:
        d = HOUSE / 'models' / model
        sdf = (d / 'model.sdf').read_text()
        assert '<inertial>' not in sdf, f'{model}: Harmonic refuses its mass of 0'
        for mesh in re.findall(r'model://' + model + r'/([\w/.]+)', sdf):
            assert (d / mesh).is_file(), f'{model}: {mesh}'
        for dae in (d / 'meshes').glob('*.DAE'):
            for tex in re.findall(r'<init_from>\.\./materials/textures/([\w.]+)<', dae.read_text(
                    errors='replace')):
                assert (d / 'materials' / 'textures' / tex).is_file(), f'{model}: {tex}'
            assert 'photos/' not in dae.read_text(errors='replace')


def test_every_room_of_the_small_house_is_reachable():
    radius = config.FOOTPRINT_RADII['herover']
    grid = obstacles.ObstacleMap.from_image(HOUSE / 'small_house.yaml', max_radius=radius)
    # A herover's flood fill from larry.yaml's start, on a grid reaching past the outside walls.
    step, x0, y0 = 0.05, -11.0, -7.5
    xs, ys = x0 + step * np.arange(441), y0 + step * np.arange(301)
    free = np.array([[grid.clearance(x, y) >= radius for y in ys] for x in xs])

    def cell(x, y):
        return round((x - x0) / step), round((y - y0) / step)

    start = cell(1.0, 1.5)
    assert free[start]
    seen, todo = {start}, [start]
    while todo:
        i, j = todo.pop()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= n[0] < len(xs) and 0 <= n[1] < len(ys) and free[n] and n not in seen:
                seen.add(n)
                todo.append(n)
    rooms = {'bedroom': (-6.25, -3.0), 'living room': (1.25, 0.75), 'sunroom': (1.25, 4.5),
             'dining room': (7.0, -3.0)}
    for room, (x, y) in rooms.items():
        assert cell(x, y) in seen, f'a herover can\'t reach the {room}'
    assert free[cell(-10.5, 0.0)]  # outside is free floor, but there's no way out to it:
    assert all(-9.6 < xs[i] < 9.6 and -6.0 < ys[j] < 6.0 for i, j in seen)


# ------------------------------------------------------- map model for RViz

NESTED = """<?xml version="1.0"?>
<sdf version="1.6"><world name="room">
  <model name="Lamp_001"><static>true</static>
    <include><uri>model://lamp</uri></include>
    <pose>1 2 0 0 0 1.5707963</pose></model>
  <model name="Moved_001"><include><uri>model://lamp</uri><pose>5 0 0 0 0 0</pose></include></model>
  <light name="sun" type="directional"/>
</world></sdf>
"""
LAMP = """<?xml version="1.0"?>
<sdf version="1.6"><model name="lamp"><pose>9 9 9 0 0 0</pose>
  <link name="base"><pose>0.5 0 0 0 0 0</pose>
    <visual name="shade"><pose>0 0 0.2 0 0 0</pose>
      <geometry><mesh><uri>model://lamp/meshes/shade.dae</uri><scale>2 2 2</scale></mesh></geometry></visual>
    <visual name="post"><geometry><cylinder><radius>0.1</radius><length>1.0</length></cylinder></geometry>
      <material><diffuse>1 0 0 1</diffuse></material></visual>
    <visual name="foot"><geometry><box><size>0.3 0.4 0.05</size></box></geometry></visual>
    <visual name="floor"><geometry><plane><normal>0 0 1</normal></plane></geometry></visual>
  </link>
</model></sdf>
"""


def _nested_scenery(tmp_path):
    (tmp_path / 'models' / 'lamp').mkdir(parents=True)
    (tmp_path / 'models' / 'lamp' / 'model.sdf').write_text(LAMP)
    (tmp_path / 'room.sdf').write_text(NESTED)
    return tmp_path / 'room.sdf'


def test_scenery_visuals_compose_poses(tmp_path):
    visuals = gazebo.scenery_visuals(_nested_scenery(tmp_path))
    lamp = [v for v in visuals if v.name == 'Lamp_001']
    assert [v.shape for v in lamp] == ['mesh', 'cylinder', 'box']  # the plane is left out
    shade, post, foot = lamp
    # wrapper at (1, 2) turned 90 deg; no include <pose>, so the lamp's own (9, 9, 9) applies;
    # then link 0.5 m along x and visual 0.2 m up: (9.5, 9, 9.2) turned 90 deg is (-9, 9.5, 9.2)
    assert shade.position == pytest.approx((1 - 9.0, 2 + 9.5, 9.2), abs=1e-6)
    assert shade.orientation == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)), abs=1e-6)
    assert shade.mesh == str(tmp_path / 'models' / 'lamp' / 'meshes' / 'shade.dae')
    assert shade.size == (2.0, 2.0, 2.0) and shade.color is None  # the mesh's own colors
    assert post.size == (0.2, 0.2, 1.0) and post.color == (1.0, 0.0, 0.0, 1.0)
    assert foot.size == pytest.approx((0.3, 0.4, 0.05))
    moved = [v for v in visuals if v.name == 'Moved_001'][0]
    assert moved.position == pytest.approx((5.5, 0.0, 0.2))  # the include's <pose>, not the model's


def test_map_visuals_by_kind_of_map(tmp_path):
    house = gazebo.map_visuals({'map': str(HOUSE / 'small_house.yaml')})
    assert len(house) > 80 and all(Path(v.mesh).is_file() for v in house if v.shape == 'mesh')
    trees, = gazebo.map_visuals({'map': str(MAPS / 'icuas26_1.bt')})
    assert trees.mesh == str(MAPS / 'icuas26_1.stl') and trees.color[:3] == gazebo.MAP_COLOR
    maze = MAPS / 'simple_maze_4x4' / 'simple_maze_4x4.yaml'
    assert gazebo.map_visuals({'map': str(maze)}) == []
    assert gazebo.map_visuals({'map': str(MAPS / 'icuas26_2.bt')}) == []  # no mesh of its own
    assert gazebo.map_visuals({'map': None}) == []


def test_map_markers(tmp_path):
    pytest.importorskip('visualization_msgs')
    from visualization_msgs.msg import Marker
    from hero_sim.robot_models import map_markers
    markers = map_markers(gazebo.scenery_visuals(_nested_scenery(tmp_path)))
    shade, post, foot = markers[:3]
    assert shade.type == Marker.MESH_RESOURCE and shade.mesh_use_embedded_materials
    assert shade.mesh_resource == (tmp_path / 'models' / 'lamp' / 'meshes' / 'shade.dae').as_uri()
    assert (post.type, foot.type) == (Marker.CYLINDER, Marker.CUBE)
    assert (post.color.r, post.color.a) == (1.0, 1.0) and not post.mesh_use_embedded_materials
    assert all(m.header.frame_id == 'world' and m.ns == 'map_model' for m in markers)
    assert len({m.id for m in markers}) == len(markers)


# ---------------------------------------------------------- camera geometry

def _rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    return 2 * (u @ v) * u + (w * w - u @ u) * np.asarray(v) + 2 * w * np.cross(u, v)


def test_optical_frame_looks_along_the_camera_x():
    q = gazebo.OPTICAL_ROTATION
    assert np.allclose(_rotate(q, [0, 0, 1]), [1, 0, 0])   # optical z: forward
    assert np.allclose(_rotate(q, [1, 0, 0]), [0, -1, 0])  # optical x: right
    assert np.allclose(_rotate(q, [0, 1, 0]), [0, 0, -1])  # optical y: down


def test_intrinsics():
    fx, fy, cx, cy = gazebo.intrinsics(config.Camera(width=640, height=480, hfov=math.pi / 2))
    assert (fx, fy, cx, cy) == pytest.approx((320.0, 320.0, 320.0, 240.0))


def test_bridge_camera_info_and_transforms():
    pytest.importorskip('sensor_msgs')
    from hero_sim import gazebo_bridge
    cam = config.Camera(x=0.1, z=0.03, pitch=0.2, width=320, height=240, hfov=math.pi / 2)
    info = gazebo_bridge.camera_info(cam, 'h/camera_optical')
    assert (info.width, info.height, info.header.frame_id) == (320, 240, 'h/camera_optical')
    assert list(info.k) == pytest.approx([160, 0, 160, 0, 160, 120, 0, 0, 1])
    mount, lens = gazebo_bridge.camera_transforms('h', cam)
    assert (mount.header.frame_id, mount.child_frame_id) == ('h', 'h/camera_link')
    assert (lens.header.frame_id, lens.child_frame_id) == ('h/camera_link', 'h/camera_optical')
    t = mount.transform
    assert (t.translation.x, t.translation.z) == pytest.approx((0.1, 0.03))
    assert 2 * math.atan2(t.rotation.y, t.rotation.w) == pytest.approx(0.2)
