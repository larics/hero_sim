import math

import numpy as np
import pytest
import yaml

from conftest import CONFIG_DIR, wall_map
from hero_sim import config
from hero_sim.config import Pillar
from hero_sim.obstacles import (
    _KEY_ORIGIN, PILLAR_RES, ObstacleMap, grid_voxels, load, pillar_voxels, read_bt,
    read_image_map, write_bt, write_image_map, write_pillar_map,
)

MAPS_DIR = CONFIG_DIR.parent / 'maps'
MAPS = sorted(MAPS_DIR.glob('*.bt'))
IMAGE_MAPS = sorted(MAPS_DIR.glob('*/*.yaml'))
ICUAS26_1 = MAPS_DIR / 'icuas26_1.bt'
TRUNK = (-3.0, 0.15)  # a tree trunk in icuas26_1


def pillar_map(max_radius=0.5):
    """No map grid, one pillar of radius 0.2 at (1, 0)."""
    return ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, max_radius, [(1.0, 0.0, 0.2)])


@pytest.mark.parametrize('path', MAPS, ids=lambda p: p.name)
def test_read_bt_shipped_maps(path):
    res, boxes = read_bt(path)  # raises on trailing bytes, so the whole tree was read
    assert res > 0 and len(boxes) > 0
    assert set(np.unique(boxes[:, 3])) <= {1, 2}


def test_read_bt_rejects_other_files(tmp_path):
    path = tmp_path / 'm.bt'
    path.write_bytes(b'# not an octomap\n')
    with pytest.raises(ValueError, match='not an octomap'):
        read_bt(path)


def test_band_keeps_trunks_and_drops_floor():
    trunks = ObstacleMap.from_bt(ICUAS26_1, 0.2, 2.0, max_radius=0.5)
    assert trunks.clearance(*TRUNK) == 0.0
    assert trunks.clearance(0.0, 0.0) == math.inf
    floor = ObstacleMap.from_bt(ICUAS26_1, 0.0, 0.1, max_radius=0.5)
    assert floor.clearance(0.0, 0.0) == 0.0


def test_empty_band_has_no_obstacles():
    empty = ObstacleMap.from_bt(ICUAS26_1, 10.0, 11.0, max_radius=0.5)
    assert empty.clearance(*TRUNK) == math.inf


def test_clearance_measured_to_the_obstacle_edge():
    m = wall_map()
    # From the centre of the cell holding the point to the wall face at x = 1.0.
    assert m.clearance(1.05, 0.0) == 0.0
    assert m.clearance(0.95, 0.0) == pytest.approx(0.05)
    assert m.clearance(0.82, 0.0) == pytest.approx(0.15)
    assert m.clearance(0.4, 0.0) == math.inf  # beyond max_radius


def test_move_stops_before_the_wall_by_radius():
    m = wall_map()
    x, y = m.move(0.0, 0.0, 2.0, 0.0, radius=0.3)
    assert y == 0.0
    assert 0.6 <= x <= 0.75  # wall face at 1.0, minus the radius, within a cell


def test_wider_robot_stops_further_out():
    m = wall_map()
    narrow, _ = m.move(0.0, 0.0, 2.0, 0.0, radius=0.1)
    wide, _ = m.move(0.0, 0.0, 2.0, 0.0, radius=0.4)
    assert wide < narrow - 0.2


def test_fast_move_cannot_tunnel_through():
    m = wall_map()
    # One step that would land well clear on the far side.
    x, _ = m.move(0.5, 0.0, 1.8, 0.0, radius=0.2)
    assert x < 1.0


def test_free_moves_untouched():
    m = wall_map()
    assert m.move(0.0, 0.0, 0.3, 0.2, radius=0.3) == (0.3, 0.2)
    assert m.move(0.0, -2.0, 3.0, -2.0, radius=0.3) == (3.0, -2.0)  # passes below the wall


def test_overlapping_robot_can_back_out_but_not_go_deeper():
    m = wall_map()
    assert m.move(0.95, 0.0, 0.5, 0.0, radius=0.3) == (0.5, 0.0)
    x, _ = m.move(0.95, 0.0, 1.05, 0.0, radius=0.3)
    assert x < 1.0


def test_radius_zero_never_collides_and_too_big_raises():
    m = wall_map(max_radius=0.3)
    assert m.move(0.0, 0.0, 2.0, 0.0, radius=0.0) == (2.0, 0.0)
    with pytest.raises(ValueError, match='max_radius'):
        m.move(0.0, 0.0, 1.0, 0.0, radius=0.4)


def test_load_off_without_robots_or_collision():
    sim = dict(config.SIM_DEFAULTS, map=str(ICUAS26_1))
    assert load(sim, [0.1]) is not None
    assert load(dict(sim, collision=False), [0.1]) is None
    assert load(sim, []) is None


def test_load_without_map_or_pillars_still_checks_others():
    m = load(dict(config.SIM_DEFAULTS, map=None), [0.3])
    assert m.clearance(0.0, 0.0) == math.inf
    assert m.move(0.0, 0.0, 2.0, 0.0, 0.3) == (2.0, 0.0)
    x, _ = m.move(0.0, 0.0, 2.0, 0.0, 0.3, others=[(1.0, 0.0, 0.2)])
    assert 0.5 - PILLAR_RES / 2 <= x <= 0.5


def test_pillar_clearance_is_exact_to_its_edge():
    m = pillar_map()
    assert m.clearance(1.5, 0.0) == pytest.approx(0.3)
    assert m.clearance(1.0, -0.45) == pytest.approx(0.25)
    assert m.clearance(1.0, 0.1) == 0.0  # inside
    assert m.clearance(2.0, 0.0) == math.inf  # 0.8 m: beyond max_radius


def test_clearance_is_the_nearer_of_map_and_pillar():
    occupied = np.zeros((10, 2), bool)
    occupied[:, 0] = True  # wall_map's wall, face at x = 1.0
    m = ObstacleMap(occupied, 1.0, -0.5, 0.1, 0.5, [(0.3, 0.0, 0.1)])
    assert m.clearance(0.62, 0.0) == pytest.approx(0.22)  # pillar edge at 0.4
    assert m.clearance(0.88, 0.0) == pytest.approx(0.15)  # the wall, from the cell centre


def test_move_stops_before_a_pillar_by_radius():
    m = pillar_map()
    x, y = m.move(0.0, 0.0, 2.0, 0.0, radius=0.3)
    assert y == 0.0
    assert 0.5 - PILLAR_RES / 2 <= x <= 0.5  # pillar edge at 0.8, minus the radius
    assert m.move(0.0, 0.6, 2.0, 0.6, radius=0.3) == (2.0, 0.6)  # passes beside it


def test_others_count_like_pillars():
    m = ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, 0.5)
    robot = [(1.0, 0.0, 0.2)]
    assert m.clearance(1.5, 0.0) == math.inf
    assert m.clearance(1.5, 0.0, robot) == pytest.approx(0.3)
    x, y = m.move(0.0, 0.0, 2.0, 0.0, 0.3, robot)
    assert 0.5 - PILLAR_RES / 2 <= x <= 0.5 and y == 0.0
    assert m.move(0.0, 0.6, 2.0, 0.6, 0.3, robot) == (2.0, 0.6)  # passes beside it
    assert m.move(0.7, 0.0, 0.0, 0.0, 0.3, robot) == (0.0, 0.0)  # overlapping: backs away
    assert m.move(0.7, 0.0, 0.9, 0.0, 0.3, robot) == (0.7, 0.0)  # but not deeper


def test_nearest_of_pillars_and_others():
    m = pillar_map()  # pillar edge at x = 0.8
    assert m.clearance(0.0, 0.0, [(0.0, 0.5, 0.1)]) == pytest.approx(0.4)
    assert m.clearance(0.0, 0.0, [(0.0, 2.0, 0.1)]) == math.inf  # both beyond max_radius


def test_load_pillars_with_and_without_map():
    pillars = [(1.0, 0.0, 0.2)]
    sim = dict(config.SIM_DEFAULTS, map=None)
    m = load(sim, [0.1], pillars)
    assert m.clearance(1.0, 0.0) == 0.0 and m.res == PILLAR_RES
    assert load(dict(sim, collision=False), [0.1], pillars) is None
    assert load(sim, [], pillars) is None

    both = load(dict(sim, map=str(ICUAS26_1)), [0.1], [(0.0, 0.0, 0.2)])
    assert both.clearance(0.0, 0.0) == 0.0
    assert both.clearance(*TRUNK) == 0.0


def bt_header(path):
    head = path.read_bytes().split(b'data\n')[0].decode()
    return dict(line.split(None, 1) for line in head.splitlines() if not line.startswith('#'))


def test_write_bt_one_voxel(tmp_path):
    path = tmp_path / 'one.bt'
    write_bt(path, {(_KEY_ORIGIN, _KEY_ORIGIN + 1, _KEY_ORIGIN + 2)}, 0.05)
    # octomap checks the node count: the root plus one node on each of 16 levels.
    assert bt_header(path) == {'id': 'OcTree', 'size': '17', 'res': '0.05'}
    res, boxes = read_bt(path)
    assert res == 0.05
    assert boxes.tolist() == [[_KEY_ORIGIN, _KEY_ORIGIN + 1, _KEY_ORIGIN + 2, 1]]
    with pytest.raises(ValueError, match='no voxels'):
        write_bt(path, set(), 0.05)


def test_pillar_voxels_fill_the_circle_from_the_floor_up():
    voxels = pillar_voxels([Pillar(1.0, -0.5, 0.2, height=0.5)], res=0.05)
    assert all(math.hypot((kx - _KEY_ORIGIN + 0.5) * 0.05 - 1.0,
                          (ky - _KEY_ORIGIN + 0.5) * 0.05 + 0.5) <= 0.2 for kx, ky, _ in voxels)
    assert sorted({kz - _KEY_ORIGIN for _, _, kz in voxels}) == list(range(10))  # 0 to 0.5 m
    columns = len(voxels) // 10
    assert columns == pytest.approx(math.pi * 0.2 ** 2 / 0.05 ** 2, rel=0.1)


def test_pillar_map_reads_back_as_the_same_obstacle(tmp_path):
    pillars = [Pillar(1.5, 0.5, 0.35), Pillar(-1.0, 2.0, 0.2, height=1.0)]
    path = write_pillar_map(tmp_path / 'new' / 'p.bt', pillars)
    res, boxes = read_bt(path)
    assert {tuple(b[:3]) for b in boxes.tolist()} == pillar_voxels(pillars)
    flat = ObstacleMap.from_bt(path, 0.2, 2.0, max_radius=0.5)
    exact = ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, 0.5, pillars)
    for x, y in ((1.5, 0.5), (-1.0, 2.0), (1.5 + 0.45, 0.5), (-1.0, 2.0 - 0.3), (0.0, 0.0)):
        assert flat.clearance(x, y) == pytest.approx(exact.clearance(x, y), abs=res)


def test_circles_ignore_a_pillar_height():
    a = ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, 0.5, [(1.0, 0.0, 0.2)])
    b = ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, 0.5, [Pillar(1.0, 0.0, 0.2, 0.1)])
    assert a.circles == b.circles == ((1.0, 0.0, 0.2),)


# ------------------------------------------------------------------ image maps

def image_map(tmp_path, gray, alpha=None, **meta):
    """Write m.yaml and m.png in tmp_path: gray levels, row 0 at the top. Returns the .yaml."""
    Image = pytest.importorskip('PIL.Image')
    gray = np.array(gray, np.uint8)
    if alpha is None:
        image = Image.fromarray(gray, 'L')
    else:
        image = Image.fromarray(np.dstack([gray, gray, gray, np.array(alpha, np.uint8)]), 'RGBA')
    image.save(tmp_path / 'm.png')
    path = tmp_path / 'm.yaml'
    path.write_text(yaml.safe_dump(
        {'image': 'm.png', 'resolution': 0.1, 'origin': [1.0, -2.0, 0.0], **meta}))
    return path


def test_image_map_rows_run_down_from_the_top(tmp_path):
    path = image_map(tmp_path, [[0, 255, 255],
                                [255, 255, 255]])  # black: the top-left pixel only
    occupied, x0, y0, res = read_image_map(path)
    assert (x0, y0, res) == (1.0, -2.0, 0.1)  # the bottom-left corner
    assert occupied.tolist() == [[False, False, False], [True, False, False]]  # row 0: bottom
    m = ObstacleMap.from_image(path, max_radius=0.5)
    assert m.clearance(1.05, -1.85) == 0.0  # in the top-left pixel: x 1.0..1.1, y -1.9..-1.8
    assert m.clearance(1.25, -1.95) > 0.0


def test_image_map_threshold_negate_and_transparency(tmp_path):
    gray = [[0, 100, 200, 255]]  # darkness 1, 0.61, 0.22, 0

    def walls(alpha=None, **meta):
        return read_image_map(image_map(tmp_path, gray, alpha, **meta))[0][0].tolist()
    assert walls() == [True, False, False, False]  # over map_server's default 0.65
    assert walls(occupied_thresh=0.5) == [True, True, False, False]
    assert walls(negate=1) == [False, False, True, True]
    assert walls(alpha=[[0, 255, 255, 255]]) == [False] * 4  # transparent black is free


@pytest.mark.parametrize('meta, match', [
    ({'origin': [0.0, 0.0, 0.5]}, 'rotated'),
    ({'origin': 5}, 'origin must be'),
    ({'resolution': 0}, 'resolution must be'),
    ({'mode': 'raw'}, 'mode'),
])
def test_image_map_rejects(tmp_path, meta, match):
    with pytest.raises(ValueError, match=match):
        read_image_map(image_map(tmp_path, [[0]], **meta))


def test_image_map_needs_image_resolution_origin(tmp_path):
    path = tmp_path / 'm.yaml'
    path.write_text('image: m.png\n')
    with pytest.raises(ValueError, match='missing resolution, origin'):
        read_image_map(path)


@pytest.mark.parametrize('path', IMAGE_MAPS, ids=lambda p: p.parent.name)
def test_shipped_image_maps(path):
    pytest.importorskip('PIL')
    assert path.stem == path.parent.name  # maps/<name>/<name>.yaml: config.resolve_map finds it
    occupied, x0, y0, res = read_image_map(path)
    h, w = occupied.shape
    assert (x0 + w * res / 2, y0 + h * res / 2) == pytest.approx((0.0, 0.0))  # centred
    assert occupied[:, 0].all() and occupied[:, -1].all()  # walled on the left and right


def test_grid_voxels_stand_on_the_cells():
    occupied = np.array([[True, False],
                         [False, True]])  # row 0: bottom
    voxels = grid_voxels(occupied, -0.1, 0.2, 0.05, height=0.2)
    assert {(kx - _KEY_ORIGIN, ky - _KEY_ORIGIN) for kx, ky, _ in voxels} == {(-2, 4), (-1, 5)}
    assert sorted({kz - _KEY_ORIGIN for *_, kz in voxels}) == [0, 1, 2, 3]  # 0 to 0.2 m
    assert len(voxels) == 8


def test_image_map_octomap_reads_back_as_the_same_walls(tmp_path):
    pytest.importorskip('PIL')
    path = MAPS_DIR / 'simple_maze_10x10' / 'simple_maze_10x10.yaml'
    bt = write_image_map(tmp_path / 'new' / 'maze.bt', path)
    res, boxes = read_bt(bt)
    occupied = read_image_map(path)[0]
    assert res == 0.05 and len(boxes) == occupied.sum() * 40  # walls 2 m tall, 5 cm voxels
    flat = ObstacleMap.from_bt(bt, 0.2, 2.0, max_radius=0.5)
    image = ObstacleMap.from_image(path, max_radius=0.5)
    for x, y in np.random.default_rng(0).uniform(-5.5, 5.5, (300, 2)):
        assert flat.clearance(x, y) == pytest.approx(image.clearance(x, y), abs=res)


def test_write_image_map_without_walls(tmp_path):
    assert write_image_map(tmp_path / 'm.bt', image_map(tmp_path, [[255, 255]])) is None


def test_load_image_map():
    pytest.importorskip('PIL')
    m = load(dict(config.SIM_DEFAULTS, map=str(MAPS_DIR / 'empty_3x3' / 'empty_3x3.yaml')), [0.3])
    assert m.res == 0.05
    assert m.clearance(1.47, 0.0) == 0.0  # the east wall: x 1.45..1.5
    assert m.clearance(0.0, 0.0) == math.inf
    x, _ = m.move(0.0, 0.0, 3.0, 0.0, 0.3)
    assert 1.45 - 0.3 - 0.05 <= x <= 1.45 - 0.3
