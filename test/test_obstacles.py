import math

import numpy as np
import pytest

from conftest import CONFIG_DIR, wall_map
from hero_sim import config
from hero_sim.obstacles import PILLAR_RES, ObstacleMap, load, read_bt

MAPS = sorted((CONFIG_DIR.parent / 'maps').glob('*.bt'))
ICUAS26_1 = CONFIG_DIR.parent / 'maps' / 'icuas26_1.bt'
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
