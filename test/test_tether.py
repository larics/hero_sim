import math

import numpy as np
import pytest

from conftest import wall_map
from hero_sim.obstacles import PILLAR_RES, ObstacleMap
from hero_sim.tether import initial_length, reached, step

# Winch, UGV pulley and UAV ring on the x axis, UAV 1 m up, object tied at the floor.
WINCH, UGV, UAV = (-2.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (1.0, 0.0, 1.0)
ROUTED = 1.0 + math.sqrt(5.0)  # |W - G| + |G - A|
OBJ = (3.0, 0.0)               # |A - O| = sqrt(5)
# A pillar between the object and the point under the UAV.
PILLAR = ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, 0.5, [(2.0, 0.0, 0.2)])


def test_initial_length_leaves_the_slack():
    assert initial_length(WINCH, UGV, UAV, OBJ, 0.0, 0.5) == pytest.approx(
        ROUTED + math.sqrt(5.0) + 0.5)


def test_slack_tether_leaves_the_object():
    s = step(ROUTED + math.sqrt(5.0) + 0.5, WINCH, UGV, UAV, OBJ, 0.0)
    assert s.obj == OBJ
    assert s.free == pytest.approx(math.sqrt(5.0) + 0.5)
    assert not s.taut and not s.stuck


def test_taut_tether_slides_object_toward_the_point_under_the_uav():
    s = step(ROUTED + 1.5, WINCH, UGV, UAV, OBJ, 0.0)
    assert s.obj == pytest.approx((1.0 + math.sqrt(1.5 ** 2 - 1.0), 0.0))
    assert math.dist(UAV, (*s.obj, 0.0)) == pytest.approx(1.5)
    assert s.taut and not s.stuck


def test_object_is_never_lifted_the_winch_slips():
    s = step(ROUTED + 0.2, WINCH, UGV, UAV, OBJ, 0.0)  # less than the UAV's height
    assert s.length == pytest.approx(ROUTED + 1.0)
    assert s.obj == pytest.approx((1.0, 0.0))
    assert s.free == pytest.approx(1.0)


def test_attach_height_shortens_the_drop():
    s = step(0.0, WINCH, UGV, UAV, OBJ, 0.25)
    assert s.free == pytest.approx(0.75)


def test_object_jams_against_a_pillar_and_the_winch_slips():
    s = step(ROUTED + 1.5, WINCH, UGV, UAV, OBJ, 0.0, PILLAR, 0.1)
    # Touching: pillar 0.2 + object 0.1, stopped within a path sample.
    assert 2.3 <= s.obj[0] <= 2.3 + PILLAR_RES / 2 and s.obj[1] == 0.0
    assert s.stuck and s.taut
    assert s.free == pytest.approx(math.dist(UAV, (*s.obj, 0.0)))
    assert s.length == pytest.approx(ROUTED + s.free)

    again = step(s.length - 0.5, WINCH, UGV, UAV, s.obj, 0.0, PILLAR, 0.1)
    assert again.obj == s.obj and again.stuck


def test_pull_away_from_the_pillar_frees_the_object():
    uav = (4.0, 0.0, 1.0)  # now on the far side of the object from the pillar
    s = step(0.0, WINCH, UGV, uav, (2.3, 0.0), 0.0, PILLAR, 0.1)
    assert s.obj == pytest.approx((4.0, 0.0))
    assert not s.stuck


def test_pillar_beside_the_path_lets_it_through():
    s = step(ROUTED + 1.5, WINCH, UGV, UAV, (3.0, 1.0), 0.0, PILLAR, 0.1)  # ends at (2, 0.5)
    assert not s.stuck
    assert math.dist(UAV, (*s.obj, 0.0)) == pytest.approx(1.5)


def test_object_jams_against_the_map():
    # Pulled from x = 2 toward x ~ 0.66 through wall_map's wall at x = 1.0..1.1.
    uav = (0.0, 0.0, 1.0)
    routed = math.dist(WINCH, UGV) + math.dist(UGV, uav)
    s = step(routed + 1.2, WINCH, UGV, uav, (2.0, 0.0), 0.0, wall_map(), 0.1)
    assert s.stuck
    assert 1.2 - 0.05 <= s.obj[0] <= 1.2 + 0.1  # far face + radius, within a cell


def test_reached():
    assert reached((1.0, 1.0), (1.0, 1.3, 0.4))
    assert not reached((1.0, 1.0), (1.0, 1.5, 0.4))
