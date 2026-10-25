import math

import pytest

from hero_sim.tether import first_contact, initial_length, reached, step

# Winch, UGV pulley and UAV ring on the x axis, UAV 1 m up, object tied at the floor.
WINCH, UGV, UAV = (-2.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (1.0, 0.0, 1.0)
ROUTED = 1.0 + math.sqrt(5.0)  # |W - G| + |G - A|
OBJ = (3.0, 0.0)               # |A - O| = sqrt(5)
PILLAR = (2.0, 0.0, 0.2)       # between the object and the point under the UAV


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
    s = step(ROUTED + 1.5, WINCH, UGV, UAV, OBJ, 0.0, [PILLAR], 0.1)
    assert s.obj == pytest.approx((2.3, 0.0))  # touching: pillar 0.2 + object 0.1
    assert s.stuck and s.taut
    assert s.free == pytest.approx(math.dist(UAV, (2.3, 0.0, 0.0)))
    assert s.length == pytest.approx(ROUTED + s.free)

    again = step(s.length - 0.5, WINCH, UGV, UAV, s.obj, 0.0, [PILLAR], 0.1)
    assert again.obj == pytest.approx(s.obj) and again.stuck


def test_pull_away_from_the_pillar_frees_the_object():
    uav = (4.0, 0.0, 1.0)  # now on the far side of the object from the pillar
    s = step(0.0, WINCH, UGV, uav, (2.3, 0.0), 0.0, [PILLAR], 0.1)
    assert s.obj == pytest.approx((4.0, 0.0))
    assert not s.stuck


def test_first_contact():
    pillar = [(0.0, 0.0, 0.5)]
    assert first_contact((-2.0, 0.0), (2.0, 0.0), pillar, 0.0) == pytest.approx((-0.5, 0.0))
    assert first_contact((-2.0, 1.0), (2.0, 1.0), pillar, 0.0) is None    # passes beside
    assert first_contact((-2.0, 0.0), (-1.0, 0.0), pillar, 0.0) is None   # stops short
    assert first_contact((-0.5, 0.0), (-2.0, 0.0), pillar, 0.0) is None   # moves away
    assert first_contact((-0.5, 0.0), (0.0, 0.0), pillar, 0.0) == (-0.5, 0.0)  # pushed in
    assert first_contact((-1.0, 0.0), (-1.0, 0.0), pillar, 0.0) is None   # no move


def test_reached():
    assert reached((1.0, 1.0), (1.0, 1.3, 0.4))
    assert not reached((1.0, 1.0), (1.0, 1.5, 0.4))
