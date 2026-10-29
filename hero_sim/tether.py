"""Tether model: a winch drags an object across the floor. No ROS imports.

The tether runs from a fixed winch, through a pulley on a ground robot (UGV),
through a guide ring on a Crazyflie (UAV), down to an object on the floor:

    W (winch) -> G (UGV pulley) -> A (UAV ring) -> O (object)

The winch pays out a total length L. What is left after the two moving
pulleys is the free length below the UAV:

    l = L - |W - G| - |G - A|

Quasi-static (no swing), and the robots never feel the tether:

  slack   |A - O| < l: the object stays put.
  taut    the object slides along the floor toward the point under the UAV
          until |A - O| = l.
  lifted  never: the object is too heavy, so once it sits under the UAV the
          winch slips (pays out) instead.
  jammed  pulled into an obstacle (a pillar or the map, hero_sim/obstacles.py),
          the object stops where it touches it and the winch slips, until a
          pull away from the obstacle frees it. The robots stop at the same
          obstacles; the tether line itself passes through them.

The node in hero_sim/tether_sim.py runs this for the scenario's tether: section.
"""
import math
from typing import NamedTuple


class State(NamedTuple):
    length: float  # m, paid out by the winch (grows when it slips)
    obj: tuple     # x, y (m) of the object on the floor
    free: float    # m, tether from the UAV ring to the object
    taut: bool
    stuck: bool    # jammed against an obstacle


def initial_length(winch, ugv, uav, obj, attach_z, slack):
    """Tether paid out with the robots where they are, leaving `slack` m to spare."""
    return (math.dist(winch, ugv) + math.dist(ugv, uav)
            + math.dist(uav, (*obj, attach_z)) + slack)


def reached(obj, goal):
    """Whether the object's center is inside the goal circle (x, y, radius)."""
    return math.hypot(obj[0] - goal[0], obj[1] - goal[1]) <= goal[2]


def step(length, winch, ugv, uav, obj, attach_z, obstacles=None, radius=0.0):
    """One quasi-static update of the tether and the object. Returns a State.

    length: tether paid out by the winch (m). winch, ugv, uav: (x, y, z) the
    tether runs through, in order. obj: object (x, y) on the floor, tied at
    height attach_z, with footprint `radius` against `obstacles`, an
    obstacles.ObstacleMap or None for none.
    """
    routed = math.dist(winch, ugv) + math.dist(ugv, uav)
    dz = max(uav[2] - attach_z, 0.0)
    # The object can't leave the floor: the winch slips rather than lift it.
    length = max(length, routed + dz)
    free = length - routed
    reach = math.sqrt(max(free * free - dz * dz, 0.0))  # horizontal, from under the UAV
    dx, dy = obj[0] - uav[0], obj[1] - uav[1]
    h = math.hypot(dx, dy)
    if h <= reach:
        return State(length, tuple(obj), free, h > reach - 1e-3, False)
    # Taut: slide toward the point under the UAV.
    target = (uav[0] + dx * reach / h, uav[1] + dy * reach / h)
    if obstacles is None:
        return State(length, target, free, True, False)
    contact = obstacles.move(obj[0], obj[1], target[0], target[1], radius)
    if contact == target:
        return State(length, target, free, True, False)
    # Jammed against an obstacle: the object stops there and the winch slips.
    free = math.dist(uav, (*contact, attach_z))
    return State(routed + free, contact, free, True, True)
