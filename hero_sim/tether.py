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
  jammed  pulled into a pillar, the object stops where it touches it and the
          winch slips, until a pull away from the pillar frees it. The
          tether itself passes over pillars, and the robots through them.

The node in hero_sim/tether_sim.py runs this for the scenario's tether: section.
"""
import math
from typing import NamedTuple


class State(NamedTuple):
    length: float  # m, paid out by the winch (grows when it slips)
    obj: tuple     # x, y (m) of the object on the floor
    free: float    # m, tether from the UAV ring to the object
    taut: bool
    stuck: bool    # jammed against a pillar


def initial_length(winch, ugv, uav, obj, attach_z, slack):
    """Tether paid out with the robots where they are, leaving `slack` m to spare."""
    return (math.dist(winch, ugv) + math.dist(ugv, uav)
            + math.dist(uav, (*obj, attach_z)) + slack)


def reached(obj, goal):
    """Whether the object's center is inside the goal circle (x, y, radius)."""
    return math.hypot(obj[0] - goal[0], obj[1] - goal[1]) <= goal[2]


def first_contact(p, q, obstacles, radius):
    """Where an object of `radius` moving straight from p to q first touches an
    obstacle (x, y, r), or None if the move is clear. Moving away from an
    obstacle it already touches is clear."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    a = dx * dx + dy * dy
    if a == 0.0:
        return None
    best = None
    for cx, cy, r in obstacles:
        r += radius
        px, py = p[0] - cx, p[1] - cy
        b = dx * px + dy * py  # < 0: heading toward the center
        if px * px + py * py <= r * r + 1e-9:
            if b < 0.0:
                return tuple(p)  # touching it and pushed in: jammed
            continue
        c = px * px + py * py - r * r
        disc = b * b - a * c
        if b >= 0.0 or disc < 0.0:
            continue
        t = (-b - math.sqrt(disc)) / a
        if t <= 1.0 and (best is None or t < best):
            best = t
    if best is None:
        return None
    return p[0] + best * dx, p[1] + best * dy


def step(length, winch, ugv, uav, obj, attach_z, obstacles=(), radius=0.0):
    """One quasi-static update of the tether and the object. Returns a State.

    length: tether paid out by the winch (m). winch, ugv, uav: (x, y, z) the
    tether runs through, in order. obj: object (x, y) on the floor, tied at
    height attach_z, with footprint `radius` against obstacles (x, y, r).
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
    contact = first_contact(obj, target, obstacles, radius)
    if contact is None:
        return State(length, target, free, True, False)
    # Jammed against an obstacle: the object stops there and the winch slips.
    free = math.dist(uav, (*contact, attach_z))
    return State(routed + free, contact, free, True, True)
