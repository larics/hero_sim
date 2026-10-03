"""Load and validate a hero_sim scenario config. No ROS imports at module level.

A scenario file has three sections (see config/example.yaml):

  ground_robots   kinematic ground robots, one entry per robot
  crazyflies      Crazyflies, named cf_1..cf_N in list order
  sim             optional overrides of SIM_DEFAULTS

Unknown keys are rejected so typos fail loudly instead of being ignored.
"""
from dataclasses import dataclass
import os
from pathlib import Path

import yaml

SIM_DEFAULTS = {
    'ground_rate': 50.0,         # Hz, ground robot integration + /pose rate
    'ground_max_speed': 2.0,     # m/s, clamp on |(linear.x, linear.y)|
    'ground_max_yaw_rate': 2.0,  # rad/s, clamp on |angular.z|
    'cf_rate': 10.0,             # Hz, Crazyflie update + /pose rate
    'cf_battery_drain': 0.025,   # V/s while flying (starts at 4.2 V, floor 3.0 V)
    'attach': True,              # run sim_attach (/sim/attach_agent dock ride-along)
    'rviz': True,                # open RViz with config/sim.rviz
    'map': 'icuas26_1',          # octomap: maps/<name>.bt, a path to a .bt, or null
}

DRIVES = ('omni', 'diff')
CONFIG_ENV = 'HERO_SIM_CONFIG'
DEFAULT_CONFIG = 'example.yaml'


@dataclass(frozen=True)
class GroundRobot:
    name: str
    x: float
    y: float
    yaw: float
    drive: str


@dataclass(frozen=True)
class Crazyflie:
    name: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Scenario:
    ground_robots: tuple
    crazyflies: tuple
    sim: dict


def load(path):
    """Read and validate a scenario file. Raises ValueError naming the file."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    try:
        return parse(raw)
    except ValueError as e:
        raise ValueError(f'{path}: {e}') from None


def parse(raw):
    """Validate an already-loaded scenario mapping and return a Scenario."""
    raw = {} if raw is None else raw
    _check_keys(raw, ('ground_robots', 'crazyflies', 'sim'), 'top level')

    ground = tuple(_ground_robot(i, r) for i, r in enumerate(_list(raw, 'ground_robots')))
    names = [r.name for r in ground]
    dups = sorted({n for n in names if names.count(n) > 1})
    if dups:
        raise ValueError(f'duplicate ground robot names: {dups}')

    cfs = tuple(_crazyflie(i, c) for i, c in enumerate(_list(raw, 'crazyflies')))
    clash = sorted(set(names) & {c.name for c in cfs})
    if clash:
        raise ValueError(f'ground robot names clash with crazyflie names: {clash}')

    sim_raw = raw.get('sim') or {}
    _check_keys(sim_raw, SIM_DEFAULTS, 'sim')
    sim = {**SIM_DEFAULTS, **sim_raw}
    for key in ('ground_rate', 'ground_max_speed', 'ground_max_yaw_rate', 'cf_rate'):
        sim[key] = float(sim[key])
        if sim[key] <= 0.0:
            raise ValueError(f'sim.{key} must be > 0, got {sim[key]}')
    sim['cf_battery_drain'] = float(sim['cf_battery_drain'])
    if sim['cf_battery_drain'] < 0.0:
        raise ValueError(f'sim.cf_battery_drain must be >= 0, got {sim["cf_battery_drain"]}')

    return Scenario(ground, cfs, sim)


def _list(raw, key):
    value = raw.get(key) or []
    if not isinstance(value, list):
        raise ValueError(f'{key} must be a list')
    return value


def _check_keys(d, allowed, where):
    if not isinstance(d, dict):
        raise ValueError(f'{where} must be a mapping, got {type(d).__name__}')
    unknown = sorted(set(d) - set(allowed))
    if unknown:
        raise ValueError(f'{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}')


def _pose(entry, where, axes):
    pose = entry.get('pose') or {}
    _check_keys(pose, axes, f'{where}.pose')
    return [float(pose.get(a, 0.0)) for a in axes]


def _ground_robot(i, entry):
    where = f'ground_robots[{i}]'
    _check_keys(entry, ('name', 'pose', 'drive'), where)
    name = entry.get('name')
    if not isinstance(name, str) or not name:
        raise ValueError(f'{where}: name is required')
    drive = str(entry.get('drive', 'omni')).lower()
    if drive not in DRIVES:
        raise ValueError(f'{where} ({name}): drive must be one of {DRIVES}, got {drive!r}')
    x, y, yaw = _pose(entry, f'{where} ({name})', ('x', 'y', 'yaw'))
    return GroundRobot(name, x, y, yaw, drive)


def _crazyflie(i, entry):
    name = f'cf_{i + 1}'
    where = f'crazyflies[{i}] ({name})'
    _check_keys(entry, ('pose',), where)
    x, y, z = _pose(entry, where, ('x', 'y', 'z'))
    return Crazyflie(name, x, y, z)


# ---------------------------------------------------------------- resolution

def share_dir():
    """Installed share directory of the hero_sim package (needs a sourced ROS env)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('hero_sim'))


def resolve_config(arg=''):
    """Turn a config argument into an existing file path.

    Empty falls back to $HERO_SIM_CONFIG, then DEFAULT_CONFIG. A bare filename
    is looked up in the package's config/ dir; anything with a '/' is a path.
    """
    arg = arg or os.environ.get(CONFIG_ENV, '') or DEFAULT_CONFIG
    path = Path(arg) if '/' in arg else share_dir() / 'config' / arg
    if not path.is_file():
        raise FileNotFoundError(f'hero_sim config not found: {path}')
    return str(path.resolve())


def resolve_map(value):
    """Turn sim.map into an existing .bt path, or None when the map is disabled."""
    if not value:
        return None
    value = str(value)
    path = Path(value) if '/' in value else share_dir() / 'maps' / f'{value}.bt'
    if not path.is_file():
        raise FileNotFoundError(f'hero_sim map not found: {path}')
    return str(path)
