"""Load and validate a hero_sim scenario config. No ROS imports at module level.

A scenario file has these sections (see config/example.yaml):

  ground_robots   kinematic ground robots, one entry per robot
  crazyflies      Crazyflies, named cf_1..cf_N in list order
  sim             optional overrides of SIM_DEFAULTS
  obstacles       optional pillars (x, y, radius) that block robots and the tether object
  tether          optional winch, tether and object (hero_sim/tether.py)

Unknown keys are rejected so typos fail loudly instead of being ignored.
"""
from dataclasses import dataclass
import math
import os
from pathlib import Path

import yaml

SIM_DEFAULTS = {
    'ground_rate': 50.0,         # Hz, ground robot integration + /pose rate
    'ground_max_speed': 2.0,     # m/s, clamp on |(linear.x, linear.y)| and go_to
    'ground_max_yaw_rate': 2.0,  # rad/s, clamp on |angular.z| and go_to
    'cf_rate': 10.0,             # Hz, Crazyflie update + /pose rate
    'cf_battery_drain': 0.025,   # V/s while flying (starts at 4.2 V, floor 3.0 V)
    'attach': 'auto',            # sim_attach dock ride-along: auto | true | false (see attach_enabled)
    'rviz': True,                # open RViz with config/sim.rviz
    'models': True,              # robot meshes in RViz (crazyflies + ground robots with a model)
    'map': 'icuas26_1',          # octomap: maps/<name>.bt, a path to a .bt, or null
    'collision': True,           # stop robots at the map, pillars and each other (obstacles.py)
    'collision_z': (0.2, 2.0),   # m, map heights counted as obstacles: trunks, not floor/canopy
}

# RViz meshes by model name, relative to models/. Crazyflies always use
# 'crazyflie'; a ground robot uses its `model:` key (none -> TF axes only).
MODEL_MESHES = {
    'crazyflie': 'crazyflie/crazyflie.dae',
    'herover': 'herover/herover.glb',
    'scout': 'scout/scout.glb',
}

# Footprint radius (m) for map collisions, by model: the circle around the
# mesh seen from above. Ground robots without a model get
# DEFAULT_GROUND_RADIUS. Any robot can set its own with `radius:`.
FOOTPRINT_RADII = {
    'crazyflie': 0.1,
    'herover': 0.17,
    'scout': 0.42,
}
DEFAULT_GROUND_RADIUS = 0.3

# Height (m) a ground robot's pose reports, by model: on hardware the mocap
# markers sit on top, so the pose is up there and not on the floor. Measured
# as the top of the deck in the model's mesh. Ground robots without a model
# get 0. Any robot can set its own with `pose: {z: ...}`.
MARKER_HEIGHTS = {
    'herover': 0.095,
    'scout': 0.227,
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
    model: str = None  # key of MODEL_MESHES, or None for no mesh
    radius: float = DEFAULT_GROUND_RADIUS  # m, footprint for map collisions
    z: float = 0.0  # m, height of the published pose (MARKER_HEIGHTS)


@dataclass(frozen=True)
class Crazyflie:
    name: str
    x: float
    y: float
    z: float
    gripper: bool = False  # starts with an open gripper on top (see hero_sim/gripper.py)
    radius: float = FOOTPRINT_RADII['crazyflie']  # m, footprint for map collisions


@dataclass(frozen=True)
class Tether:
    """Winch -> pulley on a ground robot -> ring on a Crazyflie -> object on the floor."""
    winch: tuple   # x, y, z (m): where the tether leaves the drum
    ugv: str       # ground robot carrying the first pulley (at its pose z)
    uav: str       # Crazyflie carrying the guide ring
    object: tuple  # x, y (m): start position on the floor
    goal: tuple    # x, y, radius (m)
    object_size: float = 0.3     # m, cube edge
    slack: float = 0.5           # m of tether beyond taut at the start
    max_reel_speed: float = 0.5  # m/s, clamp on /winch/cmd_rate

    @property
    def object_radius(self):
        """m, the object's footprint against obstacles: half its edge."""
        return self.object_size / 2.0


@dataclass(frozen=True)
class Scenario:
    ground_robots: tuple
    crazyflies: tuple
    sim: dict
    tether: Tether = None
    obstacles: tuple = ()  # (x, y, radius) pillars (m), see hero_sim/obstacles.py


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
    _check_keys(raw, ('ground_robots', 'crazyflies', 'sim', 'obstacles', 'tether'), 'top level')

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
    if not (isinstance(sim['attach'], bool) or sim['attach'] == 'auto'):
        raise ValueError(f'sim.attach must be auto, true or false, got {sim["attach"]!r}')
    if not isinstance(sim['collision'], bool):
        raise ValueError(f'sim.collision must be true or false, got {sim["collision"]!r}')
    band = sim['collision_z']
    if not (isinstance(band, (list, tuple)) and len(band) == 2
            and float(band[0]) < float(band[1])):
        raise ValueError(f'sim.collision_z must be [z_min, z_max], z_min < z_max, got {band!r}')
    sim['collision_z'] = (float(band[0]), float(band[1]))

    pillars = tuple(_pillar(i, o) for i, o in enumerate(_list(raw, 'obstacles')))
    tether = (_tether(raw['tether'], ground, cfs, pillars)
              if raw.get('tether') is not None else None)
    return Scenario(ground, cfs, sim, tether, pillars)


def attach_enabled(sim):
    """Whether to run sim_attach for this scenario's sim block.

    sim_attach implements hero_mrs_msgs/srv/AttachAgent, which only
    hero_mrs_control_stack calls, so hero_mrs_msgs is an optional dependency:
    'auto' runs it only when that package is installed, true requires it,
    false never runs it.
    """
    from importlib.util import find_spec
    available = find_spec('hero_mrs_msgs') is not None
    if sim['attach'] == 'auto':
        return available
    if sim['attach'] and not available:
        raise RuntimeError(
            'sim.attach is true but hero_mrs_msgs is not installed; build it from '
            'hero_mrs_control_stack, or set sim.attach to auto or false')
    return sim['attach']


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


def _pose(entry, where, axes, **defaults):
    pose = entry.get('pose') or {}
    _check_keys(pose, axes, f'{where}.pose')
    return [float(pose.get(a, defaults.get(a, 0.0))) for a in axes]


def _radius(entry, where, default):
    radius = float(entry.get('radius', default))
    if radius < 0.0:
        raise ValueError(f'{where}: radius must be >= 0, got {radius}')
    return radius


def _ground_robot(i, entry):
    where = f'ground_robots[{i}]'
    _check_keys(entry, ('name', 'pose', 'drive', 'model', 'radius'), where)
    name = entry.get('name')
    if not isinstance(name, str) or not name:
        raise ValueError(f'{where}: name is required')
    drive = str(entry.get('drive', 'omni')).lower()
    if drive not in DRIVES:
        raise ValueError(f'{where} ({name}): drive must be one of {DRIVES}, got {drive!r}')
    model = entry.get('model')
    if model is not None and model not in MODEL_MESHES:
        raise ValueError(f'{where} ({name}): model must be one of {sorted(MODEL_MESHES)} '
                         f'or left out, got {model!r}')
    x, y, z, yaw = _pose(entry, f'{where} ({name})', ('x', 'y', 'z', 'yaw'),
                         z=MARKER_HEIGHTS.get(model, 0.0))
    radius = _radius(entry, f'{where} ({name})', FOOTPRINT_RADII.get(model, DEFAULT_GROUND_RADIUS))
    return GroundRobot(name, x, y, yaw, drive, model, radius, z)


def _crazyflie(i, entry):
    name = f'cf_{i + 1}'
    where = f'crazyflies[{i}] ({name})'
    _check_keys(entry, ('pose', 'gripper', 'radius'), where)
    gripper = entry.get('gripper', False)
    if not isinstance(gripper, bool):
        raise ValueError(f'{where}: gripper must be true or false, got {gripper!r}')
    x, y, z = _pose(entry, where, ('x', 'y', 'z'))
    return Crazyflie(name, x, y, z, gripper, _radius(entry, where, FOOTPRINT_RADII['crazyflie']))


def _point(value, where, axes, **defaults):
    """A mapping of `axes` as a tuple of floats. Axes without a default are required."""
    value = value or {}
    _check_keys(value, axes, where)
    missing = [a for a in axes if a not in value and a not in defaults]
    if missing:
        raise ValueError(f'{where}: missing {missing}')
    return tuple(float(value.get(a, defaults.get(a))) for a in axes)


def _pillar(i, entry):
    where = f'obstacles[{i}]'
    x, y, r = _point(entry, where, ('x', 'y', 'radius'))
    if r <= 0.0:
        raise ValueError(f'{where}.radius must be > 0, got {r}')
    return x, y, r


def _tether(entry, ground, cfs, pillars):
    where = 'tether'
    if isinstance(entry, dict) and 'obstacles' in entry:
        raise ValueError(f'{where}.obstacles moved to the top-level obstacles: list, '
                         'which blocks the robots too')
    _check_keys(entry, ('winch', 'ugv', 'uav', 'object', 'goal', 'object_size',
                        'slack', 'max_reel_speed'), where)
    ugv = entry.get('ugv', ground[0].name if ground else None)
    if ugv not in {r.name for r in ground}:
        raise ValueError(f'{where}.ugv must name a ground robot, got {ugv!r}')
    uav = entry.get('uav', 'cf_1')
    if uav not in {c.name for c in cfs}:
        raise ValueError(f'{where}.uav must name a crazyflie (cf_1, ...), got {uav!r}')
    tether = Tether(
        winch=_point(entry.get('winch'), f'{where}.winch', ('x', 'y', 'z'), z=0.3),
        ugv=ugv,
        uav=uav,
        object=_point(entry.get('object'), f'{where}.object', ('x', 'y')),
        goal=_point(entry.get('goal'), f'{where}.goal', ('x', 'y', 'radius'), radius=0.4),
        object_size=float(entry.get('object_size', Tether.object_size)),
        slack=float(entry.get('slack', Tether.slack)),
        max_reel_speed=float(entry.get('max_reel_speed', Tether.max_reel_speed)),
    )
    for key in ('object_size', 'max_reel_speed'):
        if getattr(tether, key) <= 0.0:
            raise ValueError(f'{where}.{key} must be > 0, got {getattr(tether, key)}')
    if tether.slack < 0.0:
        raise ValueError(f'{where}.slack must be >= 0, got {tether.slack}')
    if tether.goal[2] <= 0.0:
        raise ValueError(f'{where}.goal.radius must be > 0, got {tether.goal[2]}')
    for i, (x, y, r) in enumerate(pillars):
        if math.hypot(tether.object[0] - x, tether.object[1] - y) < r + tether.object_radius:
            raise ValueError(f'{where}.object starts inside obstacles[{i}]')
    return tether


# ---------------------------------------------------------------- resolution

def share_dir():
    """Installed share directory of the hero_sim package (needs a sourced ROS env)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('hero_sim'))


def find_config(arg, root):
    """The scenario file `arg` names, under the config dir `root` (startup/_setup.sh does the same).

    Configs sit in root or in folders below it (config/tether/, config/mrs/):
      tether_demo.yaml         a bare filename, searched in root and every folder below.
                               Filenames are unique across folders; two matches raise
      tether/tether_demo.yaml  with a '/': a path if that file exists, else relative to root
    """
    root = Path(root)
    if '/' in arg:
        for path in (Path(arg), root / arg):
            if path.is_file():
                return path
        raise FileNotFoundError(f'hero_sim config not found: {arg} (as a path or under {root})')
    matches = sorted(p for p in root.rglob(arg) if p.is_file())
    if len(matches) > 1:
        names = ', '.join(str(p.relative_to(root)) for p in matches)
        raise ValueError(f'hero_sim config {arg} is in more than one folder: {names}; '
                         'name the folder too, e.g. tether/tether_demo.yaml. If a config '
                         'moved, the install may still hold its old copy: remove '
                         'build/hero_sim and install/hero_sim and rebuild')
    if not matches:
        raise FileNotFoundError(f'hero_sim config not found: {arg} (searched {root})')
    return matches[0]


def resolve_config(arg=''):
    """Turn a config argument into an existing file path.

    Empty falls back to $HERO_SIM_CONFIG, then DEFAULT_CONFIG. Names are found
    in the package's config/ dir and its folders (find_config); anything
    else with a '/' is a path.
    """
    arg = arg or os.environ.get(CONFIG_ENV, '') or DEFAULT_CONFIG
    if Path(arg).is_absolute():  # a path: no package lookup, so no sourced ROS env needed
        if not Path(arg).is_file():
            raise FileNotFoundError(f'hero_sim config not found: {arg}')
        return str(Path(arg).resolve())
    return str(find_config(arg, share_dir() / 'config').resolve())


def resolve_map(value):
    """Turn sim.map into an existing .bt path, or None when the map is disabled."""
    if not value:
        return None
    value = str(value)
    path = Path(value) if '/' in value else share_dir() / 'maps' / f'{value}.bt'
    if not path.is_file():
        raise FileNotFoundError(f'hero_sim map not found: {path}')
    return str(path)
