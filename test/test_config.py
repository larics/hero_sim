from pathlib import Path

import pytest
import yaml

from conftest import CONFIG_DIR
from hero_sim import config


def parse(text):
    return config.parse(yaml.safe_load(text))


def test_minimal_and_defaults():
    s = parse("""
ground_robots:
  - name: scout_1
crazyflies:
  - pose: {x: 1.0, y: 2.0, z: 0.03}
  - pose: {x: 3.0}
""")
    assert s.ground_robots == (config.GroundRobot('scout_1', 0.0, 0.0, 0.0, 'omni'),)
    assert s.crazyflies == (config.Crazyflie('cf_1', 1.0, 2.0, 0.03),
                            config.Crazyflie('cf_2', 3.0, 0.0, 0.0))
    assert s.sim == config.SIM_DEFAULTS


def test_empty_file():
    s = config.parse(None)
    assert s.ground_robots == () and s.crazyflies == ()


def test_sim_override_keeps_other_defaults():
    s = parse('sim: {cf_rate: 20, map: null, rviz: false}')
    assert s.sim['cf_rate'] == 20.0
    assert s.sim['map'] is None and s.sim['rviz'] is False
    assert s.sim['ground_rate'] == config.SIM_DEFAULTS['ground_rate']


def test_ground_robot_model_is_optional():
    s = parse('ground_robots: [{name: h, model: herover}, {name: r}]')
    assert [r.model for r in s.ground_robots] == ['herover', None]


def test_model_meshes_exist():
    for rel in config.MODEL_MESHES.values():
        assert (CONFIG_DIR.parent / 'models' / rel).is_file(), rel


def test_footprint_radius_defaults_by_model_and_overrides():
    s = parse('''
ground_robots:
  - {name: s, model: scout}
  - {name: h, model: herover}
  - {name: r}
  - {name: w, model: scout, radius: 0.5}
crazyflies: [{pose: {}}, {pose: {}, radius: 0.2}]
''')
    assert [r.radius for r in s.ground_robots] == [0.42, 0.17, config.DEFAULT_GROUND_RADIUS, 0.5]
    assert [cf.radius for cf in s.crazyflies] == [config.FOOTPRINT_RADII['crazyflie'], 0.2]


def test_ground_pose_z_defaults_to_marker_height_and_overrides():
    s = parse('''
ground_robots:
  - {name: s, model: scout}
  - {name: h, model: herover, pose: {x: 1.0}}
  - {name: r}
  - {name: w, model: scout, pose: {z: 0.3}}
''')
    assert [r.z for r in s.ground_robots] == [config.MARKER_HEIGHTS['scout'],
                                               config.MARKER_HEIGHTS['herover'], 0.0, 0.3]


def test_collision_z_becomes_a_float_pair():
    assert parse('sim: {collision_z: [0, 3]}').sim['collision_z'] == (0.0, 3.0)


def test_crazyflie_gripper_is_optional():
    s = parse('crazyflies: [{pose: {}, gripper: true}, {pose: {}}]')
    assert [cf.gripper for cf in s.crazyflies] == [True, False]


def test_obstacles_are_optional_and_parsed():
    assert parse('{}').obstacles == ()
    s = parse('obstacles: [{x: 1, y: 2, radius: 0.5}, {x: -1, y: 0, radius: 0.1, height: 0.4}]')
    assert s.obstacles == ((1.0, 2.0, 0.5, config.DEFAULT_PILLAR_HEIGHT), (-1.0, 0.0, 0.1, 0.4))
    assert s.obstacles[1].height == 0.4


def test_obstacles_replace_the_default_map():
    assert parse('{}').sim['map'] == config.SIM_DEFAULTS['map']
    assert parse('obstacles: [{x: 1, y: 2, radius: 0.5}]').sim['map'] is None
    assert parse('obstacles: [{x: 1, y: 2, radius: 0.5}]\nsim: {map: null}').sim['map'] is None


def test_drive_is_case_insensitive():
    s = parse('ground_robots: [{name: h, drive: DIFF}]')
    assert s.ground_robots[0].drive == 'diff'


@pytest.mark.parametrize('text, match', [
    ('robots: []', "unknown key.*'robots'"),
    ('ground_robots: [{name: a, pose: {x: 1, roll: 2}}]', r"ground_robots\[0\] \(a\)\.pose: unknown key.*'roll'"),
    ('ground_robots: [{name: a, speed: 1}]', "unknown key.*'speed'"),
    ('ground_robots: [{pose: {x: 1}}]', 'name is required'),
    ('ground_robots: [{name: a, drive: tank}]', 'drive must be one of'),
    ('ground_robots: [{name: a, model: tank}]', 'model must be one of'),
    ('ground_robots: [{name: a}, {name: a}]', 'duplicate ground robot names'),
    ('ground_robots: [{name: cf_1}]\ncrazyflies: [{pose: {}}]', 'clash with crazyflie names'),
    ('crazyflies: [{pose: {yaw: 1}}]', r"crazyflies\[0\] \(cf_1\)\.pose: unknown key"),
    ('crazyflies: {pose: {}}', 'crazyflies must be a list'),
    ('crazyflies: [{gripper: yes please}]', 'gripper must be true or false'),
    ('sim: {cf_hz: 5}', "sim: unknown key.*'cf_hz'"),
    ('sim: {ground_rate: 0}', 'ground_rate must be > 0'),
    ('sim: {cf_battery_drain: -1}', 'cf_battery_drain must be >= 0'),
    ('sim: {attach: maybe}', 'attach must be auto, true or false'),
    ('sim: {collision: maybe}', 'collision must be true or false'),
    ('sim: {collision_z: 1.0}', r'collision_z must be \[z_min, z_max\]'),
    ('sim: {collision_z: [2.0, 1.0]}', r'collision_z must be \[z_min, z_max\]'),
    ('ground_robots: [{name: a, radius: -1}]', r'\(a\): radius must be >= 0'),
    ('crazyflies: [{radius: -0.1}]', r'\(cf_1\): radius must be >= 0'),
    ('obstacles: {x: 0}', 'obstacles must be a list'),
    ('obstacles: [{x: 0, y: 0}]', r"obstacles\[0\]: missing \['radius'\]"),
    ('obstacles: [[0, 0, 1]]', r'obstacles\[0\] must be a mapping'),
    ('obstacles: [{x: 0, y: 0, radius: 1, z: 2}]', r"obstacles\[0\]: unknown key.*'z'"),
    ('obstacles: [{x: 0, y: 0, radius: 1}, {x: 0, y: 0, radius: 0}]',
     r'obstacles\[1\].radius must be > 0'),
    ('obstacles: [{x: 0, y: 0, radius: 1, height: 0}]', r'obstacles\[0\].height must be > 0'),
    ('obstacles: [{x: 0, y: 0, radius: 1}]\nsim: {map: icuas26_1}',
     'sim.map and obstacles: are both set'),
])
def test_rejects_bad_config(text, match):
    with pytest.raises(ValueError, match=match):
        parse(text)


@pytest.mark.parametrize('attach, installed, expected', [
    ('auto', True, True),
    ('auto', False, False),
    (True, True, True),
    (False, True, False),
    (False, False, False),
])
def test_attach_enabled(monkeypatch, attach, installed, expected):
    monkeypatch.setattr('importlib.util.find_spec',
                        lambda name: object() if installed else None)
    assert config.attach_enabled({'attach': attach}) is expected


def test_attach_true_without_msgs_fails_loudly(monkeypatch):
    monkeypatch.setattr('importlib.util.find_spec', lambda name: None)
    with pytest.raises(RuntimeError, match='hero_mrs_msgs is not installed'):
        config.attach_enabled({'attach': True})


def test_load_names_the_file(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('nope: 1\n')
    with pytest.raises(ValueError, match='bad.yaml'):
        config.load(path)


def test_resolve_config_path_and_env(tmp_path, monkeypatch):
    path = tmp_path / 'a.yaml'
    path.write_text('{}\n')
    assert config.resolve_config(str(path)) == str(path)
    monkeypatch.setenv(config.CONFIG_ENV, str(path))
    assert config.resolve_config('') == str(path)
    with pytest.raises(FileNotFoundError):
        config.resolve_config(str(tmp_path / 'missing.yaml'))


def test_resolve_map_disabled_and_path(tmp_path):
    assert config.resolve_map(None) is None
    assert config.resolve_map('') is None
    bt = tmp_path / 'm.bt'
    bt.write_bytes(b'')
    assert config.resolve_map(str(bt)) == str(bt)


SHIPPED = sorted(CONFIG_DIR.rglob('*.yaml'))


@pytest.mark.parametrize('path', SHIPPED, ids=lambda p: str(p.relative_to(CONFIG_DIR)))
def test_shipped_configs_load(path):
    config.load(path)


def test_shipped_config_names_are_unique_across_folders():
    names = [p.name for p in SHIPPED]
    assert sorted({n for n in names if names.count(n) > 1}) == []
    assert any(p.parent != CONFIG_DIR for p in SHIPPED)  # the folders are searched at all


@pytest.fixture()
def config_tree(tmp_path):
    """config/ with a.yaml on top, tether/b.yaml, and dup.yaml in two folders."""
    root = tmp_path / 'config'
    for rel in ('a.yaml', 'tether/b.yaml', 'mrs/dup.yaml', 'tether/dup.yaml'):
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_text('{}\n')
    return root


def test_find_config_by_name_in_any_folder(config_tree):
    assert config.find_config('a.yaml', config_tree) == config_tree / 'a.yaml'
    assert config.find_config('b.yaml', config_tree) == config_tree / 'tether' / 'b.yaml'
    assert config.find_config('tether/b.yaml', config_tree) == config_tree / 'tether' / 'b.yaml'
    assert config.find_config('mrs/dup.yaml', config_tree) == config_tree / 'mrs' / 'dup.yaml'


def test_find_config_prefers_an_existing_path(config_tree, tmp_path, monkeypatch):
    (tmp_path / 'tether').mkdir()
    (tmp_path / 'tether' / 'b.yaml').write_text('{}\n')
    monkeypatch.chdir(tmp_path)
    assert config.find_config('tether/b.yaml', config_tree) == Path('tether/b.yaml')
    assert config.find_config(str(tmp_path / 'config' / 'a.yaml'), config_tree).is_file()


def test_find_config_errors(config_tree):
    with pytest.raises(ValueError, match=r'dup.yaml is in more than one folder: mrs/dup.yaml, '
                                         r'tether/dup.yaml'):
        config.find_config('dup.yaml', config_tree)
    with pytest.raises(FileNotFoundError, match='nope.yaml'):
        config.find_config('nope.yaml', config_tree)
    with pytest.raises(FileNotFoundError, match='mrs/b.yaml'):
        config.find_config('mrs/b.yaml', config_tree)  # wrong folder


TETHER_ROBOTS = 'ground_robots: [{name: g1}, {name: g2}]\ncrazyflies: [{pose: {}}, {pose: {}}]\n'
TETHER_MIN = '{winch: {x: -1, y: 0}, object: {x: 1, y: 2}, goal: {x: 3, y: 4}'


def test_tether_is_optional():
    assert parse(TETHER_ROBOTS).tether is None


def test_tether_defaults():
    t = parse(TETHER_ROBOTS + f'tether: {TETHER_MIN}}}').tether
    assert (t.ugv, t.uav) == ('g1', 'cf_1')
    assert t.winch == (-1.0, 0.0, 0.3)
    assert t.object == (1.0, 2.0)
    assert t.goal == (3.0, 4.0, 0.4)
    assert (t.object_size, t.object_radius, t.slack, t.max_reel_speed) == (0.3, 0.15, 0.5, 0.5)


def test_tether_overrides():
    t = parse(TETHER_ROBOTS + f'tether: {TETHER_MIN}, ugv: g2, uav: cf_2, '
                              'object_size: 0.2}').tether
    assert (t.ugv, t.uav, t.object_radius) == ('g2', 'cf_2', 0.1)


@pytest.mark.parametrize('extra, match', [
    (', rope: 1', "tether: unknown key.*'rope'"),
    (', ugv: nope', 'tether.ugv must name a ground robot'),
    (', uav: cf_9', r'tether.uav must name a crazyflie'),
    (', goal: {x: 3, y: 4, z: 0}', r"tether.goal: unknown key.*'z'"),
    (', goal: {x: 3, y: 4, radius: 0}', 'tether.goal.radius must be > 0'),
    (', object: {x: 1}', r"tether.object: missing \['y'\]"),
    (', obstacles: [{x: 0, y: 0, radius: 1}]', 'tether.obstacles moved to the top-level'),
    (', object_size: 0', 'tether.object_size must be > 0'),
    (', max_reel_speed: -1', 'tether.max_reel_speed must be > 0'),
    (', slack: -1', 'tether.slack must be >= 0'),
])
def test_rejects_bad_tether(extra, match):
    with pytest.raises(ValueError, match=match):
        parse(TETHER_ROBOTS + f'tether: {TETHER_MIN}{extra}}}')


def test_tether_object_must_start_clear_of_pillars():
    clear = TETHER_ROBOTS + 'obstacles: [{x: 1.4, y: 2, radius: 0.2}]\n'
    assert parse(clear + f'tether: {TETHER_MIN}}}').obstacles == ((1.4, 2.0, 0.2, 2.0),)
    inside = TETHER_ROBOTS + 'obstacles: [{x: 0, y: 0, radius: 0.1}, {x: 1.3, y: 2, radius: 0.2}]'
    with pytest.raises(ValueError, match=r'tether.object starts inside obstacles\[1\]'):
        parse(inside + f'\ntether: {TETHER_MIN}}}')


@pytest.mark.parametrize('text, match', [
    ('tether: [1]', 'tether must be a mapping'),
    ('tether: {winch: {x: 0, y: 0}, object: {x: 1, y: 0}, goal: {x: 2, y: 0}}',
     'tether.ugv must name a ground robot'),
    ('ground_robots: [{name: g1}]\n'
     'tether: {winch: {x: 0, y: 0}, object: {x: 1, y: 0}, goal: {x: 2, y: 0}}',
     'tether.uav must name a crazyflie'),
    (TETHER_ROBOTS + 'tether: {object: {x: 1, y: 0}, goal: {x: 2, y: 0}}',
     r"tether.winch: missing \['x', 'y'\]"),
])
def test_rejects_tether_without_its_parts(text, match):
    with pytest.raises(ValueError, match=match):
        parse(text)
