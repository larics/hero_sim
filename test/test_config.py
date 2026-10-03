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


def test_drive_is_case_insensitive():
    s = parse('ground_robots: [{name: h, drive: DIFF}]')
    assert s.ground_robots[0].drive == 'diff'


@pytest.mark.parametrize('text, match', [
    ('robots: []', "unknown key.*'robots'"),
    ('ground_robots: [{name: a, pose: {x: 1, z: 2}}]', r"ground_robots\[0\] \(a\)\.pose: unknown key.*'z'"),
    ('ground_robots: [{name: a, speed: 1}]', "unknown key.*'speed'"),
    ('ground_robots: [{pose: {x: 1}}]', 'name is required'),
    ('ground_robots: [{name: a, drive: tank}]', 'drive must be one of'),
    ('ground_robots: [{name: a, model: tank}]', 'model must be one of'),
    ('ground_robots: [{name: a}, {name: a}]', 'duplicate ground robot names'),
    ('ground_robots: [{name: cf_1}]\ncrazyflies: [{pose: {}}]', 'clash with crazyflie names'),
    ('crazyflies: [{pose: {yaw: 1}}]', r"crazyflies\[0\] \(cf_1\)\.pose: unknown key"),
    ('crazyflies: {pose: {}}', 'crazyflies must be a list'),
    ('sim: {cf_hz: 5}', "sim: unknown key.*'cf_hz'"),
    ('sim: {ground_rate: 0}', 'ground_rate must be > 0'),
    ('sim: {cf_battery_drain: -1}', 'cf_battery_drain must be >= 0'),
    ('sim: {attach: maybe}', 'attach must be auto, true or false'),
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


@pytest.mark.parametrize('path', sorted(CONFIG_DIR.glob('*.yaml')), ids=lambda p: p.name)
def test_shipped_configs_load(path):
    config.load(path)
