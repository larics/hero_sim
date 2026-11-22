"""launch/octomap.launch.py: the map nodes each kind of scenario starts."""
import importlib.util
import tempfile

import pytest
import yaml

from conftest import CONFIG_DIR

MAPS_DIR = CONFIG_DIR.parent / 'maps'
MAZE = MAPS_DIR / 'simple_maze_4x4' / 'simple_maze_4x4.yaml'
OCTOMAP = ['octomap_server_node']
MAP_SERVER = ['map_server', 'lifecycle_manager']


@pytest.fixture()
def map_nodes(tmp_path, monkeypatch, scenario_file):
    """Run octomap.launch.py's _actions on a scenario; returns (executables, out dir)."""
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'octomap_launch', CONFIG_DIR.parent / 'launch' / 'octomap.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(tempfile, 'gettempdir', lambda: str(tmp_path / 'tmp'))

    def run(text):
        context = LaunchContext()
        context.launch_configurations['config'] = str(scenario_file(text))
        actions = module._actions(context)
        return [a.node_executable for a in actions if isinstance(a, Node)]
    return run, tmp_path / 'tmp' / 'hero_sim'


def test_image_map_gets_map_server_on_map(map_nodes):
    run, out = map_nodes
    assert run(f'sim: {{map: {MAZE}}}\n') == OCTOMAP + MAP_SERVER
    meta = yaml.safe_load((out / 'scenario_map.yaml').read_text())
    assert meta['image'] == str((MAZE.parent / 'simple_maze_4x4.bmp').resolve())


def test_image_map_without_walls_still_gets_map(map_nodes, tmp_path):
    Image = pytest.importorskip('PIL.Image')
    Image.new('L', (4, 4), 255).save(tmp_path / 'blank.png')
    (tmp_path / 'blank.yaml').write_text('image: blank.png\nresolution: 0.1\norigin: [0, 0, 0]\n')
    run, _ = map_nodes
    assert run(f'sim: {{map: {tmp_path / "blank.yaml"}}}\n') == MAP_SERVER  # no octomap


@pytest.mark.parametrize('text', [
    f'sim: {{map: {MAPS_DIR / "icuas26_1.bt"}}}\n',
    'obstacles: [{x: 1.0, y: 0.0, radius: 0.2}]\n',
], ids=['octomap', 'pillars'])
def test_octomaps_and_pillars_get_no_map_server(map_nodes, text):
    run, out = map_nodes
    assert run(text) == OCTOMAP
    assert not (out / 'scenario_map.yaml').exists()


def test_no_map_no_nodes(map_nodes):
    run, _ = map_nodes
    assert run('sim: {map: null}\n') == []
