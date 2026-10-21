"""Shared fixtures. ROS fixtures skip the test when rclpy isn't importable."""
from pathlib import Path

import pytest

CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'


@pytest.fixture(scope='session')
def rclpy_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(rclpy_context):
    from rclpy.node import Node
    n = Node('hero_sim_test')
    yield n
    n.destroy_node()


@pytest.fixture()
def scenario_file(tmp_path, monkeypatch):
    """Write a scenario YAML and point HERO_SIM_CONFIG at it. Returns a writer."""
    def write(text):
        path = tmp_path / 'scenario.yaml'
        path.write_text(text)
        monkeypatch.setenv('HERO_SIM_CONFIG', str(path))
        return path
    return write


def duration(sec, nanosec=0):
    from builtin_interfaces.msg import Duration
    return Duration(sec=sec, nanosec=nanosec)


def wall_map(max_radius=0.5):
    """ObstacleMap with a 1 m long wall along y, face at x = 1.0, 0.1 m thick, on a 0.1 m grid."""
    import numpy as np
    from hero_sim.obstacles import ObstacleMap
    occupied = np.zeros((10, 2), bool)
    occupied[:, 0] = True
    return ObstacleMap(occupied, 1.0, -0.5, 0.1, max_radius)
