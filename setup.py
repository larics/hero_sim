from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'hero_sim'

# RViz meshes: models/<name>/*.{dae,glb,stl}. models/source/ (raw files) is not installed.
model_files = [
    (f'share/{package_name}/{d}', [f for ext in ('dae', 'glb', 'stl') for f in glob(f'{d}/*.{ext}')])
    for d in sorted(glob('models/*'))
    if os.path.isdir(d) and os.path.basename(d) != 'source'
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*.bt')),
        *model_files,
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_server = hero_sim.crazyflie_server:main',
            'ground_robot_sim = hero_sim.ground_robots:main',
            'sim_attach = hero_sim.attach:main',
            'robot_models = hero_sim.robot_models:main',
            'tether_sim = hero_sim.tether_sim:main',
        ],
    },
)
