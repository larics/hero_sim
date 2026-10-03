from glob import glob

from setuptools import find_packages, setup

package_name = 'hero_sim'

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
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_server = hero_sim.crazyflie_server:main',
            'ground_robot_sim = hero_sim.ground_robots:main',
            'sim_attach = hero_sim.attach:main',
        ],
    },
)
