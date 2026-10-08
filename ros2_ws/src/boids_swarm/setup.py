import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'boids_swarm'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lexho',
    maintainer_email='lexho2005@gmail.com',
    description='Boids swarm cooperative pursuit game (SDD v3+v4): '
                'pygame world + ROS 2 per-agent controllers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pygame_sim = boids_swarm.pygame_sim_node:main',
            'boid_controller = boids_swarm.boid_controller_node:main',
            'target_controller = boids_swarm.target_controller_node:main',
            'nav2_bridge = boids_swarm.nav2_bridge:main',
        ],
    },
)
