import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'boids_turtlesim'

setup(
    name=package_name,
    version='0.2.0',
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
    description='Boids flocking swarm on ROS 2 turtlesim (SDD v2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boid_controller = boids_turtlesim.boid_controller:main',
            'swarm_spawner = boids_turtlesim.swarm_spawner:main',
        ],
    },
)
