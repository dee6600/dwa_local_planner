import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dwa_local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dee',
    maintainer_email='dee@todo.todo',
    description='DWA local planner (python) for demo',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_node = dwa_local_planner.dwa_node:main',
            'global_planner = dwa_local_planner.simple_global_planner:main',
        ],
    },
)
