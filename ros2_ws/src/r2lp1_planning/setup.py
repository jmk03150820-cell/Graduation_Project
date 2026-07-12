import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'r2lp1_planning'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer='jmk',
    maintainer_email='jmk03150820@gmail.com',
    description=(
        'Phase 1 if-else planning node: turns module_integrate_node\'s '
        'synced chameleon_in JSON into an /autoware/control Twist.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2lp1_planning_node = r2lp1_planning.r2lp1_planning_node:main',
        ],
    },
)
