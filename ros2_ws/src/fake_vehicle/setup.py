import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fake_vehicle'

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
        'Fake vehicle simulator: rate-limited speed/steer tracking of '
        'vehicle_interface_node\'s /vehicle/command, republished as '
        '/vehicle/raw_status.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_vehicle_node = fake_vehicle.fake_vehicle_node:main',
        ],
    },
)
