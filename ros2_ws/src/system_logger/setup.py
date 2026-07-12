import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'system_logger'

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
        'system_logger_node: subscribes to /system/log and turns every '
        'pipeline node\'s JSON status/metric line into console output, an '
        'optional log file, and a periodic rolling summary.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_logger_node = system_logger.system_logger_node:main',
        ],
    },
)
