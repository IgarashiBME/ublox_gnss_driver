import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ublox_gnss_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy',
    ],
    zip_safe=True,
    maintainer='kikai',
    maintainer_email='igarashi.jetson@gmail.com',
    description='u-blox GNSS ROS2 driver: UBX-NAV-PVT, HPPOSLLH, RELPOSNED parser with NTRIP support',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rtk_rover_node = ublox_gnss_driver.rtk_rover_node:main',
            'moving_base_node = ublox_gnss_driver.moving_base_node:main',
        ],
    },
)
