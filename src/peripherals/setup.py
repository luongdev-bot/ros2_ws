import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'peripherals'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luong',
    maintainer_email='luuluong2000hh@gmail.com',
    description='Teleoperation (keyboard + gamepad) for the JetRover Gazebo '
                'simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_key_control = peripherals.teleop_key_control:main',
            'joystick_control = peripherals.joystick_control:main',
        ],
    },
)
