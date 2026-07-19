from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luong',
    maintainer_email='luuluong2000hh@gmail.com',
    description='Teach, store and replay robot-arm action groups on Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_motion_server = arm_motion.infrastructure.ros.motion_node:main',
            'arm_motion_editor = arm_motion.presentation.qt_editor:main',
        ],
    },
)
