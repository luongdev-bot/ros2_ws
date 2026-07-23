import os
from glob import glob
from setuptools import setup

package_name = 'jetrover_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.config'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luong',
    maintainer_email='luuluong2000hh@gmail.com',
    description='Gazebo Sim physics simulation bringup for JetRover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_attacher = jetrover_gazebo.grasp_attacher:main',
            'sim_launcher_gui = jetrover_gazebo.sim_launcher_gui:main',
        ],
    },
)
