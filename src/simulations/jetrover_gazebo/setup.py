import os
from glob import glob
from setuptools import setup

package_name = 'jetrover_gazebo'
house_model_root = os.path.join('models', 'turtlebot3_house')
house_model_data_files = []

for source_dir, dirnames, filenames in os.walk(house_model_root):
    dirnames.sort()
    if not filenames:
        continue
    relative_dir = os.path.relpath(source_dir, house_model_root)
    destination_dir = os.path.join('share', 'turtlebot3_house')
    if relative_dir != '.':
        destination_dir = os.path.join(destination_dir, relative_dir)
    house_model_data_files.append((
        destination_dir,
        [os.path.join(source_dir, filename) for filename in sorted(filenames)],
    ))

if not house_model_data_files:
    raise RuntimeError(
        'Required TurtleBot3 house model asset tree is missing or empty: '
        + house_model_root
    )

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
    ] + house_model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luong',
    maintainer_email='luuluong2000hh@gmail.com',
    description='Gazebo Sim physics simulation bringup for JetRover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_wanderer = jetrover_gazebo.block_wanderer:main',
            'grasp_attacher = jetrover_gazebo.grasp_attacher:main',
            'sim_launcher_gui = jetrover_gazebo.sim_launcher_gui:main',
        ],
    },
)
