from setuptools import find_packages, setup


package_name = 'jetrover_kinematics'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['numpy', 'scipy', 'setuptools'],
    zip_safe=True,
    maintainer='luong',
    maintainer_email='luuluong2000hh@gmail.com',
    description=(
        'Pure forward and inverse kinematics for the JetRover 5-DOF arm.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
)
