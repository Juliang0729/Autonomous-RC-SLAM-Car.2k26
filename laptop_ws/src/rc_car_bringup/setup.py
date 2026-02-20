from setuptools import find_packages, setup

package_name = 'rc_car_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/laptop_bringup.launch.py',
            'launch/slam_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juliang29',
    maintainer_email='juliang29@todo.todo',
    description='RC Car bringup package for laptop',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)