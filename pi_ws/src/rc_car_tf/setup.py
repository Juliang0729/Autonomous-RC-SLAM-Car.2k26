from setuptools import find_packages, setup

package_name = 'rc_car_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juliang29',
    maintainer_email='juliang29@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_tf_node = rc_car_tf.robot_tf_node:main',
            'fake_odom_node = rc_car_tf.fake_odom_node:main',
        ],
    },
)
