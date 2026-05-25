from setuptools import setup

package_name = 'pendulum_swing'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='倒立摆起摆控制仿真（基于 ROS2 + Gazebo）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swing_up_controller = pendulum_swing.swing_up_controller:main',
        ],
    },
)
