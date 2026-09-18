from setuptools import setup
import os, glob

package_name = 'rumblex_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/teleop_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Stein',
    maintainer_email='stein.robotics@gmail.com',
    description='Teleoperation (joystick, remote control) for the RumbleX hexapod platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_teleop = rumblex_teleop.node_teleop:main',
        ],
    },
)
