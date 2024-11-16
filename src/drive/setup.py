from setuptools import setup
import os
from glob import glob
package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = drive.drive_node:main',
            'drive_station = drive.drive_station:main',
            'joystick_drive = drive.joystick_drive:main',
            "roboclaw_node = drive.roboclaw_node:main",
            "joystick_breakout = drive.joystick_breakout:main",
        ],
    },
)