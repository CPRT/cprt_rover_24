from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'arm_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='christopherrusu1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_interpreter = arm_interface.trajectoryInterpreter:main',
            'joystick_arm_controller = arm_interface.joystickArmController:main',
            'joystick_science_controller = arm_interface.joystickScienceController:main',
        ],
    },
)
