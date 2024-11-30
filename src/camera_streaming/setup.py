from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_streaming'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cprt',
    maintainer_email='softlead@curovers.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jpegPoC = camera_streaming.jpegPoC:main',
            'webrtc_node = camera_streaming.webrct_node:main'
        ],
    },
)
