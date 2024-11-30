from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_processing'
submodules = [
    'camera_processing/zed_helper_files', 
    'camera_processing/HSVImageExplore/image_colour_processing',
    'camera_processing/zed_viewers/cv_viewer',
    'camera_processing/zed_viewers/ogl_viewer']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + submodules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erik',
    maintainer_email='erikcaell@gmail.com',
    description='Contains nodes to process camera data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_markers = camera_processing.aruco_markers:main',
            'display_image_locally = camera_processing.display_image_locally:main',
            'zed_node = camera_processing.zed_node:main',
            'hsv_image_explore = camera_processing.hsv_image_explore_node:main',
            'transform_zed_points_node = camera_processing.transform_zed_points_to_map:main',
            'publish_image = camera_processing.publish_image_node:main',
        ],
    },
)
