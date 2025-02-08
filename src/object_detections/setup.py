from setuptools import find_packages, setup

package_name = 'object_detections'

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
    maintainer='erik',
    maintainer_email='erikcaell@gmail.com',
    description='Detect various objects in camera feeds to achieve rover competition tasks',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_mallet = object_detections.detect_mallet:main'
        ],
    },
)
