from setuptools import find_packages, setup

package_name = 'costmap_seeding'

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
    description='Seed the nav2 costmap with elevation data from a online elevation datasets',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SeedCostmapCanElevation = costmap_seeding.SeedCostmapCanElevation:main'
        ],
    },
)
