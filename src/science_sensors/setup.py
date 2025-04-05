from setuptools import find_packages, setup
import os
from glob import glob

package_name = "science_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aydan",
    maintainer_email="aj01cars@outlook.com",
    description="Nodes for science sensors",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gas_sensor = science_sensors.gas_sensor:main",
            "microscope_control = science_sensors.microscope_control:main",
        ],
    },
)
