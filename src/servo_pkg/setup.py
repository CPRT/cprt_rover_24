from setuptools import find_packages, setup
import os
from glob import glob

package_name = "servo_pkg"

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
    maintainer="darren",
    maintainer_email="darrenwallace368@gmail.com",
    description="Servo Node",
    license="Apache-2.0",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "USB_Servo = servo_pkg.USB_Servo:main",
            "servo_client = servo_pkg.servo_client:main",
            "i2c_Servo = servo_pkg.i2c_Servo:main",
        ],
    },
    py_modules=[
        "servo_pkg.maestro",  # Include maestro.py
    ],
)
