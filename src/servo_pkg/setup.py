from setuptools import find_packages, setup

package_name = 'servo_pkg'

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
    maintainer='darren',
    maintainer_email='darrenwallace368@gmail.com',
    description='Servo Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_servo_service = servo_pkg.USB_Servo:main',
            'usb_client = servo_pkg.USB_client:main'
        ],
    },
    py_modules=[
        'servo_pkg.maestro',  # Include maestro.py
    ],
)
