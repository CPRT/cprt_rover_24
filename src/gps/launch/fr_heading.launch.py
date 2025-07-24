import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pyubx2 import SET, UBX_PROTOCOL, UBXMessage, UBXReader
from serial import Serial


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")
    with Serial("/dev/ /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30I1LY5-if00-port0", 115200) as stream:
        msg = UBXMessage(
            "CFG",
            "CFG-CFG",
            SET,
            clearMask=b"\x1f\x1f\x00\x00",  # clear everything
            loadMask=b"\x1f\x1f\x00\x00",  # reload everything
            devBBR=1,  # clear from battery-backed RAM
            devFlash=1,  # clear from flash memory
            devEEPROM=1,  # clear from EEPROM memory
        )
        stream.write(msg.serialize)

    Heading_config_file = os.path.join(config_dir, "Heading_config.ubx")
    heading = (
        "ubxload --port /dev/ /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30I1LY5-if00-port0 --baudrate 115200 --infile " + Heading_config_file
    )
    os.system(heading)

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gps",
                executable="heading_pub_node",
                name="gps_heading_node",
                parameters=[
                    {"frame_id": "gps"},
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 115200},
                    {"Device": "/dev/ /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30I1LY5-if00-port0"},
                ],
            ),
        ]
    )