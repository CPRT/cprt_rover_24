#!/usr/bin/env python3
import io
import os
import pwd
import time
import numpy as np
import rclpy
from rclpy.node import Node
from science_sensors.stellarnet_driverLibs import stellarnet_driver3 as sn
from interfaces.srv import Raman
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]

CSV_DIR = WORKSPACE_ROOT / "raman"
CSV_DIR.mkdir(exist_ok=True) 

#This saves the CSV files to the raman directory in the workspace and returns a csv in the response
#here is how to call this stupid ass service since it runs as root
'''

sudo -E bash -c '
  source /opt/ros/humble/setup.bash
  source ~/cprt_rover_24/install/setup.bash
  ros2 service call /get_raman_spectrum interfaces/srv/Raman \
    "{inittime: 500, scansavg: 1, smoothing: 1}"
'

'''


class RamanServ(Node):
    def __init__(self) -> None:
        super().__init__("raman")

        version = sn.version()
        self.get_logger().info(f"StellarNet driver version: {version}")

        self.spectrometer, self.wav = sn.array_get_spec(0)
        self.get_logger().info(f"Device ID: {sn.getDeviceId(self.spectrometer)}")
        sn.ext_trig(self.spectrometer, True)

        self.srv = self.create_service(
            Raman,
            "get_raman_spectrum",
            self.handle_raman_request,
        )
        self.get_logger().info("Raman service ready")

    def handle_raman_request(self, request: Raman.Request,
                             response: Raman.Response) -> Raman.Response:
        self.get_logger().info(
            f"Request  int={request.inittime} ms  "
            f"avg={request.scansavg}  smooth={request.smoothing}"
        )
        if(Raman.Request.laser):
            #turn on or off the laser using the GPIO relay/send ros message to do so. Maybe put a wait if its swapping states before getting spect
            pass
            

        data = self._get_spectrum(request.inittime,
                                  request.scansavg,
                                  request.smoothing)

        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{CSV_DIR}/raman_{ts}.csv"
        np.savetxt(filename, data, delimiter=",", fmt="%.6f")
        _chown_to_invoking_user(filename)

        buf = io.StringIO()
        np.savetxt(buf, data, delimiter=",", fmt="%.6f")
        response.csv = buf.getvalue()

        sn.reset(self.spectrometer)
        self.get_logger().info("Spectrum captured and sent")
        self.get_logger().info(f"Saved to {filename}")
        return response

    def _get_spectrum(self, inttime: int, scansavg: int, smooth: int):
        """Grab one spectrum (private helper)."""
        self.spectrometer["device"].set_config(
            int_time=inttime,
            scans_to_avg=scansavg,
            x_smooth=smooth,
        )
        return sn.array_spectrum(self.spectrometer, self.wav)

def _chown_to_invoking_user(path: str) -> None:
    """Make the output file belong to the original user (not root)."""
    user = os.getenv("SUDO_USER") or os.getenv("USER")
    if not user:
        return
    try:
        pw = pwd.getpwnam(user)
        os.chown(path, pw.pw_uid, pw.pw_gid)
    except (KeyError, PermissionError):
        pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RamanServ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
