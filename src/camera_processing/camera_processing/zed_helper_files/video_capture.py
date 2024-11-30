import cv2
import threading
import re
import cv2
import os
import time
from time import sleep

"""
Captures video from a camera and always gives the latest frame.
"""
class VideoCapture:
    def __init__(self, cam):
        self.xRes = cam.value.xRes
        self.yRes = cam.value.yRes

        try:
            self.cap = cv2.VideoCapture(VideoCapture._find_video_index(cam.value.v4l_byid_name))
        except Exception as e:
            raise ValueError(f"Could not find the video index for the camera. Error: {e}")
        
        self.cam_type = cam
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.xRes)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.yRes)

        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    def _find_video_index(v4l_byid_name: str):
        if v4l_byid_name is None or len(v4l_byid_name) == 0:
            raise ValueError("Must provide a v4l_byid_name name to find the video index")
        
        v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

        if not os.path.exists(v4l_byid_name):
            raise ValueError("Path doesn't exist when following the v4l_byid_name")
        
        device_path = os.path.realpath(v4l_byid_name)
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        if not info:
            raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
        
        return int(info.group(1))

    def _reader(self):
        """
        Grab frames as soon as they are available
        """
        while True:
            with self.lock:
                ret = self.cap.grab()
            if not ret:
                break
            sleep(0.025)

    def read(self):
        """
        Retrieve the latest frame.
        """
        start = time.time()
        with self.lock:
            _, frame = self.cap.retrieve()

        delta = time.time() - start
        return frame, delta