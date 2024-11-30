from __future__ import annotations # Import ColourProcessing as a type hint
from typing import Union

class PitchYaw:
    def __init__(self, p, y):
        self.pitch = p
        self.yaw = y

    def __str__(self):
        return f"Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}"

    def __repr__(self):
        return f"PitchYaw({self.pitch}, {self.yaw})"

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Point({self.x}, {self.y})"

class CameraUtil:
    def __init__(self, name: str, xRes, yRes, xFov, yFov, v4l_byid_name: str = None):
        self.name = name
        self.xRes = xRes
        self.yRes = yRes
        self.xFov = xFov
        self.yFov = yFov
        self.v4l_byid_name = v4l_byid_name
            
    def __str__(self):
        return f"CameraUtil('{self.name}', {self.xRes}, {self.yRes}, {self.xFov}, {self.yFov})"
    
    def pitchYawFromXY(self, point: Point) -> PitchYaw:
        aX = (point.x - self.xRes / 2.0) / (self.xRes / 2.0)
        aY = (point.y - self.yRes / 2.0) / (self.yRes / 2.0)
        
        yaw = aX * self.xFov / 2.0
        pitch = aY * self.yFov / 2.0
        
        return PitchYaw(pitch, yaw)
        
    def xyFromPitchYaw(self, pitchYaw: PitchYaw) -> Point:
        aX = pitchYaw.yaw * 2.0 / self.xFov
        aY = pitchYaw.pitch * 2.0 / self.yFov
        
        x = (aX * self.xRes / 2.0) + (self.xRes / 2.0)
        y = (aY * self.yRes / 2.0) + (self.yRes / 2.0)

        return Point(x, y)
    
class LinearUndistortion:
    def __init__(self, pitch_offset: float, pitch_slope: float, yaw_offset: float, yaw_slope: float, static_pitch_offset: float, static_yaw_offset):
        """
        """
        self._pitch_offset = pitch_offset
        self._pitch_slope = pitch_slope
        self._yaw_offset = yaw_offset
        self._yaw_slope = yaw_slope

        self._static_pitch_offset = static_pitch_offset
        self._static_yaw_offset = static_yaw_offset

    def __repr__(self):
        return f"LinearUndistortion(pitch_offset={self._pitch_offset}, pitch_slope={self._pitch_slope}, yaw_offset={self._yaw_offset}, yaw_slope={self._yaw_slope}, static_pitch_offset={self._static_pitch_offset}, static_yaw_offset={self._static_yaw_offset})"
    
    def undistort(self, pitchYaw: PitchYaw) -> PitchYaw:
        p = (pitchYaw.pitch - self._pitch_offset) * self._pitch_slope + pitchYaw.pitch + self._static_pitch_offset
        y = (pitchYaw.yaw - self._yaw_offset) * self._yaw_slope + pitchYaw.yaw + self._static_yaw_offset

        # p = pitchYaw.pitch * self._pitch_slope - self._pitch_offset
        # y = pitchYaw.yaw * self._yaw_slope - self._yaw_offset


        return PitchYaw(p=p, y=y)
    
    @classmethod    
    def from_string(linear_undistortion_class, python_eval: str) -> Union[LinearUndistortion, str]: 
        """
        Create a LinearUndistortion object from a string that can be evaluated to a LinearUndistortion object..

        Parameters:
            - python_eval (str): The string to eval into a LinearUndistortion object
        
        Returns:
            - Union[LinearUndistortion, str]: The LinearUndistortion object or an error message
        """
        try:
            obj = eval(python_eval)
        except Exception as e:
            return f"Error creating LinearUndistortion object. Error: {e}"
        
        if not isinstance(obj, LinearUndistortion):
            return f"Error creating LinearUndistortion object. Error: {obj} is not a LinearUndistortion object."

        return obj
