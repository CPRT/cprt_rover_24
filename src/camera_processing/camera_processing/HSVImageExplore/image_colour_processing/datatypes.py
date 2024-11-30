from numpy import ndarray
import numpy
from typing import Tuple
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper
