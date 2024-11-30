import cv2
from enum import IntEnum
from numpy import ndarray
from typing import Tuple, Any

HUE_MAX = 180
SATURATION_MAX = 255
VALUE_MAX = 255
RGB_MAX = 255

class WaitKeyStroke(IntEnum):
    ESC = 27

class BGR(IntEnum):
    BLUE = 0
    GREEN = 1
    RED = 2

def do_nothing(nothing: Any):
    pass

def imshow(window_name: str, image: ndarray):
    """
    Display an image on a window with the given name.
    Creates the window if it isn't already created with create_window.

    Parameters:
        window_name (str): The name for the window to display the image to
        image (ndarray): The image to display
    """
    cv2.imshow(window_name, image)

def imshow_scaled(window_name: str, image: ndarray, image_scaling: float):
    """
    Display an image on a window with the given name.
    Creates the window if it isn't already created with create_window.

    Image is scaled by the given float before displaying

    Parameters:
        window_name (str): The name for the window to display the image to
        image (ndarray): The image to display
    """
    image_scaled = cv2.resize(image, None, fx=image_scaling, fy=image_scaling, interpolation = cv2.INTER_LINEAR)
    cv2.imshow(window_name, image_scaled)
    

def inRange(image: ndarray, low: Tuple[int], high: Tuple[int]) -> ndarray:
    """
    Create a mask which has pixels at 0 or 255. A pixel goes to 255 when each 3rd index is within
    the ranges formed by low to high.

    For example, in BGR, a green pixel at image[100][100][1] must be within low[1] to high[1]
    Or for HSV, the hue of a pixel at image[200][200][0] must be within low[0] to high[0]

    Paramters:
        image (ndarray): The image to create a mask from
        low (Tuple[int]): The lower bounds of the range
        high (Tuple[int]): The higher bounds of the range

    Returns:
        ndarray: A 2-dimensional array where each pixel is either 0 or 255, forming the mask of the image.
    """
    return cv2.inRange(image, low, high)
    
