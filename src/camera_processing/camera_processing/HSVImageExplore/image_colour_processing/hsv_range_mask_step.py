import cv2
from numpy import ndarray
import numpy as np
from typing import Optional
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    from process_steps import MaskStep
    from datatypes import HSVRange
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper
    from .process_steps import MaskStep
    from .datatypes import HSVRange

class HSVRangeMaskStep(MaskStep):
    hh = 'Hue High'
    hl = 'Hue Low'
    sh = 'Saturation High'
    sl = 'Saturation Low'
    vh = 'Value High'
    vl = 'Value Low'
    
    def __init__(self, window_name: str, hsv_range: HSVRange, return_mask: bool = False, copy_start_values: bool = False, invert_hue: bool = False):
        super().__init__(window_name)
        self._hsv_range = hsv_range
        self._return_mask = return_mask
        self._copy_start_values = copy_start_values
        self._invert_hue = invert_hue

    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """

        # Convert to HSV from BGR, and make gray for display
        hsv_image = cv2.cvtColor(process_img, cv2.COLOR_BGR2HSV)

        if self._return_mask:
            # Apply the range on a mask
            new_process_image = self._hsv_range.apply_range(hsv_image, self._invert_hue)

        else:
            # Apply the range on a mask
            mask = self._hsv_range.apply_range(hsv_image, self._invert_hue)

            # Apply the mask on the process image
            new_process_image = cv2.bitwise_and(process_img, process_img, mask=mask)

        if self._is_display_active:
            self._hsv_range.low.hue = cv2.getTrackbarPos(HSVRangeMaskStep.hl, self._window_name)
            self._hsv_range.high.hue = cv2.getTrackbarPos(HSVRangeMaskStep.hh, self._window_name)
            self._hsv_range.low.saturation = cv2.getTrackbarPos(HSVRangeMaskStep.sl, self._window_name)
            self._hsv_range.high.saturation = cv2.getTrackbarPos(HSVRangeMaskStep.sh, self._window_name)
            self._hsv_range.low.value = cv2.getTrackbarPos(HSVRangeMaskStep.vl, self._window_name)
            self._hsv_range.high.value = cv2.getTrackbarPos(HSVRangeMaskStep.vh, self._window_name)

            if self._return_mask:
                mask_img = np.zeros(original_img.shape, original_img.dtype)
                mask_img[:, :] = (255, 255, 255)

                print(f"Shape of mask_img: {original_img.shape}")
                print(f"Shape of new_process_images: {new_process_image.shape}")

                display_process_img = cv2.bitwise_and(mask_img, mask_img, mask=new_process_image)
            else:
                display_process_img = new_process_image
                
            images_stacked_horizontally = np.hstack([original_img, display_process_img])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        if self._copy_start_values:
            self._hsv_range = deepcopy(self._hsv_range)
            self._copy_start_values = False

        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(HSVRangeMaskStep.hl, self._window_name, self._hsv_range.low.hue, cv2_helper.HUE_MAX, cv2_helper.do_nothing) 
        cv2.createTrackbar(HSVRangeMaskStep.hh, self._window_name, self._hsv_range.high.hue, cv2_helper.HUE_MAX, cv2_helper.do_nothing)
        cv2.createTrackbar(HSVRangeMaskStep.sl, self._window_name, self._hsv_range.low.saturation, cv2_helper.SATURATION_MAX, cv2_helper.do_nothing)
        cv2.createTrackbar(HSVRangeMaskStep.sh, self._window_name, self._hsv_range.high.saturation, cv2_helper.SATURATION_MAX, cv2_helper.do_nothing)
        cv2.createTrackbar(HSVRangeMaskStep.vl, self._window_name, self._hsv_range.low.value, cv2_helper.VALUE_MAX, cv2_helper.do_nothing)
        cv2.createTrackbar(HSVRangeMaskStep.vh, self._window_name, self._hsv_range.high.value, cv2_helper.VALUE_MAX, cv2_helper.do_nothing)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)

    def __repr__(self) -> str:
        repr_str = f"HSVRangeMaskStep({repr(self._repr_name)}, {repr(self._hsv_range)}"

        if self._return_mask:
            repr_str += f", return_mask=True"
        if self._copy_start_values:
            repr_str += f", copy_start_values=True"
        if self._invert_hue:
            repr_str += f", invert_hue=True"

        return repr_str + ")"

    