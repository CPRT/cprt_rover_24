import cv2
from numpy import ndarray
import numpy as np
from typing import Optional
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    from process_steps import MaskStep
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper
    from .process_steps import MaskStep

class SingleChannelRaiseSaturationStep(MaskStep):
    sat_low_name = "Saturation Lower Bound"
    sat_set_name = "Saturation set to this when above bounds"
    
    def __init__(self, window_name: str, colour_index: int, low_bound: int, set_sat: int = cv2_helper.SATURATION_MAX):
        super().__init__(window_name)
        self._colour_index = colour_index
        self._low_bound = low_bound
        self._set_saturation = set_sat

    def __repr__(self) -> str:
        return f"SingleChannelRaiseSaturationStep({repr(self._repr_name)}, {self._colour_index}, {repr(self._low_bound)}, {repr(self._set_saturation)})"
    
    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """
        
        # All pixels of a specific colour (choosen by self._colour_index) that are greater than self._low_bound are set to self._set_saturation
        # process_img[process_img[:, :, self._colour_index] > self._low_bound, self._colour_index] = self._set_saturation

        # Grab only pixels from that the specific colour
        single_colour_image = process_img[:, :, self._colour_index]

        # Equalize hist to make darker things darker and lighter things ligher
        new_process_image = cv2.equalizeHist(single_colour_image)
        # new_process_image = single_colour_image

        if self._is_display_active:
            self._low_bound = cv2.getTrackbarPos(SingleChannelRaiseSaturationStep.sat_low_name, self._window_name)
            self._set_saturation = cv2.getTrackbarPos(SingleChannelRaiseSaturationStep.sat_set_name, self._window_name)

            # images_stacked_horizontally = np.hstack([original_img, process_img, new_process_image])
            self.imshow_scaled(self._window_name, new_process_image)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(SingleChannelRaiseSaturationStep.sat_low_name, self._window_name, self._low_bound , cv2_helper.SATURATION_MAX, cv2_helper.do_nothing) 
        cv2.createTrackbar(SingleChannelRaiseSaturationStep.sat_set_name, self._window_name, self._set_saturation, cv2_helper.SATURATION_MAX, cv2_helper.do_nothing)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)
    

class ThresholdMaskStep(MaskStep):
    threshold_name = "Threshold"
    greyscale_name = "Greyscale"
    
    def __init__(self, window_name: str, threshold: int, greyscale: int):
        super().__init__(window_name)
        self._threshold = threshold
        self._greyscale = greyscale

    def __repr__(self) -> str:
        return f"ThresholdMaskStep({repr(self._repr_name)}, {repr(self._threshold)}, {repr(self._greyscale)})"
    
    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """

        # Threshold the image
        ret, threshold_image = cv2.threshold(process_img, self._threshold, self._greyscale, cv2.THRESH_BINARY)

        # Mask the original image with the threshold image
        new_process_image = cv2.bitwise_and(original_img, original_img, mask=threshold_image)

        if self._is_display_active:
            self._threshold = cv2.getTrackbarPos(ThresholdMaskStep.threshold_name, self._window_name)
            self._greyscale = cv2.getTrackbarPos(ThresholdMaskStep.greyscale_name, self._window_name)

            images_stacked_horizontally = np.hstack([original_img, new_process_image])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(ThresholdMaskStep.threshold_name, self._window_name, self._threshold, 255, cv2_helper.do_nothing) 
        cv2.createTrackbar(ThresholdMaskStep.greyscale_name, self._window_name, self._greyscale, 255, cv2_helper.do_nothing)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)


class EqualizeHistStep(MaskStep):
   
    def __init__(self, window_name: str):
        super().__init__(window_name)

    def __repr__(self) -> str:
        return f"EqualizeHistStep({repr(self._repr_name)})"
    
    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """

        new_process_image = cv2.equalizeHist(process_img)

        if self._is_display_active:
            images_stacked_horizontally = np.hstack([original_img, new_process_image])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)

class ErodeDilateStep(MaskStep):
    erosion_name = "Erosion"
    dilation_name = "Dilation"
    
    def __init__(self, window_name: str, erosion: int = -1, dilation: int = -1, erosion_before_dilate: bool = False, return_mask: bool = False, circular_kernal: bool = False):
        super().__init__(window_name)
        self._circle_kernal = circular_kernal
        self._set_erosion_dilation_values(erosion, dilation)
        self._erosion_before_dilate = erosion_before_dilate
        self._return_mask = return_mask

    def __repr__(self) -> str:
        repr_str = f"ErodeDilateStep({repr(self._repr_name)}, erosion={self._erosion}, dilation={self._dilation}"
        if self._erosion_before_dilate:
            repr_str += ", erosion_before_dilate=True"
        if self._return_mask:
            repr_str += ", return_mask=True"
        return repr_str + ")"
    
    def _set_erosion_dilation_values(self, erosion: int, dilation: int):
        self._erosion = erosion
        self._dilation = dilation

        # Elliptical kernal
        if erosion > 0:
            if not self._circle_kernal:
                self._erode_kernal = np.ones((erosion, erosion))
            else:
                self._erode_kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion,erosion))
            # self._erode_kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion,erosion))  # cv2.MORPH_CROSS  cv2.MORPH_ELLIPSE

        if dilation > 0:
            if not self._circle_kernal:
                self._dilation_kernal = np.ones((dilation, dilation))
            else:
                self._dilation_kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilation,dilation))
            # self._dilation_kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilation,dilation))

    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """
        if self._erosion <= 0 and self._dilation <= 0:
            new_process_image = process_img

        elif self._erosion <= 0:
            new_process_image = cv2.dilate(process_img, self._dilation_kernal, iterations=1)

        elif self._dilation <= 0:
            new_process_image = cv2.erode(process_img, self._erode_kernal, iterations=1)
        
        elif self._erosion_before_dilate:
            first_image = cv2.erode(process_img, self._erode_kernal, iterations=1)
            new_process_image = cv2.dilate(first_image, self._dilation_kernal, iterations=1)
        
        else:
            first_image = cv2.dilate(process_img, self._dilation_kernal, iterations=1)
            new_process_image = cv2.erode(first_image, self._erode_kernal, iterations=1)

        if self._is_display_active:
            if self._erosion > 0 and self._dilation > 0:
                self._set_erosion_dilation_values(
                    erosion=cv2.getTrackbarPos(ErodeDilateStep.erosion_name, self._window_name)+1,
                    dilation=cv2.getTrackbarPos(ErodeDilateStep.dilation_name, self._window_name)+1)
            elif self._erosion <= 0:
                self._set_erosion_dilation_values(
                    erosion=-1,
                    dilation=cv2.getTrackbarPos(ErodeDilateStep.dilation_name, self._window_name)+1)
            elif self._dilation <= 0:
                self._set_erosion_dilation_values(
                    erosion=cv2.getTrackbarPos(ErodeDilateStep.erosion_name, self._window_name)+1,
                    dilation=-1)

            mask_img = np.zeros(original_img.shape, original_img.dtype)
            mask_img[:, :] = (255, 255, 255)

            if self._erosion > 0 and self._dilation > 0:
                first_image = cv2.bitwise_and(mask_img, mask_img, mask=first_image) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)
                second_image = cv2.bitwise_and(mask_img, mask_img, mask=new_process_image) # Add in colour channel
                images_stacked_horizontally = np.hstack([original_img, first_image, second_image])
            else:
                second_image = cv2.bitwise_and(mask_img, mask_img, mask=new_process_image) # Add in colour channel
                images_stacked_horizontally = np.hstack([original_img, second_image])

            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        if not self._return_mask:
            new_process_image = cv2.bitwise_and(original_img, original_img, mask=new_process_image)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        if self._erosion > 0:
            cv2.createTrackbar(ErodeDilateStep.erosion_name, self._window_name, self._erosion-1, 40, cv2_helper.do_nothing) 

        if self._dilation > 0:
            cv2.createTrackbar(ErodeDilateStep.dilation_name, self._window_name, self._dilation-1, 40, cv2_helper.do_nothing)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)


class InvertMask(MaskStep):
    invert_mask_name = "Invert Mask"
    
    def __init__(self, window_name: str, invert_mask: bool = True, return_mask: bool = True):
        super().__init__(window_name)
        self._invert_mask = invert_mask
        self._return_mask = return_mask

    def __repr__(self) -> str:
        repr_str = f"InvertMask({repr(self._repr_name)}"
        if self._invert_mask:
            repr_str += ", invert_mask=True"
        else:
            repr_str += ", invert_mask=False"

        if self._return_mask:
            repr_str += ", return_mask=True"

        return repr_str + ")"

    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """
        if self._invert_mask:
            new_process_image = cv2.bitwise_not(process_img)
        else:
            new_process_image = process_img

        if self._is_display_active:
            if cv2.getTrackbarPos(InvertMask.invert_mask_name, self._window_name) == 1:
                self._invert_mask = True
            else:
                self._invert_mask = False


            mask_img = new_process_image

            if len(new_process_image.shape) == 2:
                all_white_mask = np.zeros(original_img.shape, original_img.dtype)
                all_white_mask[:, :] = (255, 255, 255)
                mask_img = cv2.bitwise_and(all_white_mask, all_white_mask, mask=new_process_image) # Add in colour channel
                
            images_stacked_horizontally = np.hstack([original_img, mask_img])

            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        if not self._return_mask:
            new_process_image = cv2.bitwise_and(original_img, original_img, mask=new_process_image)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(InvertMask.invert_mask_name, self._window_name, 1 if self._invert_mask else 0, 1, cv2_helper.do_nothing) 
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)


class BlurStep(MaskStep):
    gaussian_blur_name = "Use Gaussian Blur"
    blur_strangth_name = "Blur Strength"
    
    def __init__(self, window_name: str, blur_strength: int, gaussian_blur: bool = True):
        super().__init__(window_name)
        self._blur_strength = blur_strength
        self._gaussian_blur = gaussian_blur

    def __repr__(self) -> str:
        repr_str = f"BlurStep({repr(self._repr_name)}, blur_strength={int(self._blur_strength)}"
        if self._gaussian_blur:
            repr_str += ", gaussian_blur=True"
        else:
            repr_str += ", gaussian_blur=False"

        return repr_str + ")"

    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image
        """
        if len(process_img.shape) == 2:
            all_white_mask = np.zeros(original_img.shape, original_img.dtype)
            all_white_mask[:, :] = (255, 255, 255)
            process_img = cv2.bitwise_and(all_white_mask, all_white_mask, mask=process_img) # Add in colour channel

        if self._gaussian_blur:
            new_process_image = cv2.GaussianBlur(process_img, (self._blur_strength, self._blur_strength), 0)
        else:
            new_process_image = cv2.blur(process_img, (self._blur_strength, self._blur_strength))

        if self._is_display_active:
            self._blur_strength = cv2.getTrackbarPos(BlurStep.blur_strangth_name, self._window_name)
            if cv2.getTrackbarPos(BlurStep.gaussian_blur_name, self._window_name) == 1:
                self._gaussian_blur = True
            else:
                self._gaussian_blur = False

                
            images_stacked_horizontally = np.hstack([original_img, new_process_image])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return new_process_image
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(BlurStep.gaussian_blur_name, self._window_name, 1 if self._gaussian_blur else 0, 1, cv2_helper.do_nothing) 
        cv2.createTrackbar(BlurStep.blur_strangth_name, self._window_name, self._blur_strength, 400, cv2_helper.do_nothing)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)