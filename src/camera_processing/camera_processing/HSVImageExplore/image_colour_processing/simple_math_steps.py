import cv2
from numpy import ndarray
import numpy as np
from typing import Optional, Tuple, List, Set
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    from process_steps import MaskToMathStep, MathStep
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper
    from .process_steps import MaskToMathStep, MathStep

# 
class FindContours(MaskToMathStep):
    
    def __init__(self, window_name: str, mode: int = cv2.RETR_TREE, method: int = cv2.CHAIN_APPROX_SIMPLE):
        """
        
        Example: FindContours("Find Contours", cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        """
        super().__init__(window_name)

        self._mode = mode
        self._method = method

    def __repr__(self) -> str:
        return f"FindContours({repr(self._repr_name)})"
    
    def process(self, original_image: ndarray, mask: ndarray) -> Tuple[list, list]:
        """
        Process contours to remove 

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps

        Returns:
            Tuple[list, list]: The contours and hierarchy of the mask from cv2.findContours
        """
        contours, hierarchy = cv2.findContours(mask, self._mode, self._method)

        if self._is_display_active:

            mask_img = np.zeros(original_image.shape, original_image.dtype)
            mask_img[:, :] = (255, 255, 255)
            mask_with_contours = cv2.bitwise_and(mask_img, mask_img, mask=mask) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)

            cv2.drawContours(mask_with_contours, contours, -1, (0, 255, 0), 2)

            for i, contour in enumerate(contours):
                x, y, w, h = cv2.boundingRect(contour)
                cv2.putText(mask_with_contours, str(i), (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, MathStep.Colour.GREEN, 2)
                

            images_stacked_horizontally = np.hstack([original_image, mask_with_contours])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy
  
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

class FilterInnerContours(MathStep):
    REJECT_INNER_CONTOURS = "Reject Inner Contours"

    def __init__(self, window_name: str, reject_inner_contours: bool = True):
        """
        Remove contours that are within other contours, or remove contours that are not within other contours.
        """
        super().__init__(window_name)

        self._reject_inner_contours = reject_inner_contours

    def __repr__(self) -> str:
        return f"FilterInnerContours('{self._repr_name}', reject_inner_contours={bool(self._reject_inner_contours)})"

    def process(self, original_image: ndarray, mask: ndarray, contours: list, hierarchy: list, tags: List[Set[str]]) -> Tuple[list, list, List[Set[str]]]:
        """
        Process contours to remove bad contours or tag various contours

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps
            contours (list): The original image with no processing
            hierarchy (list): The processed image returned from the previous step
            tags (List[Set[str]]): The tags for each contour

        Returns:
            ndarray: The newly processed image
        """
        removed_contours = []

        for i in range(0, len(contours)):
            if MathStep.CoreTag.REJECT in tags[i]:
                continue

            # If it has a parent contour, its an inner contour
            is_inner_contour: bool = (hierarchy[i][3] != -1)

            if (self._reject_inner_contours and is_inner_contour) or (not self._reject_inner_contours and not is_inner_contour):
                tags[i].add(MathStep.CoreTag.REJECT)
                removed_contours.append(contours[i])

        if self._is_display_active:
            self._reject_inner_contours = bool(cv2.getTrackbarPos(FilterInnerContours.REJECT_INNER_CONTOURS, self._window_name))
            mask_with_contours = self.draw_contours(original_image, mask, contours, tags, removed_contours)

            images_stacked_horizontally = np.hstack([original_image, mask_with_contours])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy, tags


    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(FilterInnerContours.REJECT_INNER_CONTOURS, self._window_name, self._reject_inner_contours, 1, cv2_helper.do_nothing)

    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)
    
class FilterByContourArea(MathStep):
    REJECT_BELOW_NAME = "Reject Below"
    FAR_CLOSE_DIVIDER_NAME = "Far Close Divider"
    REJECT_ABOVE_NAME = "Reject Above"

    def __init__(self, window_name: str, reject_below: int, far_close_divider: int, reject_above: int, area_scaling: float = 1.0):
        """
        Remove contours below an area threshold from reject_below, tag contours below far_close_divider as far, 
        tags contours above far_close_divider as close, and remove contours above reject_above.
        """
        super().__init__(window_name)

        self._reject_below = int(reject_below * area_scaling)
        self._far_close_divider = int(far_close_divider * area_scaling)
        self._reject_above = int(reject_above * area_scaling)

        self._area_scaling = area_scaling
        

    def __repr__(self) -> str:
        return f"FilterByContourArea({repr(self._repr_name)}, reject_below={self._reject_below}, " + \
                    f"far_close_divider={self._far_close_divider}, reject_above={self._reject_above})"
    
    def process(self, original_image: ndarray, mask: ndarray, contours: list, hierarchy: list, tags: List[Set[str]]) -> Tuple[list, list, List[Set[str]]]:
        """
        Process contours to remove bad contours or tag various contours

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps
            contours (list): The original image with no processing
            hierarchy (list): The processed image returned from the previous step
            tags (List[Set[str]]): The tags for each contour

        Returns:
            ndarray: The newly processed image
        """
        removed_contours = []

        for i, contour in enumerate(contours):
            if MathStep.CoreTag.REJECT in tags[i]:
                continue

            area = cv2.contourArea(contour)

            if area < self._reject_below:
                tags[i].add(MathStep.CoreTag.REJECT)
                removed_contours.append(contour)
                
            elif area < self._far_close_divider:
                tags[i].add(MathStep.CoreTag.FAR)

            elif area < self._reject_above:
                tags[i].add(MathStep.CoreTag.CLOSE)

            else:
                tags[i].add(MathStep.CoreTag.REJECT)
                removed_contours.append(contours[i])

        if self._is_display_active:
            self._reject_below = cv2.getTrackbarPos(FilterByContourArea.REJECT_BELOW_NAME, self._window_name)
            self._far_close_divider = cv2.getTrackbarPos(FilterByContourArea.FAR_CLOSE_DIVIDER_NAME, self._window_name)
            self._reject_above = cv2.getTrackbarPos(FilterByContourArea.REJECT_ABOVE_NAME, self._window_name)

            mask_img = np.zeros(original_image.shape, original_image.dtype)
            mask_img[:, :] = (255, 255, 255)
            mask_with_contours = cv2.bitwise_and(mask_img, mask_img, mask=mask) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)

            cv2.drawContours(mask_with_contours, removed_contours, -1, MathStep.Colour.RED, 2)

            for i, contour in enumerate(contours):

                if MathStep.CoreTag.REJECT in tags[i]:
                    continue

                elif MathStep.CoreTag.CLOSE in tags[i]:
                    cv2.drawContours(mask_with_contours, contours, i, MathStep.Colour.GREEN, 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.putText(mask_with_contours, f"close-{int(cv2.contourArea(contour))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, MathStep.Colour.GREEN, 2)
                
                elif MathStep.CoreTag.FAR in tags[i]:
                    cv2.drawContours(mask_with_contours, contours, i, MathStep.Colour.LIGHT_BLUE, 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.putText(mask_with_contours, f"far-{int(cv2.contourArea(contour))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, MathStep.Colour.LIGHT_BLUE, 2)
                
                else:
                    print(f"EDGE CASE. i: {i}, tags[i]: {tags[i]}")

            for contour in removed_contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.putText(mask_with_contours, f"reject-{int(cv2.contourArea(contour))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, MathStep.Colour.RED, 2)

            images_stacked_horizontally = np.hstack([original_image, mask_with_contours])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy, tags
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(FilterByContourArea.REJECT_BELOW_NAME, self._window_name, self._reject_below, 1000, cv2_helper.do_nothing)
        cv2.createTrackbar(FilterByContourArea.FAR_CLOSE_DIVIDER_NAME, self._window_name, self._far_close_divider, 6000, cv2_helper.do_nothing)
        cv2.createTrackbar(FilterByContourArea.REJECT_ABOVE_NAME, self._window_name, self._reject_above, 20000, cv2_helper.do_nothing)

    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)



class SaveMask(MathStep):
    CONTOUR_INDEX = "Contour Index"

    def __init__(self, window_name: str, save_contour_index: int = 0, save_filename: str = "mask.png"):
        """
        Remove contours below an area threshold from reject_below, tag contours below far_close_divider as far, 
        tags contours above far_close_divider as close, and remove contours above reject_above.
        """
        super().__init__(window_name)
        self._save_filename = save_filename
        self._save_contour_index = save_contour_index

        print(f"Saving image to {self._save_filename}")
        

    def __repr__(self) -> str:
        return f'SaveMask({repr(self._repr_name)}, save_filename="{self._save_filename}", ' + \
                    f"save_contour_index={self._save_contour_index})"
    
    def process(self, original_image: ndarray, mask: ndarray, contours: list, hierarchy: list, tags: List[Set[str]]) -> Tuple[list, list, List[Set[str]]]:
        """
        Process contours to remove bad contours or tag various contours

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps
            contours (list): The original image with no processing
            hierarchy (list): The processed image returned from the previous step
            tags (List[Set[str]]): The tags for each contour

        Returns:
            ndarray: The newly processed image
        """
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if len(sorted_contours) > self._save_contour_index:
            saved_contour = sorted_contours[self._save_contour_index]

            self.save_mask = np.zeros_like(mask)
            cv2.fillPoly(self.save_mask, pts=[saved_contour], color=(255))

            cv2.imwrite(self._save_filename, self.save_mask)

            filename_split = self._save_filename.split(".")
            if len(filename_split) == 2:
                original_image_name = f"{filename_split[0]}-OriginalImage.{filename_split[1]}"
                masked_original_image_name = f"{filename_split[0]}-MaskedOriginalImage.{filename_split[1]}"

                masked_original_image = cv2.bitwise_and(original_image, original_image, mask=self.save_mask)

                cv2.imwrite(original_image_name, original_image)
                cv2.imwrite(masked_original_image_name, masked_original_image)


        if self._is_display_active:
            self._save_contour_index = cv2.getTrackbarPos(SaveMask.CONTOUR_INDEX, self._window_name)

            mask_img = np.zeros(original_image.shape, original_image.dtype)
            mask_img[:, :] = (255, 255, 255)
            mask_with_contours = cv2.bitwise_and(mask_img, mask_img, mask=self.save_mask) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)

            cv2.drawContours(mask_with_contours, saved_contour, -1, MathStep.Colour.GREEN, 2)

            new_process_image = cv2.bitwise_and(original_image, original_image, mask=self.save_mask)

            images_stacked_horizontally = np.hstack([original_image, mask_with_contours, new_process_image])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy, tags
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(SaveMask.CONTOUR_INDEX, self._window_name, self._save_contour_index, 50, cv2_helper.do_nothing)

    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)
        cv2.imwrite(self._save_filename, self.save_mask)