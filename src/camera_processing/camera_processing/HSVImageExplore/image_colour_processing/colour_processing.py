from __future__ import annotations # Import ColourProcessing as a type hint
from typing import Tuple, Callable, List, Union, Any, Set

import cv2
from numpy import ndarray
import time
import numpy as np
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    from process_steps import *
    from simple_mask_steps import *
    from hsv_range_mask_step import HSVRangeMaskStep
    from simple_math_steps import *
    from datatypes import *
    print(f"{__name__} is being used as a script")

except Exception as e:

    # Import for ROS usage
    from . import cv2_helper
    from .process_steps import *
    from .simple_mask_steps import *
    from .hsv_range_mask_step import HSVRangeMaskStep
    from .simple_math_steps import *
    from .datatypes import * 

class ColourProcessing:
    DECIMALS = 5

    def __init__(self, image_scaling: float, pre_scaled: bool, display_scaling: float, process_steps: Tuple[ProcessStep]):
        """
        """
        self._in_tuning_mode: bool
        self._process_steps: Tuple[ProcessStep] = process_steps
        self._pre_scaled: bool = pre_scaled
        self._image_scaling: float = image_scaling
        self._image_reverse_scaling: float = 1 / image_scaling
        self._display_scaling: float = display_scaling
        self.last_mask = None
        self._static_mask = None
        self._static_mask_resized = None
        self._has_static_mask = False
    
        if len(self._process_steps) == 0:
            raise IndexError("Must give at least one MaskStep to ColourProcessing")
        
        self.__print_step_classes()

        found_math_to_mask_step = False
        for i in range(0, len(self._process_steps)):
            if not found_math_to_mask_step:
                if isinstance(self._process_steps[i], MaskToMathStep):
                    found_math_to_mask_step = True
                    self._mask_to_mask_index = i
                elif not isinstance(self._process_steps[i], MaskStep):
                    raise ValueError(f"All ProcessSteps before a MaskToMath step must be a MaskStep. Found {self._process_steps[i].__name__} at index {i}")
            else:
                if not isinstance(self._process_steps[i], MathStep):
                    raise ValueError(f"All ProcessSteps after a MaskToMath step must be a MathStep. Found {self._process_steps[i].__name__} at index {i}")

        for step in self._process_steps:
            step.set_display_scaling(display_scaling)

        ProcessStep.STEP_COUNT = 0

    def __print_step_classes(self):
        steps_class_list = []
        for step in self._process_steps:
            if isinstance(step, MaskStep):
                steps_class_list.append(f"{step.__class__.__name__}(MaskStep)")
            elif isinstance(step, MaskToMathStep):
                steps_class_list.append(f"{step.__class__.__name__}(MaskToMathStep)")
            elif isinstance(step, MathStep):
                steps_class_list.append(f"{step.__class__.__name__}(MathStep)")
            else:
                steps_class_list.append(f"{step.__class__.__name__} ~~ UNKNOWN ~~")
     
    def __repr__(self) -> str:
        repr_msg = '"ColourProcessing(\n'
        repr_msg += f"        image_scaling={self._image_scaling},\n"
        repr_msg += f"        display_scaling={self._display_scaling},\n"
        repr_msg += f"        pre_scaled={self._pre_scaled},\n"
        repr_msg += f"        process_steps=tuple([\n"
        for step in self._process_steps:
            repr_msg += f"            {repr(step)},\n"
        repr_msg += f'        ]))"\n'
        return repr_msg
    
    @classmethod    
    def from_string(colour_processing_class, python_eval: str) -> Union[ColourProcessing, str]:
        """
        Create a ColourProcessing object from a string that can be evaluated to a ColourProcessing object.
        The intention is for HSVImageExplore to print how strings that can be eval'd into ColourProcessing objects.

        Parameters:
            - python_eval (str): The string to eval into a ColourProcessing object
        
        Returns:
            - Union[ColourProcessing, str]: The ColourProcessing object or an error message
        """
        try:
            obj = eval(python_eval)
        except Exception as e:
            return f"Error creating ColourProcessing object. Error: {e}"
        
        if not isinstance(obj, ColourProcessing):
            return f"Error creating ColourProcessing object. Error: {obj} is not a ColourProcessing object."

        return obj
    
    def set_static_mask(self, filename: str, invert_colour: bool):
        """
        """
        try:
            image = cv2.imread(filename)
            self._static_mask = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
            
            if invert_colour:
                self._static_mask = cv2.bitwise_not(self._static_mask)

            self._static_mask_resized = cv2.resize(self._static_mask, None, fx=self._image_scaling, fy=self._image_scaling, interpolation = cv2.INTER_LINEAR)
            self._has_static_mask = True
        except Exception as e:
            self._has_static_mask = False

    def process_image(self, image: ndarray) -> Tuple[List[Tuple[int, int]], List[List[str]], str]:
        """
        Process an image and return a bounding box and tag for targets.

        Parameters:
            - image (ndarray): The image to process

        Returns:
            Intended use:
            bounding_box, tags, timings = colour_procressing.process_image(image)
            - bounding_box(List[Tuple[int, int]]): The bounding boxes for the targets as a List of Points
            - tags(List[List[str]]): A list of tags for each target
            - timings(str): A string of the timings for each step
        """
        start = time.time()

        # Resize the image
        img_resize_start = time.time()
        scale_img = bool(not self._pre_scaled and self._image_scaling != 1.0)
        if scale_img:
            image = cv2.resize(image, None, fx=self._image_scaling, fy=self._image_scaling, interpolation = cv2.INTER_LINEAR)
            processed_image = deepcopy(image)
        else:
            processed_image = deepcopy(image)
        img_resize_end = time.time()

        # Apply the static mask
        print(f"self._has_static_m,ask: {self._has_static_mask}")
        if self._has_static_mask:
            try:
                if scale_img:
                    processed_image = cv2.bitwise_and(processed_image, processed_image, mask=self._static_mask_resized)
                else:
                    processed_image = cv2.bitwise_and(processed_image, processed_image, mask=self._static_mask)
            except Exception as e:
                self._has_static_mask = False

        # Process the image through the mask steps
        mask_timings = []
        for i in range(0, self._mask_to_mask_index):
            mask_step_start = time.time()
            processed_image = self._process_steps[i].process(image, processed_image)
            mask_timings.append(f"{time.time() - mask_step_start:.{ColourProcessing.DECIMALS}f}")

        self.last_mask = processed_image

        # Process the MaskToMathStep
        mask_to_math_start = time.time()
        contours, hierarchy = self._process_steps[self._mask_to_mask_index].process(image, processed_image)
        mask_to_math_end = time.time()

        # Handle no contours producing None in the hierarchy
        if hierarchy is None:
            hierarchy = ()
        else:
            hierarchy = hierarchy[0]

        # Process the contours and hierarchy through the MathSteps
        math_timings = []
        tags: List[Set[str]] = [set() for i in range(0, len(contours))]
        for i in range(self._mask_to_mask_index + 1, len(self._process_steps)):
            math_step_start = time.time()
            contours, hierarchy, tags = self._process_steps[i].process(image, processed_image, contours, hierarchy, tags)
            math_timings.append(f"{time.time() - math_step_start:.{ColourProcessing.DECIMALS}f}")

        # Convert contours to bounding boxes
        bounding_box_start = time.time()
        bounding_boxes = []
        for contour in contours:
            bounding_boxes.append(
                self._scale_bounding_box( # Scale bounding box back into the original images size
                    ColourProcessing._contour_to_bounding_box(contour))) # Convert contour to ZED bounding box
        bounding_box_end = time.time()

        # Create a string for all the timings
        str_timings = f"TIMINGS - Total: {time.time() - start:.{ColourProcessing.DECIMALS}f}, " + \
                        f"Resize: {img_resize_end - img_resize_start:.{ColourProcessing.DECIMALS}f}, " + \
                        f"Mask: {mask_timings}, " + \
                        f"MaskToMath: {mask_to_math_end - mask_to_math_start:.{ColourProcessing.DECIMALS}f}, " + \
                        f"Math: {math_timings}, " + \
                        f"BoundingBoxes: {bounding_box_end - bounding_box_start:.{ColourProcessing.DECIMALS}f}"

        return bounding_boxes, tags, str_timings
    
    def _contour_to_bounding_box(contour) -> list:
        xywh = cv2.boundingRect(contour)
        return ColourProcessing._xywh2abcd(xywh)
 
    def _xywh2abcd(xywh):
        output = np.zeros((4, 2))

        x_min = (xywh[0])
        x_max = (xywh[0] + xywh[2])
        y_min = (xywh[1])
        y_max = (xywh[1] + xywh[3])

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_max
        output[2][1] = y_max

        output[3][0] = x_min
        output[3][1] = y_max
        return output

    def _scale_bounding_box(self, bounding_box):
        for point_index, point in enumerate(bounding_box):
            for xy_index, val in enumerate(point):
                bounding_box[point_index][xy_index] = val * self._image_reverse_scaling

        return bounding_box

    def single_image_process(self, image: ndarray):
        print(f"Image size: {image.shape}")

        for i in range(0, 10):
            bounding_box, tags, timings = self.process_image(image)
            print(timings)

    def process_steps_tuning(self, image: ndarray, starting_step: int = 0) -> Tuple[int, int]:
        process_step_index: int = starting_step
        self._process_steps[process_step_index].start_display()
        
        while True:
            bounding_box, tags, timings = self.process_image(image)            
            print(timings)
            
            key_pressed = cv2.waitKey(5)
            new_index = None

            if key_pressed == cv2_helper.WaitKeyStroke.ESC or key_pressed == ord('s') or key_pressed == ord('d'):
                self._process_steps[process_step_index].stop_display()
                print(f"The Process Steps are below:")
                print(repr(self))
                return key_pressed, process_step_index
            
            elif key_pressed == ord('w'):
                new_index = max(process_step_index - 1, 0)
            elif key_pressed == ord('e'):
                new_index = min(process_step_index + 1, len(self._process_steps) - 1)

            else:
                for i in range(1, len(self._process_steps)+1):
                    if key_pressed == ord(str(i)):
                        new_index = i - 1
                        break
                        
            if new_index is not None and new_index != process_step_index:
                print(f"Stopping {process_step_index} and starting {new_index}")
                self._process_steps[process_step_index].stop_display()
                self._process_steps[new_index].start_display()
                process_step_index = new_index

    def single_image_process_tuning(self, filename: str):
        image: ndarray = cv2.imread(filename)
        print(f"Image size: {image.shape}")
        
        process_step_index: int = 0
        self._process_steps[process_step_index].start_display()
        
        self.process_steps_tuning(image)
        cv2.destroyAllWindows()

    def multi_image_process_tuning(self, image_callback: Callable[..., ndarray]):
        images: List[ndarray] = [image_callback()]
        image_index: int = 0
        process_step: int = 0
        while True:
            key_pressed, process_step = self.process_steps_tuning(images[image_index], process_step)

            if key_pressed == cv2_helper.WaitKeyStroke.ESC:
                break
            elif key_pressed == ord('s'):
                image_index = max(0, image_index - 1)
            elif key_pressed == ord('d'):
                image_index += 1
                if image_index >= len(images):
                    images.append(image_callback())

        cv2.destroyAllWindows()
