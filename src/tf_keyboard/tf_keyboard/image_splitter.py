import numpy as np
from typing import Tuple


class ImageSplitter:
    def __init__(self):
        """Initializes the ImageSplitter."""
        self.width = 0
        self.height = 0
        self.base_image = None

        self.last_split_start_x = 0
        self.last_split_start_y = 0
        self.last_split_end_x = 0
        self.last_split_end_y = 0

    def reset_base_image(self, base_image: np.ndarray) -> None:
        """Resets the base image and its dimensions.

        Args:
            base_image (np.ndarray): The base image to reset.
        """
        self.base_image = base_image
        self.height, self.width = base_image.shape[:2]

    def get_image_subsection_centered_on_point(self, x: int, y: int, width: int, height: int) -> np.ndarray:
        """Returns a subsection of the base image centered on a given point.

        Args:
            x (int): The x-coordinate of the center point.
            y (int): The y-coordinate of the center point.
            width (int): The width of the subsection.
            height (int): The height of the subsection.

        Returns:
            np.ndarray: The subsection of the base image. It's a copy so changing it won't affect the base image.
        """
        if self.base_image is None:
            raise ValueError("Base image is not set.")

        # Check that the width and height fit in the base image
        if width > self.width or height > self.height:
            raise ValueError("Width and height must fit within the base image dimensions.")
        
        # Calculate the coordinates for the subsection
        self.last_split_start_x = x - width // 2
        self.last_split_start_y = y - height // 2
        self.last_split_end_x = x + width // 2
        self.last_split_end_y = y + height // 2

        # Adjust the coordinates to ensure they are within the base image bounds
        if self.last_split_start_x < 0:
            self.last_split_start_x = 0
            self.last_split_end_x = width
        if self.last_split_start_y < 0:
            self.last_split_start_y = 0
            self.last_split_end_y = height
        if self.last_split_end_x > self.width:
            self.last_split_end_x = self.width
            self.last_split_start_x = self.width - width
        if self.last_split_end_y > self.height:
            self.last_split_end_y = self.height
            self.last_split_start_y = self.height - height

        # Return the subsection of the base image
        return self.base_image[
            self.last_split_start_y:self.last_split_end_y,
            self.last_split_start_x:self.last_split_end_x
        ]
    
    def convert_split_image_pixels_to_base_image(self, x: int, y: int) -> Tuple[int, int]:
        """Converts the coordinates of a split image back to the base image.

        Args:
            x (int): The x-coordinate in the split image.
            y (int): The y-coordinate in the split image.

        Returns:
            Tuple[int, int]: The coordinates in the base image.
        """
        if self.base_image is None:
            raise ValueError("Base image is not set.")

        # Convert the coordinates
        base_x = self.last_split_start_x + x
        base_y = self.last_split_start_y + y

        return base_x, base_y
