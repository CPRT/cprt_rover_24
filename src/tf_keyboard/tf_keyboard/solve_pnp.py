import cv2
import numpy as np
from typing import Tuple, Dict, Any
import os

class SolvePnPURCKeyboard:
    def __init__(self, score_threshold: float = 0.8):
        """Initializes the SolvePnPURCKeyboard."""
        self._score_threshold = score_threshold

        # Erik calibration with 600 images and using mrcal
        self.camera_matrix = np.array([[1362.595875, 0.0, 972.1443409], [0.0, 1357.467093, 637.3062039], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([-0.39826831,  0.22052135, -0.0097452 , 0.00097967, -0.02851982])

        # Load the keyboard model. Dictionay mapping keys to 3D coordinates relative to the Z key.
        self._keyboard_model = self._create_keyboard_model()

    def _create_keyboard_model(self):
        reg_key_height = 14.06
        reg_key_width = 13.05
        reg_key_horizontal_spacing = 19.06507

        enter_lock_width = 36.78
        caps_lock_width = 26.10
        spacerbar_width = 113.42
        bot_row_to_middle_row = 18.7
        middle_row_to_top_row = 19.32
        left_keyboard_edge_to_left_of_Z_key = 48.67
        right_of_caps_lock_to_left_of_A_key = 7.30

        z_to_capslock_x_offset = 35
        j_to_enter_x_offset = 107
        z_to_spacebar_x_offset = 77.87
        z_to_spacebar_y_offset = 19.64

        q_to_a_x_offset = 4.5
        z_to_a_x_offset = 9.0

        z_axis_height = 0.0 # used for all keys

        keys = {}

        # Origin of keyboard is Z key. Axes: X: right. Y: up. Z: out of keyboard
        # Z is up. Z will be set to 0 for all keys after creating the dict.
        keys["z"] = (0, 0)

        # middle row starting key
        keys["a"] = (keys["z"][0] - z_to_a_x_offset,
                 keys["z"][1] + bot_row_to_middle_row)
        keys["q"] = (keys["a"][0] - q_to_a_x_offset,
                 keys["a"][1] + middle_row_to_top_row)

        # Bottom row keys
        for i, key in enumerate(["z", "x", "c", "v", "b", "n", "m"]):
            if i == 0:
                continue
            keys[key] = (keys["z"][0] + i * reg_key_horizontal_spacing, keys["z"][1])

        # Middle row keys
        for i, key in enumerate(["a", "s", "d", "f", "g", "h", "j", "k", "l"]):
            if i == 0:
                continue
            keys[key] = (keys["a"][0] + i * reg_key_horizontal_spacing, keys["a"][1])

        # Top row keys
        for i, key in enumerate(["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"]):
            if i == 0:
                continue
            keys[key] = (keys["q"][0] + i * reg_key_horizontal_spacing, keys["q"][1])

        # Caps lock key
        keys["caps_lock"] = (keys["z"][0] - z_to_capslock_x_offset, keys["z"][1] + bot_row_to_middle_row)

        # Enter key
        keys["enter"] = (keys["j"][0] + j_to_enter_x_offset, keys["j"][1])

        # Space bar
        keys["space_bar"] = (keys["z"][0] - z_to_spacebar_x_offset, keys["z"][1] - z_to_spacebar_y_offset)

        # keyboard, origin is the same as the z key
        keys["keyboard"] = (keys["z"][0], keys["z"][1])

        # Loop through keys and add a 3rd element for z coordinate
        for key in keys:
            x, y = keys[key]
            keys[key] = (x, y, z_axis_height)

        return keys
    
    def _rvec_to_quaternion(rvec):
        """Converts a rotation vector (rvec) to a quaternion.
        Args:
            rvec: A NumPy array representing the rotation vector.
        Returns:
            A NumPy array representing the quaternion.
        """
        # 1. Extract rotation axis and angle
        theta = np.linalg.norm(rvec)
        axis = rvec / theta if theta != 0 else np.array([1, 0, 0]) # Handle case where theta is zero
        # 2. Normalize the axis (already done in the previous step)
        # 3. Calculate the quaternion
        q = (np.cos(theta / 2),
                                    np.sin(theta / 2) * axis[0],
                                    np.sin(theta / 2) * axis[1],
                                    np.sin(theta / 2) * axis[2])
        return q
    
    def solvepnp(self, detections, image_np=None):
        """
        Estimates the pose of the keyboard using the detected keypoints.
        Args:
            detections (dict): The object detection results. Bounding boxes must be
                               in pixel coordinates of the original image.
            image_np (np.ndarray): The pixels used for solvepnp will be drawn onto this image if given.
        Returns:
            tuple: A tuple containing the estimated xyz, pitch, roll, and yaw.
                   Returns None if the pose cannot be estimated.
        """
        if 'detection_boxes' not in detections:
            print("detection_boxes not found in detections")
            return None
        if 'detection_scores' not in detections:
            print("detection_scores not found in detections")
            return None
        if 'detection_names' not in detections:
            print("detection_names not found in detections")
            return None
        
        pixel_boxes = detections['detection_boxes']

        object_points = []
        image_points = []
        key_chars_points = []

        if len(pixel_boxes) != len(detections['detection_names']):
            print(f"Length of pixel_boxes ({len(pixel_boxes)}) does not match length of detection_names ({len(detections['detection_names'])})")
            return None
        if len(pixel_boxes) != len(detections['detection_scores']):
            print(f"Length of pixel_boxes ({len(pixel_boxes)}) does not match length of detection_scores ({len(detections['detection_scores'])})")
            return None

        # Iterate through the bounding boxes
        for i, box in enumerate(pixel_boxes):
            # Extract the coordinates of the bounding box
            y1, x1, y2, x2 = box

            class_name = detections['detection_names'][i]
            if class_name == "keyboard":
                continue

            score = detections['detection_scores'][i]
            if score < self._score_threshold:
                continue

            if class_name is None:
                print(f"Class name is None for index {i}")
                continue

            object_point = self._keyboard_model.get(class_name)
            if object_point is None:
                print(f"Object point not found for key character {class_name}")
                continue

            key_chars_points.append(class_name)
            object_points.append(object_point)

            # Calculate the center of the bounding box
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            image_points.append([center_x, center_y])

        # Reshape the object_points and image_points to the correct dimensions for solvePnP
        object_points = np.array(object_points, dtype=np.float32).reshape(-1, 3)
        image_points = np.array(image_points, dtype=np.float32).reshape(-1, 2)

        if len(object_points) < 4 or len(image_points) < 4:
            print(f"Not enough points to solve PnP. object_points: {len(object_points)}, image_points: {len(image_points)}")
            return None
        
        if image_np is not None:
            # Draw points on the image
            for i, point in enumerate(image_points):
                cv2.circle(image_np, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1) # Green circles
                cv2.putText(image_np, key_chars_points[i], (int(point[0]) + 10, int(point[1]) + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA) # White text

            # Create a new plot for object points
            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # for i, point in enumerate(object_points):
            #     ax.scatter(point[0], point[1], point[2], c='r', marker='o') # Red spheres
            #     ax.text(point[0], point[1], point[2], f" {key_chars_points[i]}", color='black') # Black text beside point
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            # ax.set_title('3D Object Points with Keychars')
            # plt.show()