import numpy as np
import tensorflow as tf
from typing import Tuple, Dict, Any
import os
import cv2
from ament_index_python.packages import get_package_share_directory


class KeyboardDetector:
    __PACKAGE_DIR_TESTING = None  # Class variable

    @classmethod
    def set_package_dir_testing(cls, path: str):
        """Sets the __PACKAGE_DIR_TESTING class variable."""
        cls.__PACKAGE_DIR_TESTING = path

    def __init__(self):
        """Initializes the KeyboardDetector with the path to the model and label map."""
        if KeyboardDetector.__PACKAGE_DIR_TESTING is not None:
            self._package_dir = KeyboardDetector.__PACKAGE_DIR_TESTING
        else:
            self._package_dir = get_package_share_directory("tf_keyboard")

            # Download the keyboard model from google drive.
            from tf_keyboard.download_model import download_will_urc_keyboard_model

            download_will_urc_keyboard_model(run_by_ros2=True)

        # Load the model Will trained with his custom labeled dataset
        path_to_saved_model = os.path.join(
            self._package_dir, "model/exported-models/saved_model"
        )
        path_to_labels = os.path.join(
            self._package_dir, "model/annotations/label_map.pbtxt"
        )

        self._class_name_to_int: Dict[str, int] = {}
        self._class_int_to_name: Dict[int, str] = {}
        self._class_int_to_name, self._class_name_to_int = self._load_label_map(
            path_to_labels
        )

        self._model, self._detect_fn = self._load_model(path_to_saved_model)

    def _load_label_map(self, label_path: str) -> Dict[int, str]:
        """Loads the label map from the .pbtxt file.
        Args:
            label_path (str): Path to the label map file.
        Returns:
            dict: A dictionary mapping class IDs to category names.
        """
        class_name_to_int: Dict[str, int] = {}
        class_int_to_name: Dict[int, str] = {}

        try:
            with open(label_path, "r") as f:
                item_block = {}
                for line in f:
                    line = line.strip()
                    if line == "item {:":
                        item_block = {}
                    elif line == "}":
                        if "id" in item_block and "name" in item_block:
                            class_id = int(item_block["id"])
                            class_name = item_block["name"].strip("'")
                            class_name_to_int[class_name] = class_id
                            class_int_to_name[class_id] = class_name
                    elif ":" in line:
                        key, value = line.split(":", 1)
                        item_block[key.strip()] = value.strip().strip("'")
        except FileNotFoundError:
            print(f"Label map file not found: {label_path}")
            return {}, {}
        except Exception as e:
            print(f"Error loading label map: {e}")
            return {}, {}

        return class_int_to_name, class_name_to_int

    def _load_model(self, path_to_model: str) -> Tuple[Any, tf.function]:
        """Loads the object detection model from the SavedModel."""
        detection_model = tf.saved_model.load(path_to_model)
        detect_fn = detection_model.signatures[
            "serving_default"
        ]  # Get the detection function
        return detection_model, detect_fn

    def get_labels_str_to_int(self) -> Dict[str, int]:
        """Returns the mapping of class names to IDs."""
        return self._class_name_to_int

    def get_labels_int_to_str(self) -> Dict[int, str]:
        """Returns the mapping of class IDs to names."""
        return self._class_int_to_name

    def _convert_normalized_to_pixel_coordinates(
        self, normalized_boxes, image_width, image_height
    ):
        """
        Converts normalized bounding box coordinates to pixel coordinates.
        Args:
            normalized_boxes (numpy.ndarray): Array of normalized bounding box coordinates.
            image_width (int): Width of the image in pixels.
            image_height (int): Height of the image in pixels.
        Returns:
            numpy.ndarray: Array of bounding box coordinates in pixel coordinates.
        """
        pixel_boxes = normalized_boxes.copy()
        pixel_boxes[:, 0] = normalized_boxes[:, 0] * image_height
        pixel_boxes[:, 1] = normalized_boxes[:, 1] * image_width
        pixel_boxes[:, 2] = normalized_boxes[:, 2] * image_height
        pixel_boxes[:, 3] = normalized_boxes[:, 3] * image_width
        return pixel_boxes

    def detect(self, image: np.ndarray) -> Dict[str, Any]:
        """Detects objects in the image and returns the results.
        Args:
            image (np.ndarray): The input image.
        Returns:
            dict: A dictionary containing detection results:
                - 'detection_boxes': Bounding boxes of detected objects.
                - 'detection_classes': Class IDs of detected objects.
                - 'detection_names': Class names of detected objects.
                - 'detection_scores': Confidence scores of detected objects.
                - 'num_detections': Number of detected objects.
        """
        # The input needs to be a tensor
        input_tensor = tf.convert_to_tensor(image, dtype=tf.uint8)  # Changed dtype
        input_tensor = input_tensor[tf.newaxis, ...]  # add batch dimension
        detections = self._detect_fn(input_tensor)
        # All outputs are batches tensors.
        # We want to keep numpy array, so take index 0.
        # And then use tf.make_ndarray to get the numpy array.
        num_detections = int(detections.pop("num_detections"))
        detections = {
            key: value[0, :num_detections].numpy() for key, value in detections.items()
        }
        detections["num_detections"] = num_detections
        # Conversion to int64.
        detections["detection_classes"] = detections["detection_classes"].astype(
            np.int64
        )

        # Check if detection_boxes exists
        if not "detection_boxes" in detections:
            detections["detection_boxes"] = np.array([])

        detections["detection_boxes"] = self._convert_normalized_to_pixel_coordinates(
            detections["detection_boxes"], image.shape[1], image.shape[0]
        )

        # Add detection_names
        detections["detection_names"] = []
        for i in range(num_detections):
            class_id = int(detections["detection_classes"][i])
            if class_id in self._class_int_to_name:
                detections["detection_names"].append(self._class_int_to_name[class_id])
            else:
                detections["detection_names"].append(None)

        return detections

    def draw_detections_onto_image(
        self,
        image: np.ndarray,
        detections: Dict[str, Any],
        min_score_thresh: float = 0.3,
    ) -> None:
        """Draws the detection results onto the image.
        Args:
            image (np.ndarray): The input image to draw on.
            detections (dict): The detection results.
        """
        # Visualization of the results of a detection.
        for i in range(detections["detection_boxes"].shape[0]):
            box = detections["detection_boxes"][i]
            class_id = int(detections["detection_classes"][i])
            score = detections["detection_scores"][i]
            if score > min_score_thresh:
                ymin, xmin, ymax, xmax = box
                im_height, im_width = image.shape[:2]
                (left, right, top, bottom) = (
                    xmin * im_width,
                    xmax * im_width,
                    ymin * im_height,
                    ymax * im_height,
                )
                cv2.rectangle(
                    image,
                    (int(left), int(top)),
                    (int(right), int(bottom)),
                    (0, 255, 0),
                    1,
                )  # green
                score_percent = int(100 * score)
                class_name = self.get_labels_int_to_str()[class_id]
                start_y = int(top) + int((bottom - top) / 2) + 15
                start_x = int(left) + 2
                if "caps" in class_name:
                    start_x = int(left) - 80
                elif len(class_name) == 1:
                    class_name = class_name.upper()
                elif "keyboard" in class_name:
                    start_y = int(top) + 20
                label = f"{class_name}{score_percent}"
                cv2.putText(
                    image,
                    label,
                    (start_x, start_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )  # red


# Main function to test the KeyboardDetector
# This is a simple test to visualize the detection results on an image.
if __name__ == "__main__":
    KeyboardDetector.set_package_dir_testing("../")
    detector = KeyboardDetector()

    image_path = os.path.join(detector._package_dir, "model/test_images/image_0004.png")
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Image not found at {image_path}")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    dim = (1280, 720)  # Desired dimensions
    image = image[
        int((image.shape[0] - dim[1]) / 2) : int((image.shape[0] + dim[1]) / 2),
        int((image.shape[1] - dim[0]) / 2) : int((image.shape[1] + dim[0]) / 2),
    ]

    detections = detector.detect(image)

    print(f"Number of detections: {detections['num_detections']}")
    for i in range(detections["num_detections"]):
        print(f"Detection {i}:")
        print(f"  Class ID: {repr(detections['detection_classes'][i])}")
        print(f"  Class Name: {repr(detections['detection_names'][i])}")
        print(f"  Score: {repr(detections['detection_scores'][i])}")
        print(f"  Box: {repr(detections['detection_boxes'][i])}")

    detector.draw_detections_onto_image(image, detections)

    cv2.imshow("Detections", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
