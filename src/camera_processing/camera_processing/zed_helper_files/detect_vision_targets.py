from __future__ import annotations
import cv2
import pyzed.sl as sl
import numpy as np
from enum import Enum
from typing import List, Set, Tuple
from math import sqrt, isnan

# from ZedNode import STRING_TO_RESOLUTION
from ..HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing
from ..HSVImageExplore.image_colour_processing.process_steps import MathStep

from .map_targets_between_cams import PitchYaw, Point, CameraUtil, LinearUndistortion



# ELP 640x480
# elp_width = 640
# elp_height = 480
# elp_hfov = 49.29*2
# elp_vfov = 37.91*2

# ELP 1280x720
elp_width = 1280
elp_height = 720
elp_hfov = 62.91*2
elp_vfov = 37.91*2

# ELP 1920x1080
# elp_width = 1920
# elp_height = 1080
# elp_hfov = 62.91*2
# elp_vfov = 37.91*2

ZED_CAM_NAME = "ZED"
IR_CAM_NAME = "IRCam"

class CameraType(Enum):
    ZED = CameraUtil(ZED_CAM_NAME, 1920.0, 1080.0, 110.0, 70.0)
    ERIK_ELP = CameraUtil(IR_CAM_NAME, elp_width, elp_height, elp_hfov, elp_vfov)
    IRCAM_ELP = CameraUtil(IR_CAM_NAME, 1280, 720, 62.91*2, 37.91*2, "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0")
    IRCAM_PS3EYE = CameraUtil(IR_CAM_NAME, 640.0, 480.0, 58.0, 45.0, "usb-OmniVision_Technologies__Inc._USB_Camera-B4.09.24.1-video-index0")
    IRCAM_IRCUT = CameraUtil(IR_CAM_NAME, 640.0, 480.0, 50*4/3, 50, "usb-USB_Camera_Manufacture_H264_USB_Camera_2020052801-video-index0")

    # def update_scaling(self, image_scaling: float):
    #     """
    #     Resize the camera resolution based on a image_scaling factor
    #     """
    #     self.value.xRes = float(self.value.xRes * image_scaling)
    #     self.value.yRes = float(self.value.yRes * image_scaling)

class DetectVisionTargets:
    def __init__(self, ros_logger, blue_led: ColourProcessing, red_led: ColourProcessing, ir_led: ColourProcessing, ir_undistort: LinearUndistortion):
        self._ros_logger = ros_logger

        self.aruco_dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params_4x4 =  cv2.aruco.DetectorParameters()
        self.aruco_detector_4x4 = cv2.aruco.ArucoDetector(self.aruco_dict_4x4, self.aruco_params_4x4)

        self.aruco_dict_6x6 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params_6x6 = cv2.aruco.DetectorParameters()
        self.aruco_detector_6x6 = cv2.aruco.ArucoDetector(self.aruco_dict_6x6, self.aruco_params_6x6)

        self.blue_led_processing = blue_led
        self.red_led_processing = red_led
        self.ir_led_processing = ir_led

        self.ir_linear_undistort = ir_undistort

    def bounding_box(points: np.array):
        """
        Find min/max from an N-collection of coordinate pairs, shape = (N, 2), using 
        numpy's min/max along the collection-axis 
        """
        xMin = min(point[0] for point in points)
        yMin = min(point[1] for point in points)
        xMax = max(point[0] for point in points)
        yMax = max(point[1] for point in points)

        box = np.zeros((4, 2))

        box[0][0] = xMin
        box[0][1] = yMin

        box[1][0] = xMax
        box[1][1] = yMin

        box[2][0] = xMax
        box[2][1] = yMax

        box[3][0] = xMin
        box[3][1] = yMax

        return box

    def create_aruco_unique_label(camera_mapping: CameraType, marker_id: int) -> str:
        return f"{camera_mapping.value.name}-ArucoID{marker_id}"

    def is_zed_marker(unique_label: str) -> bool:
        return f"{ZED_CAM_NAME}-ArucoID" in unique_label
    
    def is_ir_cam_marker(unique_label: str) -> bool:
        return f"{IR_CAM_NAME}-ArucoID" in unique_label
    
    def is_blue_led(unique_label: str) -> bool:
        return "BlueLED" in unique_label
    
    def is_red_led(unique_label: str) -> bool:
        return "RedLED" in unique_label
    
    def is_ir_led(unique_label: str) -> bool:
        return "IRLED" in unique_label

    def get_marker_id_from_label(unique_label: str) -> int:
        try:
            return int(unique_label[unique_label.index("-ArucoID")+len("-ArucoID"):])
        except Exception as e:
            return -1

    def unique_object_id_from_tags(base: str, tags: Set[str]) -> str:
        if MathStep.CoreTag.REJECT in tags:
            return ""
        
        tag_str = ""
        
        if MathStep.CoreTag.FAR in tags:
            tag_str += f"-{MathStep.CoreTag.FAR.value}"
        elif MathStep.CoreTag.CLOSE in tags:
            tag_str += f"-{MathStep.CoreTag.CLOSE.value}"
        else:
            return "" # Error
        
        return base + tag_str

    def detectArucoMarkers(self, img, cameraMapping: CameraType) -> List[sl.CustomBoxObjectData]:
        try:
            corners, ids, rejected = self.aruco_detector_4x4.detectMarkers(img)
            detections = []

            if len(corners) == 0:
                # self._ros_logger.info("No aruco markers found")
                return detections

            elif len(corners) != len(ids):
                self._ros_logger.error("Mismatched id and corners arrays from the aruco detector")
                return detections

            else:
                for i in range(0, len(corners)):
                    markerCorners = corners[i]
                    markerID: int = ids[i][0]

                    reshapedCorners = DetectVisionTargets.bounding_box(markerCorners.reshape((4, 2)))

                    # if (cameraMapping != CameraType.ZED):
                    #     for points in reshapedCorners:
                    #         # print(f"    REMAPPING: {points}")
                    #         pitchYaw = cameraMapping.value.pitchYawFromXY(Point(points[0], points[1]))
                    #         xy = CameraType.ZED.xyFromPitchYaw(pitchYaw)
                    #         points[0] = max(0, xy.x) # Ensure values are not negative
                    #         points[1] = max(0, xy.y)
                    #         # print(f"           TO: {points}")
                    #         # print(f"     PitchYaw: {pitchYaw}")

                    # Creating ingestable objects for the ZED SDK
                    obj = sl.CustomBoxObjectData()
                    obj.unique_object_id = DetectVisionTargets.create_aruco_unique_label(cameraMapping, markerID)
                    obj.bounding_box_2d = reshapedCorners # Converts to unsigned int so values MUST be positive or it fails
                    obj.label = 1 if cameraMapping == CameraType.ZED else 2
                    obj.probability = 0.99
                    obj.is_grounded = False
                    detections.append(obj)

            return detections

        except Exception as e:
            self._ros_logger.error(f"Encounted exception trying to detect aruco markers: {e}")
            return detections


    def detectZEDBlueLEDs(self, zed_img) -> List[sl.CustomBoxObjectData]:
        detections = []

        # Blue LEDS
        bounding_boxes, tags, timings = self.blue_led_processing.process_image(zed_img)
        self._ros_logger.info("Blue LED " + timings)

        for i in range(0, len(bounding_boxes)):
            unique_object_id: str = DetectVisionTargets.unique_object_id_from_tags(f"BlueLED-{i}", tags[i])

            if unique_object_id != "":
                obj = sl.CustomBoxObjectData()
                obj.unique_object_id = unique_object_id
                obj.bounding_box_2d = bounding_boxes[i]
                obj.label = 3
                obj.probability = 0.99
                obj.is_grounded = False
                detections.append(obj)

        return detections
    
    def detectZEDRedLEDs(self, zed_img) -> List[sl.CustomBoxObjectData]:
        detections = []

        # Red LEDS
        bounding_boxes, tags, timings = self.red_led_processing.process_image(zed_img)
        self._ros_logger.info("Red LED " + timings)

        for i in range(0, len(bounding_boxes)):
            unique_object_id: str = DetectVisionTargets.unique_object_id_from_tags(f"RedLED-{i}", tags[i])

            if unique_object_id != "":
                obj = sl.CustomBoxObjectData()
                obj.unique_object_id = unique_object_id
                obj.bounding_box_2d = bounding_boxes[i]
                obj.label = 4
                obj.probability = 0.99
                obj.is_grounded = False
                detections.append(obj)

        return detections

    def detectIRLEDS(self, ir_img, cameraMapping: CameraType) -> List[sl.CustomBoxObjectData]:
        detections = []

        # try:
        #     with open("/home/jetson/LinearUnDistortionTestParams.txt") as file:
        #         new_distortion = LinearUndistortion.from_string(file.readline())
        #     if isinstance(new_distortion, LinearUndistortion):
        #         self.ir_linear_undistort = new_distortion
        #         self._ros_logger.info(f"New Undistortion: {self.ir_linear_undistort}")
        # except:
        #     pass

        # IR LEDS
        bounding_boxes, tags, timings = self.ir_led_processing.process_image(ir_img)
        self._ros_logger.info("IR LED " + timings)

        for box_index, bounding_box in enumerate(bounding_boxes):
            unique_object_id: str = DetectVisionTargets.unique_object_id_from_tags(f"IRLED-{box_index}", tags[box_index])

            if unique_object_id == "":
                self._ros_logger.info("Skipped ir target because the unique_object_id is empty")
                continue

            for point_index, point in enumerate(bounding_boxes[box_index]):
                pitchYaw = cameraMapping.value.pitchYawFromXY(Point(point[0], point[1]))
                undistorted_pitchYaw = self.ir_linear_undistort.undistort(pitchYaw)
                xy = CameraType.ZED.value.xyFromPitchYaw(undistorted_pitchYaw)

                # self._ros_logger.info(f"Original {pitchYaw} --- Undistorted {undistorted_pitchYaw}")
                # self._ros_logger.info(f"Original X: {point[0]:.2f} Y: {point[1]:.2f} --- Post X: {xy.x:.2f} Y: {xy.y:.2f}")
                
                try:
                    bounding_boxes[box_index][point_index][0] = max(0, int(xy.x)) # Ensure values are not negative
                    bounding_boxes[box_index][point_index][1] = max(0, int(xy.y))
                except Exception as e:
                    self._ros_logger.info(f"Failed to scale bounding boxes with exception: {e}")


            obj = sl.CustomBoxObjectData()
            obj.unique_object_id = unique_object_id
            obj.bounding_box_2d = bounding_boxes[box_index]
            obj.label = 4
            obj.probability = 0.99
            obj.is_grounded = False
            detections.append(obj)

        return detections

    def detect_6x6_arucos(self, zed_img) -> List[sl.CustomBoxObjectData]:
        corners, ids, rejected = self.aruco_detector_6x6.detectMarkers(zed_img)
        # cv2.aruco.drawDetectedMarkers(zed_img, corners, ids)
        detections = []

        if len(corners) == 0:
            # self._ros_logger.info("No aruco markers of the 6x6_100 family found")
            return detections

        elif len(corners) != len(ids):
            self._ros_logger.error("Mismatched id and corners arrays from the aruco detector")
            return detections
        
        else:
            for i in range(0, len(corners)):
                markerCorners = corners[i]
                markerID: int = int(ids[i][0])

                reshapedCorners = DetectVisionTargets.bounding_box(markerCorners.reshape((4, 2)))

                if markerID >= 0 and markerID <= 20:
                    unique_object_id = f"BlueLED-Aruco6x6-{i}"

                elif markerID <= 40:
                    unique_object_id = f"RedLED-Aruco6x6-{i}"

                elif markerID <= 60:
                    unique_object_id = f"IRLED-Aruco6x6-{i}"

                else:
                    continue

                # Creating ingestable objects for the ZED SDK
                obj = sl.CustomBoxObjectData()
                obj.unique_object_id = unique_object_id
                obj.bounding_box_2d = reshapedCorners # Converts to unsigned int so values MUST be positive or it fails
                obj.label = 6
                obj.probability = 0.99
                obj.is_grounded = False
                detections.append(obj)

        return detections

    def calculate_distance(position: np.array) -> str:
        if len(position) < 3:
            return ""
        
        distance = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2])
        if isnan(distance):
            return f"nan-[{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}]"
        
        return f"{distance:.2f}m"

    def is_good_object(object_data: sl.ObjectData, logger):
        if (object_data.tracking_state == sl.OBJECT_TRACKING_STATE.SEARCHING or 
            object_data.tracking_state == sl.OBJECT_TRACKING_STATE.TERMINATE):
            logger.info("Removed OBJECT due to state: " + str(object_data.tracking_state))
            return False

        return True
    
    def seperate_objects_by_zed_state(objects: List[sl.ObjectData], logger) -> Tuple[List[sl.ObjectData], List[sl.ObjectData], List[sl.ObjectData]]:
        searching: List[sl.ObjectData] = []
        moving: List[sl.ObjectData] = []
        idle: List[sl.ObjectData] = []

        for obj in objects:
            if (obj.tracking_state == sl.OBJECT_TRACKING_STATE.SEARCHING or 
                    obj.tracking_state == sl.OBJECT_TRACKING_STATE.TERMINATE):
                searching.append(obj)
            elif obj.action_state == sl.OBJECT_ACTION_STATE.MOVING:
                moving.append(obj)
            elif obj.action_state == sl.OBJECT_ACTION_STATE.IDLE:
                idle.append(obj)
            else:
                logger.info(f"Removed OBJECT due to unknown tracking/action state. Tracking: {obj.tracking_state}" +
                            f"Action: {obj.action_state}")
                
        return searching, moving, idle



    def draw_object_detection(img, object_data: sl.ObjectData):
        if not isinstance(object_data, sl.ObjectData):
            return
        
        (topLeft, topRight, bottomRight, bottomLeft) = object_data.bounding_box_2d

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # Draw the bounding box
        cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

        # Compute and draw the center (x, y)-coordinates
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)

        # Draw unique_object_id beside the contour
        text = f"{str(object_data.id)}-{str(object_data.unique_object_id)}-{DetectVisionTargets.calculate_distance(object_data.position)}"
        if object_data.action_state == sl.OBJECT_ACTION_STATE.MOVING: text += "-moving"
        if object_data.action_state == sl.OBJECT_ACTION_STATE.IDLE: text += "-idle"
        cv2.putText(img, text, (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)