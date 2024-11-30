#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy import Parameter
from ament_index_python.packages import get_package_share_directory

import cv2
import os
import numpy as np
import pyzed.sl as sl
from math import isnan
from threading import Lock, Thread
from copy import deepcopy

# import rclpy.time as Time
from typing import List
from threading import Lock, Thread
from time import sleep

from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from interfaces.msg import PointArray, ArucoMarkers
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField, Imu, MagneticField
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32, Float32

from .zed_helper import imageToROSMsg, slTime2Ros, imuDataToROSMsg, magneticDataToROSMsg

from .zed_viewers.cv_viewer import tracking_viewer as cv_viewer
from .zed_viewers.ogl_viewer import viewer as gl

from .zed_helper_files.detect_vision_targets import DetectVisionTargets, CameraType
from.zed_helper_files.map_targets_between_cams import LinearUndistortion
from .zed_helper_files.video_capture import VideoCapture
from .HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing
from .HSVImageExplore.image_colour_processing.hsv_range_mask_step import HSVRangeMaskStep
from .HSVImageExplore.image_colour_processing.simple_mask_steps import ErodeDilateStep
from .HSVImageExplore.image_colour_processing.datatypes import HSVRange, HSV

class ZedNode(Node):
    STRING_TO_RESOLUTION = {"HD720": sl.RESOLUTION.HD720, "HD1080": sl.RESOLUTION.HD1080, "HD1200": sl.RESOLUTION.HD1200, "HD2K": sl.RESOLUTION.HD2K}

    def __init__(self):
        super().__init__('zed')
        self.setup_params()
        self.setup_detect_vision_targets()

        self.cv_bridge = CvBridge()

        if self.should_detect_ir_led:
            try:
                self.ir_cam: VideoCapture = VideoCapture(CameraType.IRCAM_IRCUT)
            except Exception as e:
                self.get_logger().error("Failed to create VideoCapture for IR cam. Disabling IR camera. Error: " + str(e))
                self.should_detect_ir_led = False

        self.publish_current_svo_index_topic = self.create_publisher(Int32, "/zed/current_svo_index", 10)
        self.publish_max_svo_index = self.create_publisher(Int32, "/zed/max_svo_index", 10)

        zed_initialized = False
        while not zed_initialized:
            if self.init_zed():
                zed_initialized = True
            else:
                sl.Camera.reboot(sn=0, full_reboot=True) # Reboot fixes USB problems on Jetson Nano
    
        self.init_visualizers()

        self.image_left_tmp: sl.Mat = sl.Mat()
        self.objects: sl.Object = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.zed_pose = sl.Pose()
        self.depth_mat = sl.Mat()
        self.point_cloud = sl.Mat()
        self.sensors_data = sl.SensorsData()
        self.imu_data = sl.IMUData()
        self.magnetometer_data = sl.MagnetometerData()

        self.depth_mat_lock = Lock()
        self.has_new_depth_image = False

        self.publisher_depth_image = self.create_publisher(Image, '/zed/depth_image', 10)
        # self.publisher_point_cloud = self.create_publisher(PointCloud2, '/zed/point_cloud', 10) # CONSUMES TOO MANY RESOURCES/TOO SLOW

        self.publisher_imu_data = self.create_publisher(Imu, '/zed/imu_data', 10)
        self.publisher_magnetometer_data = self.create_publisher(MagneticField, '/zed/magnetic_field', 10)
        self.publisher_magnetic_north_heading = self.create_publisher(Float32, '/zed/magnetic_north_pole_heading', 10)

        self.publish_zed_aruco_points = self.create_publisher(ArucoMarkers, '/zed/zed_aruco_points', 10)
        self.publish_blue_led_points = self.create_publisher(PointArray, '/zed/blue_led_points', 10)
        self.publish_red_led_points = self.create_publisher(PointArray, '/zed/red_led_points', 10)
        self.publish_ir_led_points = self.create_publisher(PointArray, '/zed/ir_led_points', 10)

        self.publish_raw_image = self.create_publisher(CompressedImage, '/zed/zed_raw_image', 10)
        self.publish_raw_ir_image = self.create_publisher(CompressedImage, '/zed/ir_raw_image', 10)
        self.publish_cv_image = self.create_publisher(CompressedImage, '/zed/cv_zed_image', 10)

        self.publish_rviz_markers = self.create_publisher(MarkerArray, '/zed/zed_rviz_detections', 10)

        self.control_svo_index = 0
        self.subscribe_control_svo_index_topic = self.create_subscription(Int32, "/zed/control_svo_index", self.set_svo_index, 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.timer_period = 0.03  # 0.066 for 15 FPS
        self.timer = self.create_timer(self.timer_period, self.run_detections, callback_group=None)

        self.depth_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_depth = self.create_timer(0.1, self.publish_depth_image, callback_group=self.depth_callback_group)

        self.sensors_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_sensors = self.create_timer(0.01, self.publish_sensors_data, callback_group=self.sensors_callback_group) # IMU can run at 400 HZ or 0.0025 seconds

        self.timestamp = self.get_clock().now()
        self.header_timestamp = self.get_clock().now().to_msg()

    def setup_params(self) -> bool:
        self.frame_id = "zed_left_frame_link"
        self.imu_frame_id = "zed_imu_link"

        self.declare_parameters(
            namespace="",
            parameters=[ 
                # From zed_params.yaml. Below values are defaults, see zed_params.yaml for actual values
                ('openni_depth_mode', False),
                ('depth_image_scaling', 0.25),
                ('mask_filename', ''),
                ('profile_name', 'night'),

                ('zed_arucos_detections', True),
                ('blue_led_detections', True),
                ('red_led_detections', True),
                ('ir_led_detections', True),
                ('detect_6x6_aruco_as_leds', False),

                ('ir_undistort_pitch_offset', 0.0),
                ('ir_undistort_pitch_slope', 1.0),
                ('ir_undistort_yaw_offset', 0.0),
                ('ir_undistort_yaw_slope', 1.0),

                ('publish_raw_image', False),
                ('publish_raw_ir_image', False),
                ('publish_cv_processed_image', False),
                ('enable_gl_viewer', False),
                ('svo_realtime_mode', False), # Doesn't work on Jetson Nano
                ('always_record', False),
                ('default_record_filename', ''),
                ('resolution', 'HD1080'), # Options: HD720, HD1080, HD1200, HD2K
                ('fps', 30),
                ('depth_maximum_distance', 15), # meters
                ('exposure', 50),
                ('gain', 95),
                ('gamma', 7),
                ('white_balance', 4600),
                ('enable_object_tracking', False),

                # From colour_processing_params.yaml. This yaml file is required
                ('resize_for_processing', 0.4),
                ('resize_for_displaying', 0.6),
                ('blue_led', ""),
                ('red_led', ""),
                ('ir_led', ""),

                # From launch file arguments
                ('playback_filename', ''),
                ('record_filename', ''),
                ('publish_gl_viewer_data', False),
                ('publish_6x6_aruco_as_leds', False),
                ('playback_start_index', 0)
            ]
        )

        if self.get_parameter('resolution').value not in ZedNode.STRING_TO_RESOLUTION:
            raise ValueError(f"ROS2 parameter resolution in ZED Node is not one of {ZedNode.STRING_TO_RESOLUTION.keys()}")

        self.get_logger().info("Loaded parameters from profile named " + str(self.get_parameter('profile_name').value))


        self.openni_depth_mode = bool(self.get_parameter('openni_depth_mode').value)
        self.depth_image_scaling = float(self.get_parameter('depth_image_scaling').value)
        self.mask_filename = str(self.get_parameter('mask_filename').value)
        self.mask_full_filepath = os.path.join(get_package_share_directory('camera_processing'), 'zed_mask', self.mask_filename)
        if len(self.mask_filename) == 0:
            self.mask_full_filepath = ""

        self.should_detect_arucos = bool(self.get_parameter('zed_arucos_detections').value)
        self.should_detect_blue_led = bool(self.get_parameter('blue_led_detections').value)
        self.should_detect_red_led = bool(self.get_parameter('red_led_detections').value)
        self.should_detect_ir_led = bool(self.get_parameter('ir_led_detections').value)
        self.should_detect_6x6_aruco = bool(self.get_parameter('detect_6x6_aruco_as_leds').value)

        self.ir_undistort_pitch_offset = float(self.get_parameter('ir_undistort_pitch_offset').value)
        self.ir_undistort_pitch_slope = float(self.get_parameter('ir_undistort_pitch_slope').value)
        self.ir_undistort_yaw_offset = float(self.get_parameter('ir_undistort_yaw_offset').value)
        self.ir_undistort_yaw_slope = float(self.get_parameter('ir_undistort_yaw_slope').value)

        self.should_publish_raw_image = bool(self.get_parameter('publish_raw_image').value)
        self.should_publish_raw_ir_image = bool(self.get_parameter('publish_raw_ir_image').value)
        self.should_publish_cv_processed_image = bool(self.get_parameter('publish_cv_processed_image').value)
        self.enable_gl_viewer = bool(self.get_parameter('enable_gl_viewer').value)
        self.should_publish_gl_viewer_data = bool(self.get_parameter('publish_gl_viewer_data').value)
        self.should_publish_6x6_aruco_as_leds = bool(self.get_parameter('publish_6x6_aruco_as_leds').value)
        self.playback_start_index = int(self.get_parameter('playback_start_index').value)

        self.record_svo: bool = False
        self.playback_svo: bool = False
        self.record_filename: str = ""
        self.playback_filename: str = ""

        if str(self.get_parameter('playback_filename').value) != '':
            self.playback_svo = True
            self.playback_filename = str(self.get_parameter('playback_filename').value)

        elif str(self.get_parameter('record_filename').value) != '':
            self.record_svo = True
            self.record_filename = str(self.get_parameter('record_filename').value)

        elif self.get_parameter('always_record') and str(self.get_parameter('default_record_filename')) != '':
            self.record_svo = True
            self.record_filename = str(self.get_parameter('default_record_filename').value)

        self.resize_for_processing = float(self.get_parameter('resize_for_processing').value)
        self.resize_for_displaying = float(self.get_parameter('resize_for_displaying').value)
        self.blue_led_str = str(self.get_parameter('blue_led').value)
        self.red_led_str = str(self.get_parameter('red_led').value)
        self.ir_led_str = str(self.get_parameter('ir_led').value)
    
    def setup_detect_vision_targets(self):
        """
        Setup the DetectVisionTargets object with the ColourProcessing objects for the blue, red and ir LEDs.
        """
        blue_led_colour_processing = ColourProcessing.from_string(self.blue_led_str)
        red_led_colour_processing = ColourProcessing.from_string(self.red_led_str)
        ir_led_colour_processing = ColourProcessing.from_string(self.ir_led_str)
        
        if len(self.mask_full_filepath) != 0:
            blue_led_colour_processing.set_static_mask(self.mask_full_filepath, True)
            red_led_colour_processing.set_static_mask(self.mask_full_filepath, True)
            # ir_led_colour_processing.set_static_mask(self.mask_full_filepath, True)

        if not blue_led_colour_processing or isinstance(blue_led_colour_processing, str):
            self.get_logger().error(f"Failed to create blue_led_colour_processing: {blue_led_colour_processing}") 
            self.should_detect_blue_led = False

        if not red_led_colour_processing or isinstance(red_led_colour_processing, str):
            self.get_logger().error(f"Failed to create red_led_colour_processing: {red_led_colour_processing}")
            self.should_detect_red_led = False

        if not ir_led_colour_processing or isinstance(ir_led_colour_processing, str):
            self.get_logger().error(f"Failed to create ir_led_colour_processing: {ir_led_colour_processing}")
            self.should_detect_ir_led = False

        undistort = LinearUndistortion(
            pitch_offset=self.ir_undistort_pitch_offset, 
            pitch_slope=self.ir_undistort_pitch_slope,
            yaw_offset=self.ir_undistort_yaw_offset,
            yaw_slope=self.ir_undistort_yaw_slope,
            static_pitch_offset=4,
            static_yaw_offset=4
        )

        self.detectVisionTargets = DetectVisionTargets(
            ros_logger=self.get_logger(),
            blue_led=blue_led_colour_processing,
            red_led=red_led_colour_processing,
            ir_led=ir_led_colour_processing,
            ir_undistort=undistort
        )

    def init_zed(self) -> bool:
        """
        Initialize the ZED Camera. 
        Returns False when the camera should be rebooted and initialized again.

        Raises:
            Exception: Various exceptions from the ZED SDK about the camera failing to initialize.
        Returns:
            bool: True means the camera was correctly initialized. False means the camera should be rebooted and re-initialized.
        """
        self.record_timestamp()
        self.get_logger().info("Initializing ZED Camera")

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(svo_real_time_mode=False)
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Can use PERFORMANCE MODE but it misses some details (sl.DEPTH_MODE.ULTRA) (sl.DEPTH_MODE.PERFORMANCE)
        init_params.depth_maximum_distance = int(self.get_parameter('depth_maximum_distance').value)
        init_params.camera_resolution = ZedNode.STRING_TO_RESOLUTION[str(self.get_parameter('resolution').value)]   # HD720   HD1080   HD1200    HD2K
        init_params.camera_fps = int(self.get_parameter('fps').value)

        # Setup runtime parameters
        self.runtime_params = sl.RuntimeParameters()
        # self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD
        self.runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

        if self.playback_svo and self.playback_filename != "":
            init_params.set_from_svo_file(self.playback_filename)
            self.get_logger().info(f"Playing back SVO. Data from the ZED is a recording and NOT LIVE. Filename: {self.playback_filename}")
        
        self.zed = sl.Camera()
        status = self.zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            if status == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
                self.get_logger().error("Failed to zed.open(init_params) Got ZED error code: " + repr(status))
                return False
        
        exposure = int(self.get_parameter('exposure').value)
        white_balance = int(self.get_parameter('white_balance').value)
        white_balence_auto = 0 if int(self.get_parameter('white_balance').value) == -1 else 1
        gain = int(self.get_parameter('gain').value)
        gamma = int(self.get_parameter('gamma').value)

        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, exposure)
        self.get_logger().info(f"Set exposure to {exposure}")

        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, white_balance)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, white_balence_auto)
        self.get_logger().info(f"Set white balance to {white_balance} and auto white balance to {white_balence_auto}")
        
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, gain)
        self.get_logger().info(f"Set gain to {gain}")

        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, gamma)
        self.get_logger().info(f"Set gamma to {gamma}")

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS  # CUSTOM_BOX_OBJECTS  MULTI_CLASS_BOX_MEDIUM
        obj_param.enable_tracking = bool(self.get_parameter('enable_object_tracking').value)
        self.zed.enable_object_detection(obj_param)

        if (not (self.playback_svo and self.playback_filename != "")) and self.record_svo and self.record_filename != "":
            recordingParameters = sl.RecordingParameters()
            recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
            recordingParameters.video_filename = self.record_filename
            self.get_logger().info(f"Starting to record the ZED data to an SVO named {recordingParameters.video_filename} with compression format {recordingParameters.compression_mode.name}")
            err = self.zed.enable_recording(recordingParameters)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to enable SVO recording on ZED. Error: {err}")

        if self.playback_svo and self.playback_filename != "":
            self.zed.set_svo_position(self.playback_start_index)
            self.publish_max_svo_index.publish(Int32(data=self.zed.get_svo_number_of_frames()))
            self.publish_current_svo_index_topic.publish(Int32(data=self.playback_start_index))
            self.get_logger().info(f"Set SVO position to {self.playback_start_index}")

        camera_info = self.zed.get_camera_information()
        camera_res = camera_info.camera_configuration.resolution
        self.depth_mat_res = sl.Resolution(int(camera_res.width * self.depth_image_scaling), int(camera_res.height * self.depth_image_scaling))
        self.get_logger().info(f"ZED Resolution is {camera_res.width}x{camera_res.height}")
        self._roi_mask = sl.Mat()
        if len(self.mask_full_filepath) != 0:
            self._roi_mask.read(self.mask_full_filepath)

        # TODO: Fix error Invalid mask datatype. Expected one of (<MAT_TYPE.U8_C1: 4>, <MAT_TYPE.U8_C3: 6>, <MAT_TYPE.U8_C4: 7>). Got MAT_TYPE.F32_C1

        self.get_logger().info(f"ZED imu to left camera transformation - " + 
                               f"xyz (m): {camera_info.sensors_configuration.camera_imu_transform.get_translation().get()}" + 
                               f"rpy (rad): {camera_info.sensors_configuration.camera_imu_transform.get_euler_angles(radian=True)}")
        
        self.get_logger().info(f"ZED magnetometer to imu transformation - " + 
                               f"xyz (m): {camera_info.sensors_configuration.imu_magnetometer_transform.get_translation().get()}" + 
                               f"rpy (rad): {camera_info.sensors_configuration.imu_magnetometer_transform.get_euler_angles(radian=True)}")

        allowed_mask_datatypes = (sl.MAT_TYPE.U8_C1, sl.MAT_TYPE.U8_C3, sl.MAT_TYPE.U8_C4)
        if self._roi_mask.get_data_type() not in allowed_mask_datatypes:
            self.get_logger().error(f"Invalid mask datatype. Expected one of {allowed_mask_datatypes}. Got {self._roi_mask.get_data_type()}")
        else:
            self.zed.set_region_of_interest(self._roi_mask)
            self.get_logger().info(f"Set region of interest mask from {self.mask_full_filepath}")
            

        self.get_logger().info(f"Finished initializing ZED Camera in {self.delta_time()} seconds")
        return True

    def init_visualizers(self):
        if self.enable_gl_viewer:
            # Get Camera resolution
            camera_info = self.zed.get_camera_information()
            camera_res = camera_info.camera_configuration.resolution

            self.viewer = gl.GLViewer()
            self.viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, False)

    def cleanup(self):
        cv2.destroyAllWindows()
        if self.enable_gl_viewer:
            self.viewer.exit()

        # Disable modules and close camera
        self.zed.disable_recording()
        # self.zed.disable_object_detection()
        # self.zed.disable_positional_tracking()
        self.image_left_tmp.free(memory_type=sl.MEM.CPU)
        
        self.zed.close()
        

    def run_detections(self):
        self.record_timestamp()
        self.get_logger().info(f"Timer callback - Running zed.grab blocking call")

        # grab is a blocking call
        error_code = self.zed.grab(self.runtime_params)
        if error_code != sl.ERROR_CODE.SUCCESS:
            if error_code == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().info(f"SVO end has been reached ({self.zed.get_svo_position()}). Looping back to first frame")
                self.zed.set_svo_position(0)
            else:
                self.get_logger().error(f"ERROR_CODE reported from ZED from grab function. ERROR_CODE: {repr(error_code)}")
            return
        
        if self.playback_svo:
            self.publish_current_svo_index_topic.publish(Int32(data=self.zed.get_svo_position()))
            self.publish_max_svo_index.publish(Int32(data=self.zed.get_svo_number_of_frames()))

        self.header_timestamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"zed.grab finished after {self.delta_time()} seconds")
        self.record_timestamp()

        detections: List[sl.CustomBoxObjectData] = []

        # ZED Image
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        zed_left_image_net: np.array = self.image_left_tmp.get_data()
        zed_img = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2BGR)
        resized_zed_img = cv2.resize(zed_img, None, fx=self.resize_for_processing, fy=self.resize_for_processing, interpolation=cv2.INTER_LINEAR)

        # Store Depth Image
        if not self.depth_mat_lock.locked():
            self.depth_mat_lock.acquire(blocking=True)
            try:
                if not self.openni_depth_mode:
                    self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU, self.depth_mat_res)
                else:
                    self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH_U16_MM, sl.MEM.CPU, self.depth_mat_res)
                self.has_new_depth_image = True
            finally:
                self.depth_mat_lock.release()
        else:
            self.get_logger().info("Depth image is still being processed")

        # Publish Point Cloud
        # self.publish_point_cloud()

        # Publish Raw Image
        if self.should_publish_raw_image:
            self.publish_raw_image.publish(self.cv_bridge.cv2_to_compressed_imgmsg(resized_zed_img))

        # ZED Aruco Markers
        if self.should_detect_arucos:
            detections += self.detectVisionTargets.detectArucoMarkers(zed_img, CameraType.ZED)

        # ZED Blue LEDS
        if self.should_detect_blue_led:
            detections += self.detectVisionTargets.detectZEDBlueLEDs(resized_zed_img)

        # ZED Red LEDS
        if self.should_detect_red_led:
            detections += self.detectVisionTargets.detectZEDRedLEDs(resized_zed_img)

        # IR Cam Image 
        if self.should_detect_ir_led:
            try:
                ir_image, ir_timing = self.ir_cam.read()
                self.get_logger().info(f"IR Retrieve timing: {ir_timing}")
                
            except Exception as e:
                self.get_logger().error("Got error reading IR Cam: " + str(e))

            if self.should_publish_raw_ir_image:
                self.publish_raw_ir_image.publish(self.cv_bridge.cv2_to_compressed_imgmsg(ir_image))

            # IR Cam Aruco Markers
            # detections += self.detectVisionTargets.detectArucoMarkers(ir_image, self.ir_cam.cam_type)

            # IR Cam LEDs
            detections += self.detectVisionTargets.detectIRLEDS(ir_image, self.ir_cam.cam_type)

        # ZED detect 6x6 Arucos as fake LEDs
        if self.should_detect_6x6_aruco:
            detections += self.detectVisionTargets.detect_6x6_arucos(zed_img)

        # ZED get position in it's world frame to publish
        tracking_state = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)

        # self.get_logger().info(f"Detections: {detections}")

        # Ingest detections and get objects (TODO: Verify all objects in list are correct type to prevent crashing)
        self.zed.ingest_custom_box_objects(detections)
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        # self.get_logger().info(f"Objects: {self.objects.object_list}")

        if self.enable_gl_viewer and self.viewer.is_available():
            self.viewer.update_view(self.image_left_tmp, self.objects)

        # Iterate object detections
        zed_aruco_markers_msg = self.create_aruco_markers_msg()
        blue_led_point_arr = self.create_point_array()
        red_led_point_arr = self.create_point_array()
        ir_led_point_arr = self.create_point_array()

        for obj in self.objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.TERMINATE or obj.tracking_state == sl.OBJECT_TRACKING_STATE.SEARCHING:
                continue

            point: Point = self.zed_object_to_point(obj)

            if isnan(point.x) or isnan(point.y) or isnan(point.z):
                continue

            if DetectVisionTargets.is_zed_marker(obj.unique_object_id):
                zed_aruco_markers_msg.points.append(point)
                zed_aruco_markers_msg.marker_ids.append(DetectVisionTargets.get_marker_id_from_label(obj.unique_object_id))
                zed_aruco_markers_msg.is_moving.append(obj.action_state == sl.OBJECT_ACTION_STATE.MOVING)

                # self.get_logger().info(f"Found Aruco: {zed_aruco_markers_msg}")

            elif DetectVisionTargets.is_blue_led(obj.unique_object_id):
                blue_led_point_arr.points.append(point)
                blue_led_point_arr.is_moving.append(bool(obj.action_state == sl.OBJECT_ACTION_STATE.MOVING))

            elif DetectVisionTargets.is_red_led(obj.unique_object_id):
                red_led_point_arr.points.append(point)
                red_led_point_arr.is_moving.append(bool(obj.action_state == sl.OBJECT_ACTION_STATE.MOVING))

            elif DetectVisionTargets.is_ir_led(obj.unique_object_id):
                ir_led_point_arr.points.append(point)
                ir_led_point_arr.is_moving.append(bool(obj.action_state == sl.OBJECT_ACTION_STATE.MOVING))

            else:
                self.get_logger().warn("Lost object after zed detections called " + obj.unique_object_id)
                obj.unique_object_id += "-LOST"
                continue

            if self.should_publish_cv_processed_image:
                DetectVisionTargets.draw_object_detection(zed_img, obj)

        self.publish_zed_aruco_points.publish(zed_aruco_markers_msg)
        self.publish_blue_led_points.publish(blue_led_point_arr)
        self.publish_red_led_points.publish(red_led_point_arr)
        self.publish_ir_led_points.publish(ir_led_point_arr)

        self.publish_rviz_markers.publish(ZedNode.get_rviz_markers(self.objects.object_list, self.header_timestamp))

        self.get_logger().info(f"Zed computations finished in {self.delta_time()} seconds")

        if self.should_publish_cv_processed_image:
            # display_image = cv2.resize(zed_img, None, fx=self.resize_for_displaying, fy=self.resize_for_displaying, interpolation=cv2.INTER_LINEAR)
            self.publish_cv_image.publish(self.cv_bridge.cv2_to_compressed_imgmsg(zed_img)) 

    def publish_depth_image(self):
        if self.has_new_depth_image:
            try:
                self.depth_mat_lock.acquire(blocking=True)
                self.has_new_depth_image = False
                start_time = self.get_clock().now()
                depth_image_msg: Image = imageToROSMsg(self.depth_mat, self.frame_id, self.header_timestamp)
                self.get_logger().info(f"\nDepth image converted in {self.delta_time(start_time)} seconds\n")
                self.publisher_depth_image.publish(depth_image_msg)
                sleep(0.9)
            finally:
                self.depth_mat_lock.release()
        
    def publish_sensors_data(self):
        res = self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT)

        if res != sl.ERROR_CODE.SUCCESS:
            return
        
        self.imu_data = self.sensors_data.get_imu_data()
        self.magnetometer_data = self.sensors_data.get_magnetometer_data()

        imu_msg: Imu = imuDataToROSMsg(self.imu_data, self.imu_frame_id, self.get_clock().now().to_msg())
        self.publisher_imu_data.publish(imu_msg)

        magnet_msg: MagneticField = magneticDataToROSMsg(self.magnetometer_data, self.imu_frame_id, self.get_clock().now().to_msg())
        self.publisher_magnetometer_data.publish(magnet_msg)

        is_heading_accurate = self.magnetometer_data.magnetic_heading_accuracy > 0.2
        is_heading_good = self.magnetometer_data.magnetic_heading_state == sl.HEADING_STATE.GOOD
        is_heading_ok = self.magnetometer_data.magnetic_heading_state == sl.HEADING_STATE.OK
        if is_heading_accurate and (is_heading_good or is_heading_ok):
            magnetic_north_heading = Float32()
            magnetic_north_heading.data = self.magnetometer_data.magnetic_heading
            self.publisher_magnetic_north_heading.publish(magnetic_north_heading)

    def publish_point_cloud(self):
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZBGRA, sl.MEM.CPU)
        point_cloud_msg: PointCloud2 = PointCloud2()
        
        point_cloud_msg.header.frame_id = self.frame_id
        point_cloud_msg.header.stamp = self.header_timestamp
        point_cloud_msg.height = self.point_cloud.get_height()
        point_cloud_msg.width = self.point_cloud.get_width()

        point_cloud_msg.is_bigendian = False
        point_cloud_msg.is_dense = False

        point_cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=7, count=1),
            PointField(name="y", offset=4, datatype=7, count=1),
            PointField(name="z", offset=8, datatype=7, count=1),
            PointField(name="rgb", offset=12, datatype=7, count=1)
        ]

        point_cloud_msg.point_step = 16
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width

        point_cloud_msg.data = self.point_cloud.get_data(memory_type=sl.MEM.CPU, deep_copy=True).tobytes()

        self.publisher_point_cloud.publish(point_cloud_msg)

    def create_aruco_markers_msg(self) -> ArucoMarkers:
        markers = ArucoMarkers()
        markers.header.frame_id = self.frame_id
        markers.header.stamp = self.header_timestamp
        return markers

    def create_point_array(self) -> PointArray:
        point_arr = PointArray()
        point_arr.header.frame_id = self.frame_id
        point_arr.header.stamp = self.header_timestamp
        return point_arr

    def zed_object_to_point(self, object: sl.ObjectData) -> Point:
        point = Point()
        point.x = object.position[0]
        point.y = object.position[1]
        point.z = object.position[2]
        return point

    def record_timestamp(self):
            self.timestamp = self.get_clock().now()

    def delta_time(self, start_timestamp=None) -> float:
        if start_timestamp is None:
            return (self.get_clock().now() - self.timestamp).nanoseconds / 1000000000
        else:
            return (self.get_clock().now() - start_timestamp).nanoseconds / 1000000000

    def get_rviz_markers(object_list: sl.ObjectData, header_timestamp) -> MarkerArray:
        markerArray: MarkerArray = MarkerArray()

        for object_index, object in enumerate(object_list):
            marker = Marker()
            marker.id = object_index
            marker.header.stamp = header_timestamp
            marker.header.frame_id = "zed_left_frame_link"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = object.position[0]
            marker.pose.position.y = object.position[1]
            marker.pose.position.z = object.position[2]
            marker.lifetime = rclpy.time.Duration(seconds=0).to_msg()
            marker.frame_locked = False
            markerArray.markers.append(marker)

        for i in range(len(object_list), 12):
            marker = Marker()
            marker.id = i
            marker.header.stamp = header_timestamp
            marker.header.frame_id = "zed_left_frame_link"
            marker.action = 2
            marker.lifetime = rclpy.time.Duration(seconds=0).to_msg()
            marker.frame_locked = False
            markerArray.markers.append(marker)

        return markerArray

    def set_svo_index(self, msg):
        self.get_logger().info(f"Setting SVO index to {msg.data}")
        if self.control_svo_index != msg.data:
            self.control_svo_index = int(msg.data)
            self.zed.set_svo_position(self.control_svo_index)
            self.get_logger().info(f"Set SVO index to {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    node = ZedNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()