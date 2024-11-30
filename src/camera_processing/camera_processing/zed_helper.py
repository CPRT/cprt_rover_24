import pyzed.sl as sl
import rclpy
from sensor_msgs.msg import Image, Imu, MagneticField
from enum import Enum
from math import pi

DEG2RAD = 2 * pi / 360

class ImageEncoding(Enum):
    """
    Enum for image encodings.
    """
    TYPE_32FC1 = "32FC1"
    TYPE_32FC2 = "32FC2"
    TYPE_32FC3 = "32FC3"
    TYPE_32FC4 = "32FC4"
    MONO8 = "mono8"
    TYPE_8UC2 = "8UC2"
    BGR8 = "bgr8"
    BGRA8 = "bgra8"


def imageToROSMsg(img: sl.Mat, frame_id: str, timestamp) -> Image:
    """
    Convert a ZED image to a ROS message.
    """
    imgMessage = Image()

    imgMessage.header.stamp = timestamp
    imgMessage.header.frame_id = frame_id
    imgMessage.height = img.get_height()
    imgMessage.width = img.get_width()

    imgMessage.is_bigendian = False

    imgMessage.step = img.get_step_bytes()

    dataType: sl.MAT_TYPE = img.get_data_type()

    imgMessage.data = img.get_data(memory_type=sl.MEM.CPU, deep_copy=True).tobytes()

    if dataType == sl.MAT_TYPE.F32_C1: # float 1 channel
        imgMessage.encoding = ImageEncoding.TYPE_32FC1.value

    elif dataType == sl.MAT_TYPE.F32_C2: # float 2 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC2.value

    elif dataType == sl.MAT_TYPE.F32_C3: # float 3 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC3.value

    elif dataType == sl.MAT_TYPE.F32_C4: # float 4 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC4.value

    elif dataType == sl.MAT_TYPE.U8_C1: # unsigned char 1 channel
        imgMessage.encoding = ImageEncoding.MONO8.value

    elif dataType == sl.MAT_TYPE.U8_C2: # unsigned char 2 channels
        imgMessage.encoding = ImageEncoding.TYPE_8UC2.value

    elif dataType == sl.MAT_TYPE.U8_C3: # unsigned char 3 channels
        imgMessage.encoding = ImageEncoding.BGR8.value

    elif dataType == sl.MAT_TYPE.U8_C4: # unsigned char 4 channels
        imgMessage.encoding = ImageEncoding.BGRA8.value

    return imgMessage


def slTime2Ros(t: sl.Timestamp) -> rclpy.time.Time:
    """
    Convert a ZED timestamp to a ROS time.
    """
    return rclpy.time.Time(seconds=t.get_seconds(), nanoseconds=t.get_microseconds() * 1000)


def imuDataToROSMsg(imu_data: sl.IMUData, frame_id: str, timestamp) -> Imu:
    """
    Convert ZED IMU data to a ROS message.
    """
    imuMessage = Imu()

    imuMessage.header.stamp = timestamp
    imuMessage.header.frame_id = frame_id

    imuMessage.orientation.x = imu_data.get_pose().get_orientation().get()[0]
    imuMessage.orientation.y = imu_data.get_pose().get_orientation().get()[1]
    imuMessage.orientation.z = imu_data.get_pose().get_orientation().get()[2]
    imuMessage.orientation.w = imu_data.get_pose().get_orientation().get()[3]

    imuMessage.angular_velocity.x = imu_data.get_angular_velocity()[0] * DEG2RAD
    imuMessage.angular_velocity.y = imu_data.get_angular_velocity()[1] * DEG2RAD
    imuMessage.angular_velocity.z = imu_data.get_angular_velocity()[2] * DEG2RAD

    imuMessage.linear_acceleration.x = imu_data.get_linear_acceleration()[0]
    imuMessage.linear_acceleration.y = imu_data.get_linear_acceleration()[1]
    imuMessage.linear_acceleration.z = imu_data.get_linear_acceleration()[2]

    # Covariances copy
    for i in range(0, 3):
        r = 0

        if i == 0:
            r = 0
        elif i == 1:
            r = 1
        else:
            r = 2

        imuMessage.orientation_covariance[i * 3 + 0] = imu_data.get_pose_covariance().r[r][0] * DEG2RAD * DEG2RAD
        imuMessage.orientation_covariance[i * 3 + 1] = imu_data.get_pose_covariance().r[r][1] * DEG2RAD * DEG2RAD
        imuMessage.orientation_covariance[i * 3 + 2] = imu_data.get_pose_covariance().r[r][2] * DEG2RAD * DEG2RAD
      
        imuMessage.angular_velocity_covariance[i * 3 + 0] = imu_data.get_angular_velocity_covariance().r[r][0] * DEG2RAD * DEG2RAD
        imuMessage.angular_velocity_covariance[i * 3 + 1] = imu_data.get_angular_velocity_covariance().r[r][1] * DEG2RAD * DEG2RAD
        imuMessage.angular_velocity_covariance[i * 3 + 2] = imu_data.get_angular_velocity_covariance().r[r][2] * DEG2RAD * DEG2RAD

        imuMessage.linear_acceleration_covariance[i * 3 + 0] = imu_data.get_linear_acceleration_covariance().r[r][0]
        imuMessage.linear_acceleration_covariance[i * 3 + 1] = imu_data.get_linear_acceleration_covariance().r[r][1]
        imuMessage.linear_acceleration_covariance[i * 3 + 2] = imu_data.get_linear_acceleration_covariance().r[r][2]

    return imuMessage


def magneticDataToROSMsg(mag_data: sl.MagnetometerData, frame_id: str, timestamp) -> MagneticField:
    """
    Convert ZED Magnetometer data to a MagneticField ROS message.
    """
    mag_msg = MagneticField()

    mag_msg.header.stamp = timestamp
    mag_msg.header.frame_id = frame_id

    mag_msg.magnetic_field.x = mag_data.get_magnetic_field_calibrated()[0] * 1e-6 # convert micro tesla to tesla
    mag_msg.magnetic_field.y = mag_data.get_magnetic_field_calibrated()[1] * 1e-6
    mag_msg.magnetic_field.z = mag_data.get_magnetic_field_calibrated()[2] * 1e-6

    mag_msg.magnetic_field_covariance[0] = 0.039e-6
    mag_msg.magnetic_field_covariance[1] = 0.0
    mag_msg.magnetic_field_covariance[2] = 0.0
    mag_msg.magnetic_field_covariance[3] = 0.0
    mag_msg.magnetic_field_covariance[4] = 0.037e-6
    mag_msg.magnetic_field_covariance[5] = 0.0
    mag_msg.magnetic_field_covariance[6] = 0.0
    mag_msg.magnetic_field_covariance[7] = 0.0
    mag_msg.magnetic_field_covariance[8] = 0.047e-6

    return mag_msg