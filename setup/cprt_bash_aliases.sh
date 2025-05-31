# CPRT Bash Aliases

if [ "$0" = "$BASH_SOURCE" ]; then
    echo "Error: cprt_bash_aliases.sh must be sourced"
    exit 1
fi

echo "Added CPRT bash aliases"

debug=0

create_alias_with_echo() {
  local alias_name="$1"
  local command_string="$2"

  if [ "$debug" != 0 ]; then
    echo "Adding alias with name: $alias_name and command: $command_string"
  fi

  alias "$alias_name"="echo Running: '$command_string'; eval \"$command_string\""
}

alias ea='nano $ROVER_DIR/setup/cprt_bash_aliases.sh'
alias eaNano='nano $ROVER_DIR/setup/cprt_bash_aliases.sh'
alias eaVim='vim $ROVER_DIR/setup/cprt_bash_aliases.sh'

alias home='cd $ROVER_DIR'
alias gohome='cd $ROVER_DIR'
alias goRoverHome='cd $ROVER_DIR'
alias goWebHome='cd $WEB_DIR'
alias homeWeb='cd $WEB_DIR'

alias roverSource='source $ROVER_DIR/install/setup.bash'
rover_source_name=roverSource # Used below

create_alias_with_echo goLatLonDir 'cd $ROVER_DIR/src/nav_commanders/known_gps_coords'

create_alias_with_echo pingBaseAntenna 'ping 192.168.0.2'
create_alias_with_echo pingRoverAntenna 'ping 192.168.0.3'
create_alias_with_echo pingJetson 'ping 192.168.0.55'
create_alias_with_echo pingJetsonWifi 'ping 192.168.1.111'
create_alias_with_echo pingBattleStation 'ping 192.168.0.20'
create_alias_with_echo pingBattleStationWifi 'ping 192.168.1.100'

create_alias_with_echo sshJetson 'ssh cprt@192.168.0.55'
create_alias_with_echo sshJetsonWifi 'ssh cprt@192.168.1.111'
create_alias_with_echo sshBattleStation 'ssh cprt@192.168.0.20'
create_alias_with_echo sshBattleStationWifi 'ssh cprt@192.168.1.100'

create_alias_with_echo lsCams 'ls /dev/v4l/by-id'

create_alias_with_echo openWebUI 'chromium http://localhost:3000'
create_alias_with_echo openRoverCode 'code $ROVER_DIR'
create_alias_with_echo openWebCode 'code $WEB_DIR'

create_alias_with_echo rvizMoveit 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'
create_alias_with_echo rvizArmMoveIt 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'
create_alias_with_echo rvizNav 'rviz2 -d $ROVER_DIR/src/navigation/rviz/nav2_cprt.rviz'


# Launch helpers

create_alias_with_echo launchBasestation "$rover_source_name && ros2 launch bringup basestation.launch.py" 
create_alias_with_echo launchEquipementServicing "$rover_source_name && ros2 launch bringup equipment_servicing.launch.py"
create_alias_with_echo launchNav "$rover_source_name && ros2 launch bringup nav.launch.py"
create_alias_with_echo launchScience "$rover_source_name && ros2 launch bringup science.launch.py"
create_alias_with_echo launchTraversal "$rover_source_name && ros2 launch bringup transversal.launch.py"


setDriveCameraExposure() {
    local camera_device=""
    local exposure_value="$1"

    # Check for the first camera device path
    if [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
    # Check for the second camera device path
    elif [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
    else
        echo "Error: Drive camera device not found at expected paths."
        echo "Please ensure one of the following paths exists:"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$exposure_value" ]]; then
        echo "Usage: setDriveCameraExposure <exposure_value>"
        echo "Exposure value must be an integer between 5 and 330 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    if ! [[ "$exposure_value" =~ ^[0-9]+$ ]]; then
        echo "Error: Exposure value '$exposure_value' is not a valid integer."
        return 1
    fi

    if (( exposure_value < 5 || exposure_value > 330 )); then
        echo "Error: Exposure value must be between 5 and 330 (inclusive)."
        return 1
    fi

    echo "Setting Drive Camera exposure to $exposure_value using device: $camera_device"
    # Execute the v4l2-ctl command
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute="$exposure_value"

    if [[ $? -eq 0 ]]; then
        echo "Exposure set successfully."
    else
        echo "Failed to set exposure. Check permissions or camera status."
    fi
} 

setDriveCameraAutoExposure() {
    local camera_device=""

    # Check for the first camera device path
    if [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
    # Check for the second camera device path
    elif [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
    else
        echo "Error: Gimbal camera device not found at expected paths."
        echo "Please ensure one of the following paths exists:"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
        return 1
    fi

    echo "Turning on auto exposure for Gimbal camera using device: $camera_device"
    # Execute the v4l2-ctl command to set auto_exposure to 0 (Auto Mode)
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=0

    if [[ $? -eq 0 ]]; then
        echo "Auto exposure turned on successfully."
    else
        echo "Failed to turn on auto exposure. Check permissions or camera status."
    fi
}
 
setDriveCameraBrightness() {
    local camera_device=""
    local brightness_value="$1"

    # Check for the first camera device path
    if [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
    # Check for the second camera device path
    elif [[ -e "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0" ]]; then
        camera_device="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
    else
        echo "Error: Drive camera device not found at expected paths."
        echo "Please ensure one of the following paths exists:"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP_-video-index0"
        echo "  /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_20231205_0001-video-index0"
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$brightness_value" ]]; then
        echo "Usage: setDriveCameraBrightness <brightness_value>"
        echo "Exposure value must be an integer between -64 and 64 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    # if ! [[ "$brightness_value" =~ ^[0-9]+$ ]]; then
    #     echo "Error: Brightness value '$brightness_value' is not a valid integer."
    #     return 1
    # fi

    if (( brightness_value < -64 || brightness_value > 64 )); then
        echo "Error: Brightness value must be between -64 and 64 (inclusive)."
        return 1
    fi

    echo "Setting Drive Camera brightness to $brightness_value using device: $camera_device"
    # Execute the v4l2-ctl command
    v4l2-ctl -d "$camera_device" --set-ctrl=brightness="$brightness_value"

    if [[ $? -eq 0 ]]; then
        echo "Brightness set successfully."
    else
        echo "Failed to set exposure. Check permissions or camera status."
    fi
} 

setBellyCameraExposure() {
    local camera_device="/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0"
    local exposure_value="$1"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Belly camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$exposure_value" ]]; then
        echo "Usage: setBellyCameraExposure <exposure_value>"
        echo "Exposure value must be an integer between 1 and 5000 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    if ! [[ "$exposure_value" =~ ^[0-9]+$ ]]; then
        echo "Error: Exposure value '$exposure_value' is not a valid integer."
        return 1
    fi

    # Based on your list-ctrls, exposure_time_absolute min=1 max=5000
    if (( exposure_value < 1 || exposure_value > 5000 )); then
        echo "Error: Exposure value must be between 1 and 5000 (inclusive)."
        return 1
    fi

    echo "Setting Belly Camera exposure to $exposure_value using device: $camera_device"
    # Execute the v4l2-ctl command
    # Set auto_exposure to 1 (Manual Mode) and then set exposure_time_absolute
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute="$exposure_value"

    if [[ $? -eq 0 ]]; then
        echo "Exposure set successfully."
    else
        echo "Failed to set exposure. Check permissions or camera status."
    fi
}

setBellyCameraAutoExposure() {
    local camera_device="/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Belly camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    echo "Turning on auto exposure for Belly Camera using device: $camera_device"
    # Execute the v4l2-ctl command to set auto_exposure to 3 (Auto Mode)
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=3

    if [[ $? -eq 0 ]]; then
        echo "Auto exposure turned on successfully."
    else
        echo "Failed to turn on auto exposure. Check permissions or camera status."
    fi
}


setBellyCameraBrightness() {
    local camera_device="/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0"
    local brightness_value="$1"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Belly camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$brightness_value" ]]; then
        echo "Usage: setBellyCameraBrightness <value>"
        echo "Exposure value must be an integer between -64 and 64 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    # if ! [[ "$brightness_value" =~ ^[0-9]+$ ]]; then
    #     echo "Error: Brightness value '$brightness_value' is not a valid integer."
    #     return 1
    # fi

    # Based on your list-ctrls, exposure_time_absolute min=1 max=5000
    if (( brightness_value < -64 || brightness_value > 64 )); then
        echo "Error: Brightness value must be between -64 and 64 (inclusive)."
        return 1
    fi

    echo "Setting Belly Camera brightness to $brightness_value using device: $camera_device"
    # Execute the v4l2-ctl command
    # Set auto_exposure to 1 (Manual Mode) and then set exposure_time_absolute
    v4l2-ctl -d "$camera_device" --set-ctrl=brightness="$brightness_value"

    if [[ $? -eq 0 ]]; then
        echo "Brightness set successfully."
    else
        echo "Failed to set brightness. Check permissions or camera status."
    fi
}


setArmCameraExposure() {
    local camera_device="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0"
    local exposure_value="$1"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Arm camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$exposure_value" ]]; then
        echo "Usage: setArmCameraExposure <exposure_value>"
        echo "Exposure value must be an integer between 1 and 5000 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    if ! [[ "$exposure_value" =~ ^[0-9]+$ ]]; then
        echo "Error: Exposure value '$exposure_value' is not a valid integer."
        return 1
    fi

    # Based on your list-ctrls, exposure_time_absolute min=1 max=5000
    if (( exposure_value < 1 || exposure_value > 5000 )); then
        echo "Error: Exposure value must be between 1 and 5000 (inclusive)."
        return 1
    fi

    echo "Setting Arm Camera exposure to $exposure_value using device: $camera_device"
    # Execute the v4l2-ctl command
    # Set auto_exposure to 1 (Manual Mode) and then set exposure_time_absolute
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute="$exposure_value"

    if [[ $? -eq 0 ]]; then
        echo "Exposure set successfully."
    else
        echo "Failed to set exposure. Check permissions or camera status."
    fi
}

setArmCameraAutoExposure() {
    local camera_device="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Arm camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    echo "Turning on auto exposure for Arm Camera using device: $camera_device"
    # Execute the v4l2-ctl command to set auto_exposure to 3 (Auto Mode)
    v4l2-ctl -d "$camera_device" --set-ctrl=auto_exposure=3

    if [[ $? -eq 0 ]]; then
        echo "Auto exposure turned on successfully."
    else
        echo "Failed to turn on auto exposure. Check permissions or camera status."
    fi
}

setArmCameraBrightness() {
    local camera_device="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0"
    local brightness_value="$1"

    # Check if the camera device path exists
    if [[ ! -e "$camera_device" ]]; then
        echo "Error: Arm camera not found at: $camera_device"
        echo "Please ensure the camera is connected and the device path is correct."
        return 1
    fi

    # Check if an exposure value was provided
    if [[ -z "$brightness_value" ]]; then
        echo "Usage: setArmCameraBrightness <value>"
        echo "Exposure value must be an integer between -64 and 64 (inclusive)."
        return 1
    fi

    # Validate the exposure value
    # if ! [[ "$brightness_value" =~ ^[0-9]+$ ]]; then
    #     echo "Error: Brightness value '$brightness_value' is not a valid integer."
    #     return 1
    # fi

    # Based on your list-ctrls, exposure_time_absolute min=1 max=5000
    if (( brightness_value < -64 || brightness_value > 64 )); then
        echo "Error: Brightness value must be between -64 and 64 (inclusive)."
        return 1
    fi

    echo "Setting Arm Camera brightness to $brightness_value using device: $camera_device"
    # Execute the v4l2-ctl command
    # Set auto_exposure to 1 (Manual Mode) and then set exposure_time_absolute
    v4l2-ctl -d "$camera_device" --set-ctrl=brightness="$brightness_value"

    if [[ $? -eq 0 ]]; then
        echo "Brightness set successfully."
    else
        echo "Failed to set brightness. Check permissions or camera status."
    fi
}


create_alias_with_echo setDefaultV4l2CameraSettings 'setDriveCameraBrightness -25; setArmCameraBrightness 0; setBellyCameraBrightness -10'