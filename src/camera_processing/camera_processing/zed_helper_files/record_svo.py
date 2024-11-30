import argparse
from pyzed import sl
from time import sleep


def main():
    parser = argparse.ArgumentParser(
        prog="Record ZED svo2 files to playback later",
        description="Resolution can be HD720, HD1080, HD1200, HD2K")
    
    
    parser.add_argument('-n', '--name')
    parser.add_argument('-r', '--resolution')
    parser.add_argument('-d', '--date')
    parser.add_argument('-f', '--directory')

    args = parser.parse_args()

    if not args.name or args.name is None:
        print("Must specify --name option")
        exit()

    if not args.resolution or args.resolution is None:
        print("Must specify --resolution option")
        exit()

    if args.resolution == "HD720":
        resolution_zed = sl.RESOLUTION.HD720
    elif args.resolution == "HD1080":
        resolution_zed = sl.RESOLUTION.HD1080
    elif args.resolution == "HD1200":
        resolution_zed = sl.RESOLUTION.HD1200
    elif args.resolution == "HD2K":
        resolution_zed = sl.RESOLUTION.HD2K
    else:
        print("Invalid option for --resolution. Must be HD720, HD1080, HD1200, HD2K")
        exit()

    if args.date and args.date is not None:
        filename = f"{args.resolution}-{args.date}-{args.name}.svo2"
    else:
        filename = f"{args.resolution}-{args.name}.svo2"


    if args.directory and args.directory is not None:
        filename = f"{args.directory}{filename}" 

    print(f"Filename: {filename}")


    init_params = sl.InitParameters(svo_real_time_mode=False)
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Can use PERFORMANCE MODE but it misses some details (sl.DEPTH_MODE.ULTRA) (sl.DEPTH_MODE.PERFORMANCE)
    init_params.depth_maximum_distance = 15
    init_params.camera_resolution = sl.RESOLUTION.HD1080   # HD720   HD1080   HD1200    HD2K
    init_params.camera_fps = 15 # Use 15 FPS to improve low-light performance

    zed = sl.Camera()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        if status == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
            print("Error opening: CAMERA_NOT_DETECTED. Rebooting and trying again")
            sl.Camera.reboot(sn=0, full_reboot=True) # Reboot fixes USB problems on Jetson Nano
            main()
        else:
            print(f"Error opening: {repr(status)}")
            exit()


    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS  # CUSTOM_BOX_OBJECTS  MULTI_CLASS_BOX_MEDIUM
    obj_param.enable_tracking = False
    zed.enable_object_detection(obj_param)

    recordingParameters = sl.RecordingParameters()
    recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recordingParameters.video_filename = filename
    print(f"Starting to record the ZED data to an SVO named {recordingParameters.video_filename} with compression format {recordingParameters.compression_mode.name}")
    err = zed.enable_recording(recordingParameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to enable SVO recording on ZED. Error: {err}")
        exit()

    i = 1
    try:
        status = sl.ERROR_CODE.SUCCESS
        while status == sl.ERROR_CODE.SUCCESS:
            status = zed.grab()
            print(f"Grabbed image! ({i})")
            i += 1

    except KeyboardInterrupt:
        pass
    finally:
        print(f"Stopping recording")
        zed.disable_recording()
        sleep(5)
        print(f"Closing Zed")
        zed.close()

    print(f"ZED Closed. Exiting")



if __name__ == "__main__":
    main()
