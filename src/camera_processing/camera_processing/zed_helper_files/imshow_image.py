import cv2, os ,re

def _find_video_index(v4l_byid_name: str):
    if v4l_byid_name is None or len(v4l_byid_name) == 0:
        raise ValueError("Must provide a v4l_byid_name name to find the video index")
    
    v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

    if not os.path.exists(v4l_byid_name):
        raise ValueError("Path doesn't exist when following the v4l_byid_name")
    
    device_path = os.path.realpath(v4l_byid_name)
    device_re = re.compile("\/dev\/video(\d+)")
    info = device_re.match(device_path)
    if not info:
        raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
    
    return int(info.group(1))

def main():
    # v4l_byid_name = "usb-CN0HK46K8LG00114H364A03_Integrated_Webcam_HD_200901010001-video-index0" # Erik's Laptop webcam
    # v4l_byid_name = "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0" # IR Camera 
    # v4l_byid_name = "usb-OmniVision_Technologies__Inc._USB_Camera-B4.09.24.1-video-index0" # ps3 eye camera
    # v4l_byid_name = "usb-e-con_systems_See3CAM_CU27_3B1519112B010900-video-index0" # See3CAM_CU27
    v4l_byid_name = "usb-USB_Camera_Manufacture_H264_USB_Camera_2020052801-video-index0" # ELP IR Cut camera
  
    cap = cv2.VideoCapture(_find_video_index(v4l_byid_name))
    if cap is None or not cap.isOpened():
        raise ValueError("Could not open the camera as " + str(v4l_byid_name))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
