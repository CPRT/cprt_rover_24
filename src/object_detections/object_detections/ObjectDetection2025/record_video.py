import cv2
import argparse

def record_camera(output_file, device_id=0):
    # Open the video capture
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    # Get the width and height of the frame
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_file, fourcc, 30, (width, height))

    if not out.isOpened():
        print("Error: Could not open video writer.")
        cap.release()
        return

    print("Recording... Press 'q' to stop.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Write the frame to the output file
        out.write(frame)

        # Display the frame
        cv2.imshow('Recording', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release everything
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record video from camera and save to file.")
    parser.add_argument("--output-file", type=str, required=True, help="Path to save the output video.")
    parser.add_argument("--device-id", type=int, default=0, help="Device ID for the camera (default is 0).")

    args = parser.parse_args()

    record_camera(output_file=args.output_file, device_id=args.device_id)