import argparse
from run_roboflow_model import RunRoboflowModel

if __name__ == "__main__":

    model_id = "orange-hammer-detection-desert/2"

    parser = argparse.ArgumentParser(description="Process video for orange mallet detection.")
    parser.add_argument("--output-file", type=str, help="Path to save the output video.")
    parser.add_argument("--live", type=int, default=1, help="Device ID for live video input.")
    parser.add_argument("--im-show", action="store_true", help="Flag to display video frames using OpenCV imshow.")
    parser.add_argument("--input-file", type=str, help="Path to the input video file.")

    parser.add_argument("--model1", action="store_true", help="Use model 1")

    args = parser.parse_args()

    if args.model1:
        model_id = "orange-hammer-detection-desert/2"

    

    detector = RunRoboflowModel(
        model_id=model_id, 
        output_path=args.output_file, 
        live_device=args.live, 
        im_show=args.im_show, 
        input_file=args.input_file)
    
    detector.run()