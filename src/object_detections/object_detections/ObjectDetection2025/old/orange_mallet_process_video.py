import os
import cv2
import argparse
from inference import InferencePipeline
from inference.core.interfaces.camera.entities import VideoFrame
import supervision as sv


class DetectOrangeMallet:

    def __init__(self, output_path, live_device, im_show, input_file):
        self.label_annotator = sv.LabelAnnotator()
        self.box_annotator = sv.BoxAnnotator()

        self.video_writer = None
        self.output_path = output_path
        self.im_show = im_show

        video_reference = live_device if input_file is None else input_file

        self.pipeline = InferencePipeline.init(
            model_id="orange-hammer-detection-desert/2", # set the model id to a yolov8x model with in put size 1280
            video_reference=video_reference, # "your_video.mp4", # set the video reference (source of video), it can be a link/path to a video file, an RTSP stream url, or an integer representing a device id (usually 0 for built in webcams)
            on_prediction=self.custom_sink, # tell the pipeline object what to do with each set of inference by passing a function
            api_key="2m1j9MfDiUIr5Z8qwMom", # provide your roboflow api key for loading models from the roboflow api
        )

        self.key = None

    def run(self):
        self.pipeline.start(use_main_thread=False)

        running = True  # Control loop execution

        try:
            while running:
                if self.key == ord('q') or self.key == 27:  # 'q' or ESC key
                    running = False
                else:
                    pass
        except KeyboardInterrupt:
            pass

        print("calling terminate")
        self.pipeline.terminate() # stop the pipeline before joining
        print("calling join")
        self.pipeline.join()

        if self.video_writer is not None:
            self.video_writer.release()
        elif self.output_path is not None:
            print(f"No frames written for {self.video_path}")

        cv2.destroyAllWindows()

    def custom_sink(self, predictions: dict, video_frame: VideoFrame):

        labels = [p["class"] for p in predictions.get("predictions", [])]
        detections = sv.Detections.from_inference(predictions)
        image = self.label_annotator.annotate(video_frame.image.copy(), detections=detections, labels=labels)
        image = self.box_annotator.annotate(image, detections=detections)

        if self.output_path is not None:
            if self.video_writer is None:
                print("Constructing VideoWriter")
                height, width, _ = image.shape
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self.video_writer = cv2.VideoWriter(self.output_path, fourcc, 30, (width, height))
                if not self.video_writer.isOpened():
                    print("VideoWriter failed to open! Check output path and permissions.")
    
            if self.video_writer:
                self.video_writer.write(image)

        if self.im_show:
            cv2.imshow("Predictions", image)
            self.key = cv2.waitKey(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process video for orange mallet detection.")
    parser.add_argument("--output-file", type=str, help="Path to save the output video.")
    parser.add_argument("--live", type=int, default=1, help="Device ID for live video input.")
    parser.add_argument("--im-show", action="store_true", help="Flag to display video frames using OpenCV imshow.")
    parser.add_argument("--input-file", type=str, help="Path to the input video file.")

    args = parser.parse_args()

    print(args)

    detector = DetectOrangeMallet(output_path=args.output_file, live_device=args.live, im_show=args.im_show, input_file=args.input_file)
    detector.run()