import os
import cv2
import supervision as sv
from inference import InferencePipeline
from inference.core.interfaces.camera.entities import VideoFrame

input_folder = "./server/videos"
output_folder = "./server/annotated_videos"
os.makedirs(output_folder, exist_ok=True)

label_annotator = sv.LabelAnnotator()
box_annotator = sv.BoxAnnotator()

def process_video(video_path, output_path):
    video_writer = None

    def my_custom_sink(predictions: dict, video_frame: VideoFrame):
        nonlocal video_writer

        labels = [p["class"] for p in predictions.get("predictions", [])]
        detections = sv.Detections.from_inference(predictions)
        image = label_annotator.annotate(video_frame.image.copy(), detections=detections, labels=labels)
        image = box_annotator.annotate(image, detections=detections)

        if video_writer is None:
            height, width, _ = image.shape
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(output_path, fourcc, 30, (width, height))
            if not video_writer.isOpened():
                print("VideoWriter failed to open! Check output path and permissions.")

        if video_writer:
            video_writer.write(image)

        cv2.imshow("Predictions", image)
        cv2.waitKey(1)

    pipeline = InferencePipeline.init(
        model_id="fire-and-smoke-detection-o4uhv/2",
        video_reference=video_path,
        on_prediction=my_custom_sink,
    )

    pipeline.start()
    pipeline.join()

    if video_writer is not None:
        video_writer.release()
    else:
        print(f"No frames written for {video_path}")

    cv2.destroyAllWindows()

def process_all_videos():
    for filename in os.listdir(input_folder):
        if filename.lower().endswith((".mp4", ".avi", ".mov")):
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, f"annotated_{filename}")
            process_video(input_path, output_path)
