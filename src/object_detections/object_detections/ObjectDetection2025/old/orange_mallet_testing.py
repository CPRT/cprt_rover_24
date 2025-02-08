# import the InferencePipeline interface
from inference import InferencePipeline
# import a built in sink called render_boxes (sinks are the logic that happens after inference)
from inference.core.interfaces.stream.sinks import render_boxes

from pathlib import Path



api_key_file = Path('/home') / "erikjetsonsd2" / ".ssh" / "RoboflowApiKey"
api_key = ""
with api_key_file.open(mode='r') as file:
    api_key = file.read()

if api_key == "":
    print("Failed to load api key")
    exit(1)


# create an inference pipeline object
pipeline = InferencePipeline.init(
    model_id="orange-hammer-detection-desert/2", # set the model id to a yolov8x model with in put size 1280
    video_reference=1, # "your_video.mp4", # set the video reference (source of video), it can be a link/path to a video file, an RTSP stream url, or an integer representing a device id (usually 0 for built in webcams)
    on_prediction=render_boxes, # tell the pipeline object what to do with each set of inference by passing a function
    api_key="2m1j9MfDiUIr5Z8qwMom", # provide your roboflow api key for loading models from the roboflow api
)

# start the pipeline
pipeline.start()
# wait for the pipeline to finish
pipeline.join()