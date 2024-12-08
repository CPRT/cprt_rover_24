import rclpy
import rclpy.logging
from rclpy.node import Node
from interfaces.srv import VideoOut
import gi
from gi.repository import Gst

gi.require_version("Gst", "1.0")


class WebRTCStreamer(Node):
    """
    A ROS2 Node that creates a WebRTC stream from multiple video sources using GStreamer.

    This node listens for a 'start_video' service request, builds a GStreamer pipeline
    with the provided video sources, and streams the video through WebRTC.
    The node also supports optional web server functionality to host the stream.

    Attributes:
        web_server (bool): Flag indicating if a web server is enabled for the stream.
        web_server_path (str): Path for the web server directory.
        source_list (dict): A dictionary mapping camera names to device paths.
        pipeline (Gst.Pipeline): The GStreamer pipeline for video streaming.
    """

    def __init__(self):
        """
        Initializes the WebRTCStreamer node, loads parameters, and sets up the service.

        This constructor initializes GStreamer, declares the necessary parameters
        (such as web_server, camera_name, and camera_path), and creates the ROS2 service
        to start the video stream.
        """
        Gst.init(None)
        super().__init__("webrtc_node")
        self.declare_parameter("web_server", False)
        self.declare_parameter("web_server_path", ".")
        self.web_server = (
            self.get_parameter("web_server").get_parameter_value().bool_value
        )
        self.web_server_path = (
            self.get_parameter("web_server_path").get_parameter_value().string_value
        )
        self.start = self.create_service(VideoOut, "start_video", self.start_video_cb)
        self.declare_parameter("camera_name", [""])
        self.declare_parameter("camera_path", [""])

        # Fetch the parameter values
        camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_array_value
        )
        camera_path = (
            self.get_parameter("camera_path").get_parameter_value().string_array_value
        )

        # Convert to dictionary format
        self.source_list = {}
        for name, path in zip(camera_name, camera_path):
            self.source_list[name] = path
        self.pipeline = None

    def start_video_cb(self, request, response):
        """
        Callback function for starting the video stream.

        This function constructs a GStreamer pipeline based on the requested sources,
        starts the pipeline, and returns a success response.

        Args:
            request (VideoOut.Request): The service request containing video stream details.
            response (VideoOut.Response): The service response to return success or failure.

        Returns:
            VideoOut.Response: The response indicating success or failure.
        """
        pipeline_str = self.create_pipeline(request)
        self.get_logger().info(pipeline_str)
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except:
                self.pipeline = None
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.pipeline.set_state(Gst.State.PLAYING)
            response.success = True
        except:
            response.success = False
        return response

    def create_source(self, name):
        """
        Creates a GStreamer source element based on the camera name.

        Args:
            name (str): The name of the camera source.

        Returns:
            str: A GStreamer pipeline source element for the camera.
        """
        if name == "test":
            return "videotestsrc"
        return f"v4l2src device={self.source_list[name]}"

    def create_pipeline(self, request):
        """
        Creates the GStreamer pipeline string based on the service request.

        This function generates the GStreamer pipeline to combine multiple video sources
        and set up the compositor, applying the required video properties (width, height, position).

        Args:
            request (VideoOut.Request): The service request containing details of the video sources.

        Returns:
            str: The GStreamer pipeline string ready for launch.
        """
        pipeline = ""
        compositor = "compositor name=mix"
        total_width = request.width
        total_height = request.height
        i = 0
        for source in request.sources:
            name = source.name
            height = int(source.height * total_height / 100)
            width = int(source.width * total_width / 100)
            origin_x = int(source.origin_x * total_width / 100)
            origin_y = int(source.origin_y * total_height / 100)
            pipeline += f'{self.create_source(name)} ! nvvidconv ! capsfilter caps="video/x-raw,height={height},width={width}" ! mix.sink_{i} '
            compositor += f" sink_{i}::xpos={origin_x} sink_{i}::ypos={origin_y} sink_{i}::height={height} sink_{i}::width={width}"
            i = i + 1
        video_out = "webrtcsink run-signalling-server=true"
        if self.web_server:
            video_out += f" run-web-server=true web-server-host-addr=http://0.0.0.0:8080/ web-server-directory={self.web_server_path}"
        pipeline += compositor + " ! " + video_out
        return pipeline


def main(args=None):
    """
    The main entry point of the program.

    This function initializes the ROS2 system, creates the WebRTCStreamer node,
    and starts the ROS2 event loop to process incoming requests.

    Args:
        args (list, optional): Arguments passed from the command line (default is None).
    """
    rclpy.init(args=args)
    node = WebRTCStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
