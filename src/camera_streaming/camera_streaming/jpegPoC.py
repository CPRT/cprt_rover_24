import rclpy
import rclpy.logging
import random
from rclpy.node import Node
from interfaces.srv import ChangeRes
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

pipeline_name = 'p0'


class jpegPoC(Node):
    def __init__(self):
        Gst.init(None)
        super().__init__("JpegPoCNode")
        self.declare_parameter("port", 7001)
        self.target_port = (
            self.get_parameter("port").get_parameter_value().integer_value
        )
        self.declare_parameter("host", "127.0.0.1")
        self.target_host = (
            self.get_parameter("host").get_parameter_value().string_value
        )
        self.declare_parameter("device", "/dev/video0")
        self.device = (
            self.get_parameter("device").get_parameter_value().string_value
        )
        self.declare_parameter("height", 1080)
        self.height = (
            self.get_parameter("height").get_parameter_value().integer_value
        )
        self.declare_parameter("width", 1920)
        self.width = (
            self.get_parameter("width").get_parameter_value().integer_value
        )
        self.declare_parameter("interval", 1000)
        self.interval = (
            self.get_parameter("interval").get_parameter_value().integer_value
        )
        self.declare_parameter("max_framerate", 60)
        self.max_framerate = (
            self.get_parameter("max_framerate").get_parameter_value().integer_value
        )
        self.declare_parameter("init_framerate", 1)
        init_framerate = (
            self.get_parameter("init_framerate").get_parameter_value().integer_value
        )
        self.timer = self.create_timer(self.interval/1000, self.check_bandwidth)
        self.srv = self.create_service(ChangeRes, 'change_resolution', self.change_res_callback)
        self.frame_rate = init_framerate
        self.last_timestamp = None
        self.create_pipeline()


    def create_pipeline(self):
        pipeline_string = f'v4l2src device={self.device} do-timestamp=true name=srcElement ! capsfilter name=caps ! videoconvert ! nvvidconv ! nvjpegenc ! rtpjpegpay ! udpsink host={self.target_host} port={self.target_port} sync=false'
        self.pipeline = Gst.parse_launch(pipeline_string)
        self.get_logger().info(pipeline_string)
        self.pipeline.set_state(Gst.State.PLAYING)
        src = self.pipeline.get_by_name("srcElement")
        src_pad = src.get_static_pad("src")
        src_pad.add_probe(Gst.PadProbeType.BUFFER, self.buffer_probe)
    
    def buffer_probe(self, pad, info):
        buffer = info.get_buffer()

        if buffer is None:
            return Gst.PadProbeReturn.OK
        
        if self.last_timestamp is not None:
            time_diff = buffer.pts - self.last_timestamp
            if time_diff < (Gst.SECOND / self.frame_rate):
                self.get_logger().debug(f"Dropping buffer. Time since last buffer: {time_diff:.2f}s")
                return Gst.PadProbeReturn.DROP

        self.last_timestamp = buffer.pts
        self.get_logger().debug(f"Sending buffer. Time since last buffer: {time_diff:.2f}s")
        return Gst.PadProbeReturn.OK

    def change_res_callback(self, request, response):
        response.success = True
        return response
    
    def check_bandwidth(self):
        # TODO (get bandwidth estimate from link)
        # For now, pick random numbers
        bandwidth = random.randint(10, 100)
        in_use = random.randint(1, bandwidth)
        # --------------------- #
        usage = in_use / bandwidth
        if self.update_bandwidth(usage):
            self.get_logger().info(f"Framerate changed. Bandwidth: {bandwidth} In use: {in_use} ({usage * 100:.2f}%), new frame rate: {self.frame_rate}")
        return
    
    def update_bandwidth(self, usage):
        # TODO make this more configurable
        if usage > 0.95:
            self.frame_rate = int(self.frame_rate * 0.5)
        elif usage > 0.8:
            self.frame_rate = self.frame_rate - 1
        elif usage < 0.5:
            self.frame_rate = self.frame_rate + 1
        else:
            # Bandwidth usage is fine
            return False

        # Update framerate
        if self.frame_rate < 1:
            self.frame_rate = 1
        elif self.frame_rate > self.max_framerate:
            self.frame_rate = self.max_framerate
        return True

    def refreshCaps(self):
        element = self.pipeline.get_by_name("caps")
        if not element:
            self.get_logger().error("Element not found")
            return
        self.pipeline.set_state(Gst.State.NULL)
        new_caps = Gst.Caps.from_string(f"image/x-raw, height={self.height}, width={self.width}")
        element.set_property("caps", new_caps)
        self.pipeline.set_state(Gst.State.PLAYING)

def main(args=None):
    rclpy.init(args=args)
    node = jpegPoC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()