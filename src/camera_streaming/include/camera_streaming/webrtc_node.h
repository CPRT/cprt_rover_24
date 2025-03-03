#ifndef WEBRTC_STREAMER_HPP
#define WEBRTC_STREAMER_HPP

#include <gst/gst.h>

#include <interfaces/srv/video_out.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class WebRTCStreamer : public rclcpp::Node {
 public:
  WebRTCStreamer();
  ~WebRTCStreamer();
  enum class CameraType { V4l2Src = 0, TestSrc };
  struct CameraSource {
    std::string name;
    std::string path;
    CameraType type;
  };

 private:
  void start_video_cb(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
      std::shared_ptr<interfaces::srv::VideoOut::Response> response);

  GstElement* create_source(const CameraSource& src);
  bool update_pipeline(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request);
  GstElement* initialize_pipeline();
  void unlink_sources_from_compositor();
  static bool unlink_pad(GstPad* pad);
  GstElement* create_vid_conv();

  bool web_server_;
  std::string web_server_path_;
  std::vector<GstElement*> connected_sources_;
  GstElement* pipeline_;
  GstElement* compositor_;
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr start_video_service_;
  static gboolean on_bus_message(GstBus* bus, GstMessage* message,
                                 gpointer user_data);
};

#endif  // WEBRTC_STREAMER_HPP
