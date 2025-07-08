#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <gst/gst.h>
#include <gst/video/videooverlay.h>

#include <QWidget>
#include <interfaces/msg/video_source.hpp>
#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_out.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class CameraClient : public rclcpp::Node {
 public:
  typedef struct GstData {
    GstElement* pipeline;
    GstElement* source;
    GstElement* convert;
    GstElement* sink;
    WId winId;
  } GstData;

  CameraClient();
  ~CameraClient();

  std::vector<std::string> get_cameras();
  void start_video(int num_sources,
                   std::vector<interfaces::msg::VideoSource> sources);

 private:
  void open_gst_widget();
  GstBusSyncReply bus_sync_handler(GstBus* bus, GstMessage* message,
                                   GstData* data);
};

#endif
