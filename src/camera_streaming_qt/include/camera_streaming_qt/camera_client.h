#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <interfaces/msg/video_source.hpp>
#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_out.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class CameraClient : public rclcpp::Node {
 public:
  CameraClient();
  ~CameraClient();

  std::vector<std::string> get_cameras();
  void start_video(int num_sources,
                   std::vector<interfaces::msg::VideoSource> sources);
};

#endif
