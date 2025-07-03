/**
 * @file test_get_camera_node.h
 * @brief Header file for the GetCameraNode class.
 * @author Aria Wong
 * 
 * This file contains the declaration of the GetCameraNode which is used as a test class for the get_cameras and start_video services.
 */
#ifndef TEST_GET_CAMERA_NODE_H
#define TEST_GET_CAMERA_NODE_H

#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_out.hpp>
#include <rclcpp/rclcpp.hpp>

class GetCameraNode : public rclcpp::Node {
 public:
  GetCameraNode();
  ~GetCameraNode();

  void get_cameras(
      const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
      std::shared_ptr<interfaces::srv::GetCameras::Response> response);
  rclcpp::Service<interfaces::srv::GetCameras>::SharedPtr get_cameras_service_;

  void start_video(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
      std::shared_ptr<interfaces::srv::VideoOut::Response> response);
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr start_video_service_;
};

#endif