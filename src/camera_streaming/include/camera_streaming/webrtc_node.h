/**
 * @file webrtc_node.h
 * @brief Header file for the WebRTCStreamer class.
 * @author Connor Needham
 *
 * This file contains the declaration of the WebRTCStreamer class, which is
 * responsible for handling video streaming using WebRTC and GStreamer.
 */

#ifndef WEBRTC_STREAMER_HPP
#define WEBRTC_STREAMER_HPP

#include <gst/gst.h>

#include <interfaces/srv/video_out.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

template <typename T>
struct GstDeleter {
  void operator()(T* object) const {
    if (object) {
      gst_object_unref(object);
    }
  }
};

// Type alias for unique_ptr managing a GStreamer object
template <typename T>
using GstUniquePtr = std::unique_ptr<T, GstDeleter<T>>;

/**
 * @class WebRTCStreamer
 * @brief A class for streaming video using WebRTC and GStreamer.
 *
 * The WebRTCStreamer class provides functionality to start video streaming
 * from different camera sources, manage the GStreamer pipeline, and handle
 * video output requests.
 */
class WebRTCStreamer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for WebRTCStreamer.
   */
  WebRTCStreamer();

  /**
   * @brief Destructor for WebRTCStreamer.
   */
  ~WebRTCStreamer();

  /**
   * @enum CameraType
   * @brief Enum representing the type of camera source.
   */
  enum class CameraType {
    V4l2Src = 0, /**< V4L2 source */
    TestSrc      /**< Test source */
  };

  /**
   * @struct CameraSource
   * @brief Struct representing a camera source.
   */
  struct CameraSource {
    std::string name;
    std::string path;
    CameraType type;
  };

 private:
  /**
   * @brief Callback function to start video streaming.
   *
   * @param request The request object containing video output parameters.
   * @param response The response object to be populated with the result.
   */
  void start_video_cb(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
      std::shared_ptr<interfaces::srv::VideoOut::Response> response);

  /**
   * @brief Creates a GStreamer source element for the given camera source.
   *
   * @param src The camera source information.
   * @return A pointer to the created GStreamer element.
   */
  GstElement* create_source(const CameraSource& src);

  /**
   * @brief Updates the GStreamer pipeline based on the video output request.
   *
   * @param request The request object containing video output parameters.
   * @return True if the pipeline was successfully updated, false otherwise.
   */
  bool update_pipeline(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request);

  /**
   * @brief Initializes the GStreamer pipeline.
   *
   * @return A pointer to the initialized GStreamer pipeline element.
   */
  GstElement* initialize_pipeline();

  /**
   * @brief Unlinks current sources from the compositor element.
   */
  void unlink_sources_from_compositor();

  /**
   * @brief Unlinks a specific pad from its peer.
   *
   * @param pad The pad to be unlinked.
   * @return True if the pad was successfully unlinked, false otherwise.
   */
  static bool unlink_pad(GstPad* pad);

  /**
   * @brief Creates a GStreamer video converter element.
   *
   * @return A pointer to the created video converter element.
   */
  GstElement* create_vid_conv();

  bool web_server_; /**< Flag indicating if the web server is enabled */
  std::string web_server_path_; /**< Path to the web server */
  std::vector<GstElement*>
      connected_sources_; /**< List of connected GStreamer source elements */
  GstUniquePtr<GstElement> pipeline_; /**< GStreamer pipeline element */
  GstElement* compositor_;            /**< GStreamer compositor element */
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr
      start_video_service_; /**< ROS2 service for starting video output */

  /**
   * @brief Callback function for handling GStreamer bus messages.
   *
   * @param bus The GStreamer bus.
   * @param message The GStreamer message.
   * @param user_data User data passed to the callback.
   * @return True if the message was successfully handled, false otherwise.
   */
  static gboolean on_bus_message(GstBus* bus, GstMessage* message,
                                 gpointer user_data);
};

#endif  // WEBRTC_STREAMER_HPP
