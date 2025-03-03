#include "webrtc_node.h"

#include <filesystem>

#include "rclcpp/executors.hpp"

WebRTCStreamer::WebRTCStreamer()
    : Node("webrtc_node"), pipeline_(nullptr), compositor_(nullptr) {
  gst_init(nullptr, nullptr);

  // Declare parameters
  this->declare_parameter("web_server", true);
  this->declare_parameter("web_server_path", ".");
  this->declare_parameter("camera_name", std::vector<std::string>());
  this->get_parameter("web_server", web_server_);
  this->get_parameter("web_server_path", web_server_path_);

  // Set up the service for starting video
  start_video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&WebRTCStreamer::start_video_cb, this,
                               std::placeholders::_1, std::placeholders::_2));

  // Fetch camera parameters
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  pipeline_ = initialize_pipeline();

  for (const auto &name : camera_name) {
    std::string camera_path;
    this->declare_parameter(name + ".path", "");
    this->get_parameter(name + ".path", camera_path);
    this->declare_parameter(name + ".type",
                            static_cast<int>(CameraType::V4l2Src));
    int camera_type;
    this->get_parameter(name + ".type", camera_type);
    CameraSource source;
    source.name = name;
    source.path = camera_path;
    source.type = static_cast<CameraType>(camera_type);
    if (source.type != CameraType::TestSrc && camera_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Camera path not set for %s",
                   name.c_str());
      continue;
    }
    if (source.type != CameraType::TestSrc &&
        !std::filesystem::exists(source.path)) {
      RCLCPP_ERROR(this->get_logger(), "Camera path does not exist: %s",
                   camera_path.c_str());
      continue;
    }
    create_source(source);
  }
  const auto ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL,
                            "start_pipeline");
}

WebRTCStreamer::~WebRTCStreamer() {
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
}

void WebRTCStreamer::start_video_cb(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  if (pipeline_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (update_pipeline(request) != true) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update pipeline");
    response->success = false;
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL,
                              "error_pipeline");
    return;
  }
  const auto ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
    response->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    response->success = false;
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL,
                            "pipeline");
}

GstElement *WebRTCStreamer::create_vid_conv() {
  GstElement *videoconvert = gst_element_factory_make("nvvidconv", nullptr);
  if (videoconvert) {
    return videoconvert;
  }
  RCLCPP_INFO(this->get_logger(),
              "Failed to create nvvidconv, using videoconvert instead");
  return gst_element_factory_make("videoconvert", nullptr);
}
GstElement *WebRTCStreamer::create_source(const CameraSource &src) {
  GstElement *src_element = nullptr;
  const std::string &name = src.name;
  if (src.type == CameraType::TestSrc) {
    src_element = gst_element_factory_make("videotestsrc", nullptr);
    g_object_set(G_OBJECT(src_element), "pattern", 0, nullptr);
    g_object_set(G_OBJECT(src_element), "is-live", TRUE, nullptr);
  } else if (src.type == CameraType::V4l2Src) {
    src_element = gst_element_factory_make("v4l2src", nullptr);
    g_object_set(G_OBJECT(src_element), "device", src.path.c_str(), nullptr);
  } else {
    // TODO: Add support for network source for science cameras
    RCLCPP_WARN(this->get_logger(), "Unimplemented Type for camera: %s",
                name.c_str());
    return nullptr;
  }

  GstElement *videoconvert = create_vid_conv();
  GstElement *tee = gst_element_factory_make("tee", name.c_str());
  if (!src_element || !videoconvert || !tee) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create source for camera: %s",
                 name.c_str());
    return nullptr;
  }
  g_object_set(G_OBJECT(tee), "allow-not-linked", TRUE, nullptr);
  gst_bin_add_many(GST_BIN(pipeline_), src_element, videoconvert, tee, nullptr);
  gst_element_sync_state_with_parent(src_element);
  gst_element_sync_state_with_parent(tee);
  gst_element_sync_state_with_parent(videoconvert);
  if (!gst_element_link_many(src_element, videoconvert, tee, nullptr)) {
    RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                 __FUNCTION__);
    return nullptr;
  }
  return tee;
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  GstElement *pipeline = gst_pipeline_new("webrtc-pipeline");

  GstElement *stable_source = gst_element_factory_make("videotestsrc", nullptr);

  GstElement *videoconvert_in = create_vid_conv();

  compositor_ = gst_element_factory_make("nvcompositor", nullptr);
  if (!compositor_) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvcompositor, using compositor instead");
    compositor_ = gst_element_factory_make("compositor", nullptr);
  }

  GstElement *queue = gst_element_factory_make("queue", nullptr);

  GstElement *videoconvert_out = create_vid_conv();
  GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
  GstElement *webrtcsink = gst_element_factory_make("webrtcsink", "webrtcsink");

  if (!stable_source || !videoconvert_in || !compositor_ || !queue ||
      !videoconvert_out || !capsfilter || !webrtcsink) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create webrtc elements");
    return nullptr;
  }

  // Set properties for the elements
  g_object_set(G_OBJECT(stable_source), "pattern", 2, nullptr);
  g_object_set(G_OBJECT(stable_source), "is-live", TRUE, nullptr);
  // g_object_set(G_OBJECT(compositor_), "ignore-inactive-pads", TRUE, nullptr);
  g_object_set(G_OBJECT(queue), "max-size-buffers", 1, nullptr);
  GstCaps *caps = gst_caps_from_string("video/x-raw");
  g_object_set(G_OBJECT(capsfilter), "caps", caps, nullptr);
  gst_caps_unref(caps);
  g_object_set(G_OBJECT(webrtcsink), "run-signalling-server", TRUE, nullptr);
  if (web_server_) {
    g_object_set(G_OBJECT(webrtcsink), "run-web-server", TRUE, nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-host-addr",
                 "http://0.0.0.0:8080/", nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-directory",
                 web_server_path_.c_str(), nullptr);
  }

  // Add elements to the pipeline
  gst_bin_add_many(GST_BIN(pipeline), stable_source, videoconvert_in,
                   compositor_, queue, videoconvert_out, capsfilter, webrtcsink,
                   nullptr);
  // Final linking for the pipeline
  if (!gst_element_link_many(stable_source, videoconvert_in, compositor_, queue,
                             videoconvert_out, capsfilter, webrtcsink,
                             nullptr)) {
    RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                 __FUNCTION__);
    return nullptr;
  }
  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  if (gst_bus_add_watch(bus, &WebRTCStreamer::on_bus_message, this)) {
    RCLCPP_INFO(this->get_logger(), "Bus watch added successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to add bus watch.");
  }
  gst_object_unref(bus);
  return pipeline;
}
bool WebRTCStreamer::unlink_pad(GstPad *pad) {
  if (!pad) {
    return false;
  }
  GstPad *peer = gst_pad_get_peer(pad);
  if (peer) {
    if (GST_PAD_IS_SRC(peer)) {
      gst_pad_unlink(peer, pad);
    } else {
      gst_pad_unlink(pad, peer);
    }
    gst_element_release_request_pad(
        GST_ELEMENT(gst_pad_get_parent_element(peer)), peer);
    gst_object_unref(peer);
  }
  return true;
}

void WebRTCStreamer::unlink_sources_from_compositor() {
  for (const auto &source : connected_sources_) {
    unlink_pad(gst_element_get_static_pad(source, "src"));
    unlink_pad(gst_element_get_static_pad(source, "sink"));
    gst_bin_remove(GST_BIN(pipeline_), source);
  }
  connected_sources_.clear();
}

bool WebRTCStreamer::update_pipeline(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    return false;
  }
  unlink_sources_from_compositor();
  const auto &total_height = request->height;
  const auto &total_width = request->width;

  int i = 1;
  for (const auto &source : request->sources) {
    const std::string &name = source.name;
    const int height = source.height * total_height / 100;
    const int width = source.width * total_width / 100;
    const int origin_x = source.origin_x * total_width / 100;
    const int origin_y = source.origin_y * total_height / 100;

    GstElement *queue = gst_element_factory_make("queue", nullptr);
    if (!queue) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create queue");
      return false;
    }
    gst_bin_add(GST_BIN(pipeline_), queue);
    gst_element_sync_state_with_parent(queue);
    GstElement *source_tee =
        gst_bin_get_by_name(GST_BIN(pipeline_), name.c_str());
    if (!source_tee) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get source: %s",
                   name.c_str());
      return false;
    }
    if (gst_element_link(source_tee, queue) != TRUE) {
      RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                   __FUNCTION__);
      return false;
    }
    GstPad *pad = gst_element_request_pad_simple(compositor_, "sink_%u");
    if (!pad) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get pad");
      return false;
    }
    if (gst_pad_link(gst_element_get_static_pad(queue, "src"), pad) !=
        GST_PAD_LINK_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to link pads");
      return false;
    }
    connected_sources_.push_back(queue);
    g_object_set(G_OBJECT(pad), "xpos", origin_x, "ypos", origin_y, "height",
                 height, "width", width, NULL);
    gst_object_unref(pad);
    ++i;
  }
  return true;
}

gboolean WebRTCStreamer::on_bus_message(GstBus *bus, GstMessage *message,
                                        gpointer user_data) {
  WebRTCStreamer *streamer = static_cast<WebRTCStreamer *>(user_data);
  RCLCPP_INFO(streamer->get_logger(), "Received bus message: %s",
              GST_MESSAGE_TYPE_NAME(message));

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR:
    case GST_MESSAGE_WARNING:
      gchar *debug;
      GError *err;
      gst_message_parse_error(message, &err, &debug);
      RCLCPP_ERROR(streamer->get_logger(), "GStreamer Error: %s", err->message);
      g_error_free(err);
      g_free(debug);
      break;
    case GST_MESSAGE_STATE_CHANGED: {
      GstState old_state, new_state, pending_state;
      gst_message_parse_state_changed(message, &old_state, &new_state,
                                      &pending_state);
      RCLCPP_INFO(streamer->get_logger(), "State change: %s -> %s",
                  gst_element_state_get_name(old_state),
                  gst_element_state_get_name(new_state));
    } break;
    default:
      break;
  }
  return TRUE;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<WebRTCStreamer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
