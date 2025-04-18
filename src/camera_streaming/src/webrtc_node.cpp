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
  pipeline_ = GstUniquePtr<GstElement>(initialize_pipeline());

  for (const auto &name : camera_name) {
    std::string camera_path;
    this->declare_parameter(name + ".path", "");
    this->get_parameter(name + ".path", camera_path);
    this->declare_parameter(name + ".type",
                            static_cast<int>(CameraType::V4l2Src));
    int camera_type;
    this->get_parameter(name + ".type", camera_type);
    this->declare_parameter(name + ".encoded", false);
    bool encoded;
    this->get_parameter(name + ".encoded", encoded);
    CameraSource source;
    source.name = name;
    source.path = camera_path;
    source.type = static_cast<CameraType>(camera_type);
    source.encoded = encoded;
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
    source_pipelines_.emplace_back(create_source(source));
  }
  setPipelineState(pipeline_.get(), GST_STATE_PLAYING);
  for (const auto &src : source_pipelines_) {
    setPipelineState(src.get(), GST_STATE_PLAYING);
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
                            "start_pipeline");
}

WebRTCStreamer::~WebRTCStreamer() {
  if (pipeline_) {
    gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  }
}

void WebRTCStreamer::start_video_cb(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_.get(), GST_STATE_PAUSED);
  if (update_pipeline(request) != true) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update pipeline");
    response->success = false;
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()),
                              GST_DEBUG_GRAPH_SHOW_ALL, "error_pipeline");
    return;
  }
  response->success = setPipelineState(pipeline_.get(), GST_STATE_PLAYING);
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
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

  GstElement *pipe = gst_pipeline_new(std::string(name + "_pipe").c_str());

  GstElement *sink = gst_element_factory_make("interpipesink", name.c_str());
  if (!src_element || !sink) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create source for camera: %s",
                 name.c_str());
    return nullptr;
  }
  gst_bin_add_many(GST_BIN(pipe), src_element, sink, nullptr);
  gst_element_sync_state_with_parent(src_element);
  gst_element_sync_state_with_parent(sink);
  gboolean ret;
  if (src.encoded) {
    GstElement *parser = gst_element_factory_make("jpegparse", nullptr);
    if (!parser) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create jpegparse");
      return nullptr;
    }
    GstElement *decoder = gst_element_factory_make("nvv4l2decoder", nullptr);
    if (!decoder) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create nvv4l2decoder");
      return nullptr;
    }
    gst_bin_add_many(GST_BIN(pipe), parser, decoder, nullptr);
    gst_element_sync_state_with_parent(parser);
    gst_element_sync_state_with_parent(decoder);
    ret = gst_element_link_many(src_element, parser, decoder, sink, nullptr);
  } else {
    ret = gst_element_link_many(src_element, sink, nullptr);
  }

  if (!ret) {
    RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                 __FUNCTION__);
    return nullptr;
  }
  return pipe;
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  GstElement *pipeline = gst_pipeline_new("webrtc-out-pipeline");

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
  g_object_set(G_OBJECT(queue), "max-size-buffers", 1, nullptr);
  auto caps = GstUniquePtr<GstCaps>(gst_caps_from_string("video/x-raw"));
  g_object_set(G_OBJECT(capsfilter), "caps", caps.get(), nullptr);
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
  auto bus = GstUniquePtr<GstBus>(gst_pipeline_get_bus(GST_PIPELINE(pipeline)));
  if (gst_bus_add_watch(bus.get(), &WebRTCStreamer::on_bus_message, this)) {
    RCLCPP_INFO(this->get_logger(), "Bus watch added successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to add bus watch.");
  }
  return pipeline;
}
bool WebRTCStreamer::unlink_pad(GstPad *pad) {
  if (!pad) {
    return false;
  }
  auto peer = GstUniquePtr<GstPad>(gst_pad_get_peer(pad));
  if (peer) {
    if (GST_PAD_IS_SRC(peer.get())) {
      gst_pad_unlink(peer.get(), pad);
    } else {
      gst_pad_unlink(pad, peer.get());
    }
    gst_element_release_request_pad(
        GST_ELEMENT(gst_pad_get_parent_element(peer.get())), peer.get());
  }
  return true;
}

void WebRTCStreamer::unlink_sources_from_compositor() {
  for (const auto &source : connected_sources_) {
    const auto srcPad =
        GstUniquePtr<GstPad>(gst_element_get_static_pad(source, "src"));
    if (srcPad) {
      unlink_pad(srcPad.get());
    }
    const auto sinkPad =
        GstUniquePtr<GstPad>(gst_element_get_static_pad(source, "sink"));
    if (sinkPad) {
      unlink_pad(sinkPad.get());
    }
    gst_bin_remove(GST_BIN(pipeline_.get()), source);
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

    GstElement *src = gst_element_factory_make("interpipesrc", nullptr);
    if (!src) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create interpipesrc");
      return false;
    }
    g_object_set(G_OBJECT(src), "accept-eos-event", FALSE, nullptr);
    g_object_set(G_OBJECT(src), "listen-to", name.c_str(), nullptr);
    g_object_set(G_OBJECT(src), "format", GST_FORMAT_TIME, nullptr);
    g_object_set(G_OBJECT(src), "stream-sync", 2, nullptr);
    g_object_set(G_OBJECT(src), "is-live", TRUE, nullptr);

    GstElement *queue = gst_element_factory_make("queue", nullptr);
    if (!queue) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create queue");
      return false;
    }
    GstElement *videoconvert = create_vid_conv();
    if (!videoconvert) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create video converter");
      return false;
    }
    gst_bin_add_many(GST_BIN(pipeline_.get()), src, queue, videoconvert,
                     nullptr);
    gst_element_sync_state_with_parent(src);
    gst_element_sync_state_with_parent(queue);
    gst_element_sync_state_with_parent(videoconvert);

    if (!gst_element_link_many(src, queue, videoconvert, nullptr)) {
      RCLCPP_ERROR(this->get_logger(), "%s: Failed to link elements",
                   __FUNCTION__);
      return false;
    }
    auto pad = GstUniquePtr<GstPad>(
        gst_element_request_pad_simple(compositor_, "sink_%u"));
    if (!pad) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get pad");
      return false;
    }
    if (gst_pad_link(gst_element_get_static_pad(videoconvert, "src"),
                     pad.get()) != GST_PAD_LINK_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to link pads");
      return false;
    }
    connected_sources_.push_back(queue);
    connected_sources_.push_back(src);
    connected_sources_.push_back(videoconvert);
    g_object_set(G_OBJECT(pad.get()), "xpos", origin_x, "ypos", origin_y,
                 "height", height, "width", width, NULL);
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

bool WebRTCStreamer::setPipelineState(GstElement *pipe, GstState state) {
  if (!pipe) {
    RCLCPP_ERROR(this->get_logger(), "%s: Pipeline not initialized",
                 __FUNCTION__);
    return false;
  }
  std::string name;
  char *name_ = gst_element_get_name(pipe);
  if (!name_) {
    name = "Unknown";
  }
  name = name_;
  g_free(name_);

  const auto ret = gst_element_set_state(pipe, state);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline %s state changed successfully",
                name.c_str());
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Pipeline %s: Failed to set pipeline state", name.c_str());
    return false;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebRTCStreamer>());
  rclcpp::shutdown();
  return 0;
}