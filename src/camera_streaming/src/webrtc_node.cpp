#include "webrtc_node.h"

#include <gst/app/gstappsink.h>

#include <filesystem>
#include <fstream>

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
  capture_service_ = this->create_service<interfaces::srv::VideoCapture>(
      "capture_frame", std::bind(&WebRTCStreamer::capture_frame, this,
                                 std::placeholders::_1, std::placeholders::_2));

  // Fetch camera parameters
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  initialize_pipeline();

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
    if (source.type == CameraType::V4l2Src &&
        !std::filesystem::exists(source.path)) {
      RCLCPP_ERROR(this->get_logger(), "Camera path does not exist: %s",
                   camera_path.c_str());
      continue;
    }
    create_source(source);
  }
  const auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
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
  const auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
    response->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    response->success = false;
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
                            "pipeline");
}

void WebRTCStreamer::capture_frame(
    const std::shared_ptr<interfaces::srv::VideoCapture::Request> request,
    std::shared_ptr<interfaces::srv::VideoCapture::Response> response) {
  response->success = true;
  const std::string &name = request->source;
  auto source_tee = GstUniquePtr<GstElement>(
      gst_bin_get_by_name(GST_BIN(pipeline_.get()), name.c_str()));
  if (!source_tee) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get source: %s", name.c_str());
    return;
  }
  std::vector<GstElement *> elements;
  elements.emplace_back(create_vid_conv());
  elements.emplace_back(create_jpeg_enc());

  GstElement *sink = create_element("appsink");
  elements.push_back(sink);
  if (!add_element_chain(elements)) {
    response->success = false;
    return;
  }

  if (!gst_element_link(source_tee.get(), elements.front())) {
    RCLCPP_ERROR(this->get_logger(),
                 "%s: Failed to link capture pipe section to %s", __FUNCTION__,
                 name.c_str());
    response->success = false;
    for (auto element : elements) {
      gst_bin_remove(GST_BIN(pipeline_.get()), element);
    }
    return;
  }
  constexpr auto timeout = 1000000000;  // 1 second
  GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), timeout);

  unlink_pad(gst_element_get_static_pad(elements.front(), "sink"));
  for (auto element : elements) {
    gst_bin_remove(GST_BIN(pipeline_.get()), element);
  }

  if (!sample) {
    RCLCPP_ERROR(this->get_logger(), "Failed to pull sample from appsink");
    response->success = false;
    return;
  }
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get buffer from sample");
    gst_sample_unref(sample);
    response->success = false;
    return;
  }
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to map buffer");
    gst_sample_unref(sample);
    response->success = false;
    return;
  }
  std::vector<uint8_t> &img_vec = response->image.data;
  std::string &filename = request->filename;
  img_vec.resize(map.size);
  std::memcpy(img_vec.data(), map.data, map.size);
  if (!filename.empty()) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing",
                   filename.c_str());
      response->success = false;
    } else {
      file.write(reinterpret_cast<const char *>(map.data), map.size);
      if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to file %s",
                     filename.c_str());
        response->success = false;
      }
      file.close();
    }
  }
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
}

GstElement *WebRTCStreamer::create_vid_conv() {
  GstElement *videoconvert = create_element("nvvidconv");
  if (videoconvert) {
    return videoconvert;
  }
  RCLCPP_INFO(this->get_logger(),
              "Failed to create nvvidconv, using videoconvert instead");
  return create_element("videoconvert");
}
GstElement *WebRTCStreamer::create_jpeg_enc() {
  GstElement *encoder = create_element("nvjpegenc");
  if (encoder) {
    return encoder;
  }
  RCLCPP_INFO(this->get_logger(),
              "Failed to create nvjpegenc, using jpegenc instead");
  return create_element("jpegenc");
}
GstElement *WebRTCStreamer::create_element(std::string element_type,
                                           std::string element_name) {
  GstElement *element = nullptr;
  if (element_name.empty()) {
    element = gst_element_factory_make(element_type.c_str(), nullptr);
  } else {
    element =
        gst_element_factory_make(element_type.c_str(), element_name.c_str());
  }
  if (!element) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create %s",
                 element_type.c_str());
  }
  return element;
}

GstElement *WebRTCStreamer::add_element_chain(
    const std::vector<GstElement *> &chain) {
  GstElement *lastElement = nullptr;
  bool success = true;
  for (auto element : chain) {
    if (!element) {
      RCLCPP_ERROR(this->get_logger(), "%s: Bad chain", __FUNCTION__);
      success = false;
      break;
    }
    gst_bin_add(GST_BIN(pipeline_.get()), element);
    gst_element_sync_state_with_parent(element);
    if (!lastElement) {
      lastElement = element;
      continue;
    }
    if (!gst_element_link(lastElement, element)) {
      char *name1 = gst_element_get_name(lastElement);
      char *name2 = gst_element_get_name(element);
      RCLCPP_ERROR(this->get_logger(), "Could not link %s with %s", name1,
                   name2);
      g_free(name1);
      g_free(name2);
      success = false;
      break;
    }
    lastElement = element;
  }
  if (success) {
    return lastElement;
  }
  for (auto element : chain) {
    if (element) {
      gst_bin_remove(GST_BIN(pipeline_.get()), element);
    }
    if (element == lastElement) {
      break;
    }
  }
  return nullptr;
}
GstElement *WebRTCStreamer::create_source(const CameraSource &src) {
  std::vector<GstElement *> elements;

  const std::string &name = src.name;
  const CameraType &type = src.type;
  const std::map<CameraType, std::string> source_map = {
      {CameraType::TestSrc, "videotestsrc"},
      {CameraType::V4l2Src, "v4l2src"},
      {CameraType::NetworkSrc, "udpsrc"},
  };

  // Add src element(s)
  const auto iter = source_map.find(type);
  if (iter == source_map.end()) {
    RCLCPP_WARN(this->get_logger(), "Unimplemented Type for camera: %s",
                name.c_str());
    return nullptr;
  }
  GstElement *source_element = create_element(iter->second);
  if (!source_element) {
    return nullptr;
  }
  elements.push_back(source_element);
  if (src.type == CameraType::TestSrc) {
    g_object_set(G_OBJECT(source_element), "pattern", 0, nullptr);
    g_object_set(G_OBJECT(source_element), "is-live", TRUE, nullptr);
  } else if (src.type == CameraType::V4l2Src) {
    g_object_set(G_OBJECT(source_element), "device", src.path.c_str(), nullptr);
  } else if (src.type == CameraType::NetworkSrc) {
    g_object_set(G_OBJECT(source_element), "uri", src.path.c_str(), nullptr);
    auto caps = GstUniquePtr<GstCaps>(gst_caps_from_string(
        "application/x-rtp, media=video, encoding-name=JPEG, payload=26, "
        "clock-rate=90000"));
    g_object_set(G_OBJECT(source_element), "caps", caps.get(), nullptr);
    elements.push_back(create_element("rtpjpegdepay"));
  }
  // Add jpeg decoders (if necessary)
  if (src.encoded) {
    elements.emplace_back(create_element("jpegparse"));
    elements.emplace_back(create_element("nvjpegdec"));
  }

  // Add small queue that drops oldest buffers
  elements.emplace_back(create_element("queue"));
  if (!elements.back()) {
    return nullptr;
  }
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  // Add video converter
  elements.emplace_back(create_vid_conv());

  // Add tee to connect to
  elements.emplace_back(create_element("tee", name));
  if (!elements.back()) {
    return nullptr;
  }
  g_object_set(G_OBJECT(elements.back()), "allow-not-linked", TRUE, nullptr);
  return add_element_chain(elements);
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  pipeline_ = GstUniquePtr<GstElement>(gst_pipeline_new("webrtc-pipeline"));
  std::vector<GstElement *> elements;

  elements.emplace_back(create_element("videotestsrc"));
  assert(elements.back() != nullptr);
  g_object_set(G_OBJECT(elements.back()), "pattern", 2, nullptr);
  g_object_set(G_OBJECT(elements.back()), "is-live", TRUE, nullptr);

  elements.emplace_back(create_vid_conv());

  compositor_ = create_element("nvcompositor");
  if (!compositor_) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvcompositor, using compositor instead");
    compositor_ = create_element("compositor");
  }
  elements.push_back(compositor_);

  elements.push_back(create_element("queue"));
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  elements.emplace_back(create_vid_conv());

  elements.emplace_back(create_element("webrtcsink", "webrtcsink"));
  GstElement *webrtcsink = elements.back();
  assert(webrtcsink != nullptr);
  g_object_set(G_OBJECT(webrtcsink), "run-signalling-server", TRUE, nullptr);
  if (web_server_) {
    g_object_set(G_OBJECT(webrtcsink), "run-web-server", TRUE, nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-host-addr",
                 "http://0.0.0.0:8080/", nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-directory",
                 web_server_path_.c_str(), nullptr);
  }

  if (!add_element_chain(elements)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add elements to pipeline");
    return nullptr;
  }

  bus_ =
      GstUniquePtr<GstBus>(gst_pipeline_get_bus(GST_PIPELINE(pipeline_.get())));
  if (gst_bus_add_watch(bus_.get(), &WebRTCStreamer::on_bus_message, this)) {
    RCLCPP_INFO(this->get_logger(), "Bus watch added successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to add bus watch.");
  }
  return pipeline_.get();
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
    unlink_pad(gst_element_get_static_pad(source, "src"));
    unlink_pad(gst_element_get_static_pad(source, "sink"));
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

    GstElement *queue = create_element("queue");
    if (!queue) {
      return false;
    }
    g_object_set(G_OBJECT(queue), "max-size-buffers", 1, nullptr);
    g_object_set(G_OBJECT(queue), "leaky", 2, nullptr);
    gst_bin_add(GST_BIN(pipeline_.get()), queue);
    gst_element_sync_state_with_parent(queue);
    auto source_tee = GstUniquePtr<GstElement>(
        gst_bin_get_by_name(GST_BIN(pipeline_.get()), name.c_str()));
    if (!source_tee) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get source: %s",
                   name.c_str());
      return false;
    }
    if (gst_element_link(source_tee.get(), queue) != TRUE) {
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
    if (gst_pad_link(gst_element_get_static_pad(queue, "src"), pad.get()) !=
        GST_PAD_LINK_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to link pads");
      return false;
    }
    connected_sources_.push_back(queue);
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebRTCStreamer>());
  rclcpp::shutdown();
  return 0;
}