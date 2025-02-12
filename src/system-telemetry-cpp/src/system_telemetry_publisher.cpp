#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "cpu_collector.hpp"
#include "gpu_collector.hpp"
#include "interfaces/msg/system_telemetry.hpp"
#include "memory_collector.hpp"
#include "telemetry_collector.hpp"

using namespace std::chrono_literals;

class SystemTelemetryPublisher : public rclcpp::Node {
 public:
  SystemTelemetryPublisher() : Node("system_telemetry_publisher") {
    publisher_ = this->create_publisher<interfaces::msg::SystemTelemetry>(
        "/system_telemetry", 10);

    collectors_.emplace_back(std::make_unique<CPUCollector>());
    collectors_.emplace_back(std::make_unique<MemoryCollector>());
    collectors_.emplace_back(std::make_unique<GPUCollector>());

    timer_ = this->create_wall_timer(
        1s, std::bind(&SystemTelemetryPublisher::publish_telemetry, this));
    RCLCPP_INFO(this->get_logger(), "System Telemetry Publisher started.");
  }

 private:
  void publish_telemetry() {
    interfaces::msg::SystemTelemetry msg;
    for (const auto& collector : collectors_) {
      collector->collect(msg);
    }
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
                "Published Telemetry: CPU: %.1f%%, Mem: %.1f%%, GPU: %.1f%%",
                msg.cpu_usage, msg.mem_usage, msg.gpu_usage);
  }

  rclcpp::Publisher<interfaces::msg::SystemTelemetry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::unique_ptr<TelemetryCollector>> collectors_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemTelemetryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
