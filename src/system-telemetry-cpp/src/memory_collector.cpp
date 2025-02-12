#include "memory_collector.hpp"

#include <fstream>
#include <sstream>
#include <string>

void MemoryCollector::collect(interfaces::msg::SystemTelemetry& msg) {
  std::ifstream mem_file("/proc/meminfo");
  std::string line;
  double mem_total = 0.0, mem_free = 0.0, buffers = 0.0, cached = 0.0;
  while (std::getline(mem_file, line)) {
    std::istringstream iss(line);
    std::string key;
    double value;
    std::string unit;
    iss >> key >> value >> unit;
    if (key == "MemTotal:") {
      mem_total = value;
    } else if (key == "MemFree:") {
      mem_free = value;
    } else if (key == "Buffers:") {
      buffers = value;
    } else if (key == "Cached:") {
      cached = value;
    }
  }
  if (mem_total == 0) {
    msg.mem_usage = 0.0;
    return;
  }
  double used = mem_total - mem_free - buffers - cached;
  msg.mem_usage = static_cast<float>(100.0 * used / mem_total);
}
