#include "gpu_collector.hpp"

#include <cstdio>
#include <regex>
#include <string>

double GPUCollector::get_gpu_usage() {
  FILE* pipe = popen("tegrastats --oneshot", "r");
  if (!pipe) {
    return 0.0;
  }
  char buffer[256];
  std::string result;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  pclose(pipe);

  std::regex gpu_regex("GPU\\s+(\\d+)%");
  std::smatch match;
  if (std::regex_search(result, match, gpu_regex)) {
    if (match.size() >= 2) {
      return std::stod(match[1].str());
    }
  }
  return 0.0;
}

void GPUCollector::collect(interfaces::msg::SystemTelemetry& msg) {
  msg.gpu_usage = static_cast<float>(get_gpu_usage());
}
