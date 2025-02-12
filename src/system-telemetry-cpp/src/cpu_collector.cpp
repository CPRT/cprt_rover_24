#include "cpu_collector.hpp"

#include <fstream>
#include <sstream>
#include <string>

CPUCollector::CPUCollector() : prev_stats_(read_cpu_stats()) {}

CPUCollector::CPUStats CPUCollector::read_cpu_stats() {
  CPUStats stats{0, 0};
  std::ifstream stat_file("/proc/stat");
  std::string line;
  if (std::getline(stat_file, line)) {
    std::istringstream iss(line);
    std::string cpu;
    iss >> cpu;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    stats.idle = idle + iowait;
    stats.total = user + nice + system + idle + iowait + irq + softirq + steal;
  }
  return stats;
}

double CPUCollector::calculate_cpu_usage(const CPUStats& prev,
                                         const CPUStats& curr) {
  unsigned long long total_diff = curr.total - prev.total;
  unsigned long long idle_diff = curr.idle - prev.idle;
  if (total_diff == 0) return 0.0;
  return 100.0 * (total_diff - idle_diff) / static_cast<double>(total_diff);
}

void CPUCollector::collect(interfaces::msg::SystemTelemetry& msg) {
  CPUStats current = read_cpu_stats();
  double usage = calculate_cpu_usage(prev_stats_, current);
  prev_stats_ = current;
  msg.cpu_usage = static_cast<float>(usage);
}
