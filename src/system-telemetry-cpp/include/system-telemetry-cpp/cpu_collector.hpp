#ifndef CPU_COLLECTOR_HPP_
#define CPU_COLLECTOR_HPP_

#include "telemetry_collector.hpp"

class CPUCollector : public TelemetryCollector {
 public:
  CPUCollector();
  void collect(interfaces::msg::SystemTelemetry& msg) override;

 private:
  struct CPUStats {
    unsigned long long total;
    unsigned long long idle;
  };

  CPUStats read_cpu_stats();
  double calculate_cpu_usage(const CPUStats& prev, const CPUStats& curr);
  CPUStats prev_stats_;
};

#endif  // CPU_COLLECTOR_HPP_
