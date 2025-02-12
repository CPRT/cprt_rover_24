#ifndef MEMORY_COLLECTOR_HPP_
#define MEMORY_COLLECTOR_HPP_

#include "telemetry_collector.hpp"

class MemoryCollector : public TelemetryCollector {
 public:
  void collect(interfaces::msg::SystemTelemetry& msg) override;
};

#endif  // MEMORY_COLLECTOR_HPP_
