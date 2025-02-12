#ifndef TELEMETRY_COLLECTOR_HPP_
#define TELEMETRY_COLLECTOR_HPP_

#include "interfaces/msg/system_telemetry.hpp"

class TelemetryCollector {
 public:
  virtual void collect(interfaces::msg::SystemTelemetry& msg) = 0;
  virtual ~TelemetryCollector() = default;
};

#endif  // TELEMETRY_COLLECTOR_HPP_
