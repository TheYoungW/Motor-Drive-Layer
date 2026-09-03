#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "articore/runtime.hpp"

namespace articore::dds {

struct ServiceConfig {
  std::string robot_id = "yunyi-001";
  std::uint32_t domain_id = 0;
  std::vector<std::string> network_interfaces{"eth0"};
  YunyiRuntimeConfig runtime{};
  std::chrono::milliseconds can_retry_initial{250};
  std::chrono::milliseconds can_retry_max{5000};
};

Result<ServiceConfig> load_service_config(const std::string& path);
std::string cyclone_uri(const ServiceConfig& config);

class CycloneService final {
 public:
  explicit CycloneService(ServiceConfig config);
  ~CycloneService();
  CycloneService(const CycloneService&) = delete;
  CycloneService& operator=(const CycloneService&) = delete;

  Status run(const std::atomic<bool>& stop_requested);
  void request_stop() noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace articore::dds
