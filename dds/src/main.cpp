#include "articore/dds/service.hpp"

#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>

namespace {
std::atomic<bool> g_stop{false};
void stop_handler(int) { g_stop.store(true, std::memory_order_relaxed); }
}  // namespace

int main(int argc, char** argv) {
  const std::string config_path =
      argc > 1 ? argv[1] : "/etc/articore/runtime-service.conf";
  auto config = articore::dds::load_service_config(config_path);
  if (!config) {
    std::cerr << "configuration error: " << config.status().message() << '\n';
    return 2;
  }
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  try {
    articore::dds::CycloneService service(std::move(config).value());
    const auto result = service.run(g_stop);
    if (!result) {
      std::cerr << "service error: " << result.message() << '\n';
      return 1;
    }
  } catch (const std::exception& error) {
    // Startup races (for example an interface disappearing after ExecStartPre)
    // must be reported as a normal service failure, not an uncaught SIGABRT
    // and core dump. systemd can then retry through the network readiness gate.
    std::cerr << "service startup error: " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "service startup error: unknown exception\n";
    return 1;
  }
  return 0;
}
