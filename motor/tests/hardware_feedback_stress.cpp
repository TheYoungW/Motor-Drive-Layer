#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "damiao/runtime.hpp"
#include "damiao/socketcan_fd_bus.hpp"

namespace {

uint64_t parse_u64(const char* raw, const char* name) {
  try {
    std::size_t used = 0;
    const auto value = std::stoull(raw, &used, 10);
    if (raw[used] != '\0') throw std::invalid_argument("trailing characters");
    return value;
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("invalid ") + name + ": " + raw);
  }
}

const char* status_name(damiao::FeedbackBatchStatus status) {
  switch (status) {
    case damiao::FeedbackBatchStatus::Ok: return "ok";
    case damiao::FeedbackBatchStatus::Timeout: return "timeout";
    case damiao::FeedbackBatchStatus::Incomplete: return "incomplete";
    case damiao::FeedbackBatchStatus::TransportError: return "transport_error";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 8) {
    std::cerr << "usage: " << argv[0]
              << " INTERFACE [BATCHES=200] [TIMEOUT_MS=50] [TX_GAP_US=120]"
                 " [BATCH_INTERVAL_MS=2] [MOTOR_ID=0(all)] [BRS=1]\n";
    return 2;
  }

  try {
    const std::string interface = argv[1];
    const uint64_t batches = argc > 2 ? parse_u64(argv[2], "batches") : 200;
    const uint64_t timeout_ms = argc > 3 ? parse_u64(argv[3], "timeout_ms") : 50;
    const uint64_t tx_gap_us = argc > 4 ? parse_u64(argv[4], "tx_gap_us") : 120;
    const uint64_t interval_ms = argc > 5 ? parse_u64(argv[5], "batch_interval_ms") : 2;
    const uint64_t selected_id = argc > 6 ? parse_u64(argv[6], "motor_id") : 0;
    const uint64_t brs = argc > 7 ? parse_u64(argv[7], "brs") : 1;
    if (batches == 0 || timeout_ms == 0 || timeout_ms > 60000 ||
        tx_gap_us > 1000000 || interval_ms > 60000 || selected_id > 8 || brs > 1) {
      throw std::invalid_argument("numeric argument outside the safe diagnostic range");
    }

    auto bus = damiao::SocketCanFdBus::open(interface, brs != 0);
    damiao::Controller controller(bus, interface);
    const char* models[8] = {
        "8009", "8009", "4340P", "4340P",
        "4310", "4310", "4310", "4310"};
    std::vector<std::shared_ptr<damiao::MotorHandle>> motors;
    const uint16_t first_id = selected_id == 0 ? 1 : static_cast<uint16_t>(selected_id);
    const uint16_t last_id = selected_id == 0 ? 8 : static_cast<uint16_t>(selected_id);
    for (uint16_t id = first_id; id <= last_id; ++id) {
      motors.push_back(controller.add_damiao_motor(id, 0x10U | id, models[id - 1]));
    }
    controller.set_tx_gap(std::chrono::microseconds(tx_gap_us));

    std::map<uint16_t, uint64_t> misses;
    uint64_t ok = 0;
    uint64_t incomplete = 0;
    uint64_t timeout = 0;
    uint64_t transport_error = 0;
    double latency_sum_ms = 0.0;
    double latency_max_ms = 0.0;
    std::vector<double> latencies;
    latencies.reserve(batches);

    for (uint64_t batch = 0; batch < batches; ++batch) {
      const auto started = std::chrono::steady_clock::now();
      const auto report = controller.request_feedback_all_report(
          std::chrono::milliseconds(timeout_ms));
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      latencies.push_back(elapsed_ms);
      latency_sum_ms += elapsed_ms;
      latency_max_ms = std::max(latency_max_ms, elapsed_ms);

      switch (report.status) {
        case damiao::FeedbackBatchStatus::Ok: ++ok; break;
        case damiao::FeedbackBatchStatus::Incomplete: ++incomplete; break;
        case damiao::FeedbackBatchStatus::Timeout: ++timeout; break;
        case damiao::FeedbackBatchStatus::TransportError: ++transport_error; break;
      }
      for (const auto id : report.missing_motor_ids) ++misses[id];
      if (report.status != damiao::FeedbackBatchStatus::Ok) {
        std::cout << "failure batch=" << batch + 1
                  << " status=" << status_name(report.status)
                  << " elapsed_ms=" << std::fixed << std::setprecision(3)
                  << elapsed_ms << " received=" << report.received_count
                  << "/" << report.expected_count << " missing=";
        for (const auto id : report.missing_motor_ids) std::cout << id << ',';
        if (!report.error.empty()) std::cout << " error=" << report.error;
        std::cout << '\n';
      }
      if (interval_ms != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
      }
    }

    std::sort(latencies.begin(), latencies.end());
    const auto percentile = [&](double fraction) {
      const auto index = static_cast<std::size_t>(
          fraction * static_cast<double>(latencies.size() - 1));
      return latencies[index];
    };
    const auto health = controller.transport_health();
    std::cout << "summary interface=" << interface
              << " batches=" << batches
              << " timeout_ms=" << timeout_ms
              << " tx_gap_us=" << tx_gap_us
              << " interval_ms=" << interval_ms
              << " motor_id=" << selected_id
              << " brs=" << brs
              << " ok=" << ok
              << " incomplete=" << incomplete
              << " timeout=" << timeout
              << " transport_error=" << transport_error
              << " latency_avg_ms=" << std::fixed << std::setprecision(3)
              << latency_sum_ms / static_cast<double>(batches)
              << " latency_p95_ms=" << percentile(0.95)
              << " latency_p99_ms=" << percentile(0.99)
              << " latency_max_ms=" << latency_max_ms
              << " tx_frames=" << health.tx_frames
              << " rx_frames=" << health.rx_frames
              << " send_errors=" << health.send_errors
              << " receive_errors=" << health.receive_errors
              << " misses=";
    for (uint16_t id = 1; id <= 8; ++id) {
      if (id != 1) std::cout << ',';
      std::cout << id << ':' << misses[id];
    }
    std::cout << '\n';
    controller.close_bus();
    return (incomplete == 0 && timeout == 0 && transport_error == 0) ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << '\n';
    return 2;
  }
}
