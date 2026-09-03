#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>

namespace articore::detail {

template <std::size_t N>
inline void copy_text(char (&destination)[N], const std::string& source) {
  std::memset(destination, 0, N);
  std::memcpy(destination, source.data(), std::min(source.size(), N - 1));
}

inline uint64_t age_ns(std::chrono::steady_clock::time_point value,
                       bool available,
                       std::chrono::steady_clock::time_point now) {
  if (!available) return std::numeric_limits<uint64_t>::max();
  const auto age =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - value);
  return static_cast<uint64_t>(std::max<int64_t>(0, age.count()));
}

// Execute at most one job per active CAN side using the caller plus at most
// one helper thread. In the deployed service both inherit the non-RT CPU 0..4
// housekeeping mask, so lifecycle and maintenance operations never create an
// unbounded per-Motor thread fan-out.
template <typename ActiveSides, typename Function>
inline void run_active_sides(const ActiveSides& active_sides,
                             Function&& function) {
  std::exception_ptr helper_error;
  std::thread helper;
  if (active_sides[0] && active_sides[1]) {
    helper = std::thread([&] {
      try {
        function(static_cast<uint8_t>(1));
      } catch (...) {
        helper_error = std::current_exception();
      }
    });
  }

  std::exception_ptr caller_error;
  try {
    if (active_sides[0]) {
      function(static_cast<uint8_t>(0));
    } else if (active_sides[1]) {
      function(static_cast<uint8_t>(1));
    }
  } catch (...) {
    caller_error = std::current_exception();
  }

  if (helper.joinable()) helper.join();
  if (caller_error) std::rethrow_exception(caller_error);
  if (helper_error) std::rethrow_exception(helper_error);
}

}  // namespace articore::detail
