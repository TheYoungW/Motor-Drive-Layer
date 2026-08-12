#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

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

}  // namespace articore::detail
