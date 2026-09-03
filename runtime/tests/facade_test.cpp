#include <iostream>
#include <stdexcept>

#include "articore/runtime.hpp"

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main() {
  articore::YunyiRuntimeConfig config;
  config.left_can_interface = "same-can";
  config.right_can_interface = "same-can";
  auto invalid = articore::YunyiRuntime::create(config);
  require(!invalid, "facade rejects one netdev assigned to both arms");
  require(invalid.status().code() == articore::RuntimeErrorCode::InvalidArgument,
          "facade returns stable invalid-argument code");

  articore::Status ok = articore::Status::success();
  require(static_cast<bool>(ok), "success Status is truthy");
  articore::Result<int> value(42);
  require(value && value.value() == 42, "Result carries typed values");
  std::cout << "runtime facade tests passed\n";
}
