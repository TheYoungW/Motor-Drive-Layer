#include <type_traits>

#include <articore/runtime.hpp>

int main() {
  static_assert(!std::is_copy_constructible_v<articore::Runtime>);
  const auto api = articore::detail::motor_api();
  return api.group_send_mit && api.controller_request_feedback_all_ex ? 0 : 1;
}
