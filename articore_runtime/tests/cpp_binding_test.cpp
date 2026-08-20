#include <type_traits>
#include "articore/runtime.hpp"

int main() {
  static_assert(!std::is_copy_constructible_v<articore::Runtime>);
  static_assert(!std::is_copy_assignable_v<articore::Runtime>);
  static_assert(std::is_move_constructible_v<articore::Runtime>);
  static_assert(std::is_constructible_v<
                articore::Runtime, ArticoreControlMode, bool>);

  // This target is a cross-platform compile/link smoke test for the public
  // RAII wrapper. Runtime behavior and invalid construction are exercised by
  // runtime_test through the C ABI without fabricated native handles.
  return 0;
}
