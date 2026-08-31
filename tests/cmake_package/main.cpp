#include <type_traits>

#include <articore/runtime.hpp>

int main() {
  static_assert(!std::is_copy_constructible_v<articore::Runtime>);
  static_assert(std::is_move_constructible_v<articore::Runtime>);
  return articore_runtime_abi_version() == 0x000D0000U ? 0 : 1;
}
