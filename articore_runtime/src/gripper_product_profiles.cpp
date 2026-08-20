#include "gripper_product_profiles.hpp"

#include <cstring>

namespace articore {
namespace {

constexpr GripperProductProfile kYunyiGripperV1{
    "yunyi_gripper_v1",
    2.64f,
    0.0f,
    10.0f,
    200U,
    0.01f,
    0.05f,
    200U,
    50U,
    0.08f,
    0.15f,
    100U,
    50U,
    ARTICORE_GRIPPER_FAULT_HOLD,
    {{
        {1.0f, 2.0f, 6.0f, 0.3f, 2.0f, 0.2f},
        {1.15f, 2.25f, 6.5f, 0.35f, 2.5f, 0.275f},
        {1.3f, 2.5f, 7.0f, 0.4f, 3.0f, 0.35f},
        {1.45f, 2.75f, 7.5f, 0.45f, 3.5f, 0.425f},
        {1.6f, 3.0f, 8.0f, 0.5f, 4.0f, 0.5f},
        {1.76f, 3.2f, 8.8f, 0.56f, 4.4f, 0.54f},
        {1.92f, 3.4f, 9.6f, 0.62f, 4.8f, 0.58f},
        {2.08f, 3.6f, 10.4f, 0.68f, 5.2f, 0.62f},
        {2.24f, 3.8f, 11.2f, 0.74f, 5.6f, 0.66f},
        {2.4f, 4.0f, 12.0f, 0.8f, 6.0f, 0.7f},
    }}};

}  // namespace

const GripperProductProfile* find_builtin_gripper_product_profile(
    const char* profile_id) {
  if (!profile_id) return nullptr;
  return std::strcmp(profile_id, kYunyiGripperV1.id) == 0
      ? &kYunyiGripperV1 : nullptr;
}

}  // namespace articore
