#include "gripper_product_profiles.hpp"

#include <cstring>

namespace articore {
namespace {

constexpr GripperProductProfile kYunyiGripperV1{
    "yunyi_gripper_v1",
    2.64f,
    0.0f,
    5.0f,
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
        {0.5f, 1.0f, 3.0f, 0.3f, 1.0f, 0.2f},
        {0.575f, 1.125f, 3.25f, 0.35f, 1.25f, 0.275f},
        {0.65f, 1.25f, 3.5f, 0.4f, 1.5f, 0.35f},
        {0.725f, 1.375f, 3.75f, 0.45f, 1.75f, 0.425f},
        {0.8f, 1.5f, 4.0f, 0.5f, 2.0f, 0.5f},
        {0.88f, 1.6f, 4.4f, 0.56f, 2.2f, 0.54f},
        {0.96f, 1.7f, 4.8f, 0.62f, 2.4f, 0.58f},
        {1.04f, 1.8f, 5.2f, 0.68f, 2.6f, 0.62f},
        {1.12f, 1.9f, 5.6f, 0.74f, 2.8f, 0.66f},
        {1.2f, 2.0f, 6.0f, 0.8f, 3.0f, 0.7f},
    }}};

}  // namespace

const GripperProductProfile* find_builtin_gripper_product_profile(
    const char* profile_id) {
  if (!profile_id) return nullptr;
  return std::strcmp(profile_id, kYunyiGripperV1.id) == 0
      ? &kYunyiGripperV1 : nullptr;
}

}  // namespace articore
