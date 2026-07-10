#pragma once

#include <cstdint>
#include <optional>

namespace damiao {

enum class RegisterAccess { ReadOnly, ReadWrite };
enum class RegisterDataType { Float, UInt32 };

struct RegisterInfo {
  uint8_t rid;
  RegisterAccess access;
  RegisterDataType data_type;
};

inline std::optional<RegisterInfo> register_info(uint8_t rid) {
  if (rid <= 6) return RegisterInfo{rid, RegisterAccess::ReadWrite, RegisterDataType::Float};
  if (rid <= 10) return RegisterInfo{rid, RegisterAccess::ReadWrite, RegisterDataType::UInt32};
  if (rid == 11 || rid == 12) {
    return RegisterInfo{rid, RegisterAccess::ReadOnly, RegisterDataType::Float};
  }
  if (rid >= 13 && rid <= 16) {
    return RegisterInfo{rid, RegisterAccess::ReadOnly, RegisterDataType::UInt32};
  }
  if (rid >= 17 && rid <= 20) {
    return RegisterInfo{rid, RegisterAccess::ReadOnly, RegisterDataType::Float};
  }
  if (rid >= 21 && rid <= 34) {
    return RegisterInfo{rid, RegisterAccess::ReadWrite, RegisterDataType::Float};
  }
  if (rid == 35) return RegisterInfo{rid, RegisterAccess::ReadWrite, RegisterDataType::UInt32};
  if (rid == 36) return RegisterInfo{rid, RegisterAccess::ReadOnly, RegisterDataType::UInt32};
  if ((rid >= 50 && rid <= 56) || (rid >= 59 && rid <= 65) || rid == 80 || rid == 81) {
    return RegisterInfo{rid, RegisterAccess::ReadOnly, RegisterDataType::Float};
  }
  return std::nullopt;
}

}  // namespace damiao
