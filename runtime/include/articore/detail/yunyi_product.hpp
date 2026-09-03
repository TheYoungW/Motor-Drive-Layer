#pragma once

#include <array>

namespace articore {

inline constexpr std::array<std::array<float, 7>, 2>
    kYunyiJointDirection{{
        {{1, 1, 1, -1, -1, 1, 1}},
        {{-1, 1, 1, 1, -1, -1, 1}},
    }};
inline constexpr std::array<float, 7> kYunyiLogicalTorqueRange{
    {40, 40, 27, 27, 7, 7, 7}};
inline constexpr std::array<float, 7> kYunyiNativeTorqueRange{
    {54, 54, 28, 28, 10, 10, 10}};

}  // namespace articore
