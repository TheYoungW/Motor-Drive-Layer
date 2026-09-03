#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "articore/detail/native_types.hpp"

namespace articore::cartesian {

inline void require_pv_mode(ArticoreControlMode mode) {
  if (mode != ARTICORE_MODE_PV) {
    throw std::runtime_error("product Cartesian motion requires PV mode");
  }
}

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

using Vector3 = std::array<double, 3>;

inline Vector3 subtract(const double* lhs, const double* rhs) {
  return {lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]};
}

inline Vector3 cross(const Vector3& lhs, const Vector3& rhs) {
  return {
      lhs[1] * rhs[2] - lhs[2] * rhs[1],
      lhs[2] * rhs[0] - lhs[0] * rhs[2],
      lhs[0] * rhs[1] - lhs[1] * rhs[0],
  };
}

inline double dot(const Vector3& lhs, const Vector3& rhs) {
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

inline double norm(const Vector3& value) {
  return std::sqrt(dot(value, value));
}

inline Vector3 scale(const Vector3& value, double factor) {
  return {value[0] * factor, value[1] * factor, value[2] * factor};
}

inline Vector3 add(const Vector3& lhs, const Vector3& rhs) {
  return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}

struct CircularArc {
  Vector3 center{};
  Vector3 radial_axis{};
  Vector3 tangent_axis{};
  Vector3 normal{};
  double radius = 0.0;
  double via_angle = 0.0;
  double end_angle = 0.0;

  Vector3 position(double angle) const {
    return add(
        center,
        scale(add(scale(radial_axis, std::cos(angle)),
                  scale(tangent_axis, std::sin(angle))), radius));
  }
};

inline CircularArc circular_arc_from_three_points(
    const double* start, const double* via, const double* end) {
  const Vector3 start_value{start[0], start[1], start[2]};
  const auto start_to_via = subtract(via, start);
  const auto start_to_end = subtract(end, start);
  const double via_distance = norm(start_to_via);
  const double end_distance = norm(start_to_end);
  if (!std::isfinite(via_distance) || !std::isfinite(end_distance) ||
      via_distance <= 1e-6 || end_distance <= 1e-6 ||
      norm(subtract(end, via)) <= 1e-6) {
    throw std::invalid_argument(
        "circular start, via and end positions must be distinct");
  }
  const auto plane = cross(start_to_via, start_to_end);
  const double plane_norm = norm(plane);
  if (!std::isfinite(plane_norm) ||
      plane_norm <= 1e-8 * via_distance * end_distance) {
    throw std::invalid_argument(
        "circular start, via and end positions must not be collinear");
  }
  const double plane_norm_squared = dot(plane, plane);
  const auto center_offset = scale(
      add(scale(cross(start_to_end, plane),
                dot(start_to_via, start_to_via)),
          scale(cross(plane, start_to_via),
                dot(start_to_end, start_to_end))),
      1.0 / (2.0 * plane_norm_squared));

  CircularArc result;
  result.center = add(start_value, center_offset);
  const auto start_radius = subtract(start, result.center.data());
  result.radius = norm(start_radius);
  if (!std::isfinite(result.radius) || result.radius <= 1e-6 ||
      result.radius > 100.0) {
    throw std::invalid_argument("circular path radius is invalid");
  }
  result.radial_axis = scale(start_radius, 1.0 / result.radius);
  result.normal = scale(plane, 1.0 / plane_norm);
  result.tangent_axis = cross(result.normal, result.radial_axis);

  const auto angle_of = [&](const double* point) {
    const auto radial = subtract(point, result.center.data());
    double angle = std::atan2(
        dot(radial, result.tangent_axis), dot(radial, result.radial_axis));
    if (angle < 0.0) angle += 2.0 * 3.14159265358979323846;
    return angle;
  };
  result.via_angle = angle_of(via);
  result.end_angle = angle_of(end);
  if (result.via_angle >= result.end_angle) {
    result.normal = scale(result.normal, -1.0);
    result.tangent_axis = scale(result.tangent_axis, -1.0);
    result.via_angle = angle_of(via);
    result.end_angle = angle_of(end);
  }
  if (result.via_angle <= 1e-9 ||
      result.end_angle - result.via_angle <= 1e-9 ||
      result.end_angle > 2.0 * 3.14159265358979323846 + 1e-9) {
    throw std::invalid_argument("circular arc angles are degenerate");
  }
  return result;
}

inline std::array<double, 3> lerp_position(
    const double* start, const double* end, double amount) {
  return {
      start[0] + amount * (end[0] - start[0]),
      start[1] + amount * (end[1] - start[1]),
      start[2] + amount * (end[2] - start[2]),
  };
}

inline Quaternion quaternion_from_rotation(const double* rotation) {
  Quaternion result;
  const double trace = rotation[0] + rotation[4] + rotation[8];
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    result.w = 0.25 * scale;
    result.x = (rotation[7] - rotation[5]) / scale;
    result.y = (rotation[2] - rotation[6]) / scale;
    result.z = (rotation[3] - rotation[1]) / scale;
  } else if (rotation[0] > rotation[4] && rotation[0] > rotation[8]) {
    const double scale = std::sqrt(1.0 + rotation[0] - rotation[4] -
                                   rotation[8]) * 2.0;
    result.w = (rotation[7] - rotation[5]) / scale;
    result.x = 0.25 * scale;
    result.y = (rotation[1] + rotation[3]) / scale;
    result.z = (rotation[2] + rotation[6]) / scale;
  } else if (rotation[4] > rotation[8]) {
    const double scale = std::sqrt(1.0 + rotation[4] - rotation[0] -
                                   rotation[8]) * 2.0;
    result.w = (rotation[2] - rotation[6]) / scale;
    result.x = (rotation[1] + rotation[3]) / scale;
    result.y = 0.25 * scale;
    result.z = (rotation[5] + rotation[7]) / scale;
  } else {
    const double scale = std::sqrt(1.0 + rotation[8] - rotation[0] -
                                   rotation[4]) * 2.0;
    result.w = (rotation[3] - rotation[1]) / scale;
    result.x = (rotation[2] + rotation[6]) / scale;
    result.y = (rotation[5] + rotation[7]) / scale;
    result.z = 0.25 * scale;
  }
  const double norm = std::sqrt(
      result.w * result.w + result.x * result.x + result.y * result.y +
      result.z * result.z);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("Cartesian orientation is invalid");
  }
  result.w /= norm;
  result.x /= norm;
  result.y /= norm;
  result.z /= norm;
  return result;
}

inline double dot(const Quaternion& lhs, const Quaternion& rhs) {
  return lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline double angular_distance(
    const Quaternion& start, const Quaternion& end) {
  return 2.0 * std::acos(std::clamp(std::abs(dot(start, end)), 0.0, 1.0));
}

inline Quaternion slerp(
    const Quaternion& start, Quaternion end, double amount) {
  double cosine = dot(start, end);
  if (cosine < 0.0) {
    cosine = -cosine;
    end.w = -end.w;
    end.x = -end.x;
    end.y = -end.y;
    end.z = -end.z;
  }
  cosine = std::clamp(cosine, -1.0, 1.0);
  if (cosine > 0.9995) {
    Quaternion result{
        start.w + amount * (end.w - start.w),
        start.x + amount * (end.x - start.x),
        start.y + amount * (end.y - start.y),
        start.z + amount * (end.z - start.z)};
    const double norm = std::sqrt(
        result.w * result.w + result.x * result.x + result.y * result.y +
        result.z * result.z);
    result.w /= norm;
    result.x /= norm;
    result.y /= norm;
    result.z /= norm;
    return result;
  }
  const double angle = std::acos(cosine);
  const double denominator = std::sin(angle);
  const double start_weight = std::sin((1.0 - amount) * angle) / denominator;
  const double end_weight = std::sin(amount * angle) / denominator;
  return Quaternion{
      start_weight * start.w + end_weight * end.w,
      start_weight * start.x + end_weight * end.x,
      start_weight * start.y + end_weight * end.y,
      start_weight * start.z + end_weight * end.z};
}

inline Quaternion circular_slerp_through_via(
    const Quaternion& start, const Quaternion& via, const Quaternion& end,
    double via_amount, double amount) {
  if (!std::isfinite(via_amount) || via_amount <= 0.0 || via_amount >= 1.0 ||
      !std::isfinite(amount)) {
    throw std::invalid_argument(
        "circular orientation progress is invalid");
  }
  const double bounded = std::clamp(amount, 0.0, 1.0);
  if (bounded <= via_amount) {
    return slerp(start, via, bounded / via_amount);
  }
  return slerp(
      via, end, (bounded - via_amount) / (1.0 - via_amount));
}

inline void rotation_from_quaternion(
    const Quaternion& value, double* rotation) {
  const double xx = value.x * value.x;
  const double yy = value.y * value.y;
  const double zz = value.z * value.z;
  const double xy = value.x * value.y;
  const double xz = value.x * value.z;
  const double yz = value.y * value.z;
  const double wx = value.w * value.x;
  const double wy = value.w * value.y;
  const double wz = value.w * value.z;
  rotation[0] = 1.0 - 2.0 * (yy + zz);
  rotation[1] = 2.0 * (xy - wz);
  rotation[2] = 2.0 * (xz + wy);
  rotation[3] = 2.0 * (xy + wz);
  rotation[4] = 1.0 - 2.0 * (xx + zz);
  rotation[5] = 2.0 * (yz - wx);
  rotation[6] = 2.0 * (xz - wy);
  rotation[7] = 2.0 * (yz + wx);
  rotation[8] = 1.0 - 2.0 * (xx + yy);
}

}  // namespace articore::cartesian
