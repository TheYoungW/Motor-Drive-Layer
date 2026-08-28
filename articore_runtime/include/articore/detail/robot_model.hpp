#pragma once

#include <chrono>
#include <cstdint>
#include <array>
#include <memory>
#include <string>

#include "articore/runtime_abi.h"

namespace articore {

namespace detail {

using YunyiIkSeed = std::array<double, ARTICORE_PRODUCT_ARM_DOF>;
using YunyiPtpFallbackSeeds = std::array<YunyiIkSeed, 8>;

// The live/planned configuration remains the primary set_pose IK seed. These are the
// eight deterministic fallbacks: product Home, zero, joint-range midpoint and
// five low-discrepancy configurations inside the product limits.
YunyiPtpFallbackSeeds yunyi_ptp_fallback_ik_seeds(
    uint32_t side,
    const YunyiIkSeed& lower_limits,
    const YunyiIkSeed& upper_limits);

}  // namespace detail

class RobotModel final {
 public:
  RobotModel(std::string product_id, uint32_t side,
             bool use_gripper_tool_frame = false);
  RobotModel(std::string product_id, uint32_t side,
             const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& tcp_offset);
  ~RobotModel();
  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;

  void get_info(ArticoreRobotModelInfo* info) const;
  void fk(const double* q, uint32_t q_count, ArticoreRobotPose* pose) const;
  void jacobian(const double* q, uint32_t q_count, uint32_t reference,
                double* output, uint32_t output_count) const;
  void gravity(const double* q, uint32_t q_count, double* output,
               uint32_t output_count) const;
  // Allocation-free workspace owned by this model instance. Runtime control
  // code serializes access and calls this only from its native worker.
  void gravity_realtime(const std::array<double, 7>& q,
                        std::array<double, 7>& output);
  void mass_matrix(const double* q, uint32_t q_count, double* output,
                   uint32_t output_count) const;
  void coriolis_matrix(const double* q, uint32_t q_count, const double* dq,
                       uint32_t dq_count, double* output,
                       uint32_t output_count) const;
  void nonlinear_effects(const double* q, uint32_t q_count, const double* dq,
                         uint32_t dq_count, double* output,
                         uint32_t output_count) const;
  void rnea(const double* q, uint32_t q_count, const double* dq,
            uint32_t dq_count, const double* ddq, uint32_t ddq_count,
            double* output, uint32_t output_count) const;
  void aba(const double* q, uint32_t q_count, const double* dq,
           uint32_t dq_count, const double* torque, uint32_t torque_count,
           double* output, uint32_t output_count) const;
  void ik(const ArticoreRobotPose* target, const double* initial_q,
          uint32_t initial_q_count, const ArticoreIkOptions* options,
          ArticoreIkResult* result) const;
  // Continuous Cartesian paths stay on the branch defined by the current
  // planned configuration. Solve only from that seed: no deterministic
  // fallback set and no random retry seeds.
  void ik_from_seed(const ArticoreRobotPose* target, const double* initial_q,
                    uint32_t initial_q_count,
                    const ArticoreIkOptions* options,
                    ArticoreIkResult* result,
                    const double* preferred_q = nullptr,
                    uint32_t preferred_q_count = 0) const;
  // set_pose endpoint IK searches the configured retry budget and selects the
  // successful solution nearest to the measured/planned seed.
  void ik_nearest(const ArticoreRobotPose* target, const double* initial_q,
                  uint32_t initial_q_count,
                  const ArticoreIkOptions* options,
                  ArticoreIkResult* result) const;
  // set_pose uses the same nearest-solution policy but bounds
  // the search by a steady-clock deadline. A converged solution found before
  // the deadline is returned; otherwise the call fails without installing a
  // partial joint target.
  void ik_nearest_until(
      const ArticoreRobotPose* target, const double* initial_q,
      uint32_t initial_q_count, const ArticoreIkOptions* options,
      std::chrono::steady_clock::time_point deadline,
      ArticoreIkResult* result) const;

 private:
  void ik_impl(const ArticoreRobotPose* target, const double* initial_q,
               uint32_t initial_q_count, const ArticoreIkOptions* options,
               ArticoreIkResult* result,
               bool prefer_nearest_success,
               const std::chrono::steady_clock::time_point* deadline,
               const detail::YunyiPtpFallbackSeeds* fallback_seeds =
                   nullptr,
               bool allow_random_retries = true,
               bool regularize_to_seed = false,
               const double* preferred_q = nullptr,
               uint32_t preferred_q_count = 0) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace articore
