#pragma once

#include <cstdint>
#include <array>
#include <memory>
#include <string>

#include "articore/runtime_abi.h"

namespace articore {

class RobotModel final {
 public:
  RobotModel(std::string product_id, uint32_t side,
             bool use_gripper_tool_frame = false);
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
  // Product PTP planning searches the configured retry budget and selects the
  // successful solution nearest to the measured/planned seed.
  void ik_nearest(const ArticoreRobotPose* target, const double* initial_q,
                  uint32_t initial_q_count,
                  const ArticoreIkOptions* options,
                  ArticoreIkResult* result) const;

 private:
  void ik_impl(const ArticoreRobotPose* target, const double* initial_q,
               uint32_t initial_q_count, const ArticoreIkOptions* options,
               ArticoreIkResult* result,
               bool prefer_nearest_success) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace articore
