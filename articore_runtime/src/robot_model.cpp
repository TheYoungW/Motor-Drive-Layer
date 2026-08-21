#include "articore/detail/robot_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace articore {
namespace {

constexpr uint32_t kDof = 7;

// Fixed Yunyi gripper-center control frame relative to link7. This is the
// midpoint of the two gripper slide origins in the product URDF. Products
// without grippers use link7 directly (an identity tool transform).
constexpr std::array<double, 3> kGripperToolTranslation = {
    -0.004, 0.0, -0.178};

struct JointSpec {
  std::array<double, 3> axis;
  std::array<double, 3> translation;
  std::array<double, 9> rotation;
  double mass;
  std::array<double, 3> com;
  std::array<double, 9> inertia;
  double lower;
  double upper;
};

constexpr std::array<double, 9> kIdentity = {1, 0, 0, 0, 1, 0, 0, 0, 1};

const std::array<JointSpec, kDof> kLeft = {
    JointSpec{{0, 1, 0}, {4.99999999996615e-05, .166992997382172, .6337095589940774},
     {{1, 0, 0, 0, .9659258262890676, -.2588190451025233, 0, .2588190451025233, .9659258262890676}},
     .612628872562558, {{.00274747462925445, .0322526731703391, -.00124058111370681}},
     {{.000675991629583124, 8.42180561013196e-09, -4.33749178612517e-07, 8.42180561013196e-09, .000443464013577626, -3.75589885873176e-08, -4.33749178612517e-07, -3.75589885873176e-08, .000393481626554317}}, -2.745, 2.745},
    JointSpec{{1, 0, 0}, {-5.0000000000329e-05, .0579999999999999, 0},
     {{1, 0, 0, 0, .9659258262890676, .2588190451025233, 0, -.2588190451025233, .9659258262890676}},
     .791808015126076, {{.0151683415787723, -.0118084664996551, -.0646524014617347}},
     {{.000365729757645324, 3.4401550972432e-07, -3.6334893034197e-08, 3.4401550972432e-07, .000320126103367448, 3.29450787874754e-07, -3.6334893034197e-08, 3.29450787874754e-07, .000323002055254654}}, -.3489, 2.2678},
    JointSpec{{0, 0, -1}, {.00434139983447429, .00799999999863457, -.11925}, kIdentity,
     .842434356423893, {{.00373447329073571, -.0242302055769374, -.0898395650759556}},
     {{.00075945085050508, -4.55331742412528e-08, -2.37479824083893e-05, -4.55331742412528e-08, .000873620695617603, 1.72938628404127e-07, -2.37479824083893e-05, 1.72938628404127e-07, .000336273525464224}}, -2.5294, 2.5294},
    JointSpec{{0, -1, 0}, {.00719572519347152, .000125000001366032, -.134000000000001}, kIdentity,
     .117703346646349, {{-.00681977687417615, .000291370140096558, -.0324571817459631}},
     {{4.34039412925388e-05, 1.09777265113083e-07, 6.21718414428949e-06, 1.09777265113083e-07, 5.15367325195561e-05, 4.330056213507e-07, 6.21718414428949e-06, 4.330056213507e-07, 2.60949506604642e-05}}, -.1744, 2.2678},
    JointSpec{{0, 0, 1}, {-.00983268534763712, .000750000000000084, -.059}, kIdentity,
     .818152596601028, {{3.09329397326042e-05, .00840471837158996, -.0632387515751979}},
     {{.000387196493000292, -1.57672112846699e-07, -9.40447660695902e-07, -1.57672112846699e-07, .000444266598402289, -3.56195175154644e-08, -9.40447660695902e-07, -3.56195175154644e-08, .000267197591088817}}, -2.0933, 2.0933},
    JointSpec{{0, -1, 0}, {0, 0, -.155}, kIdentity,
     .320964692532981, {{-.00124686281535716, -9.45680002003568e-05, 1.01546292083365e-05}},
     {{.00013839024539347, -4.91126777188069e-09, -9.55552374602084e-09, -4.91126777188069e-09, 9.30413058648934e-05, -8.11389854588129e-08, -9.55552374602084e-09, -8.11389854588129e-08, .000106857256018368}}, -.785, .785},
    JointSpec{{1, 0, 0}, {0, 0, 0}, kIdentity,
     .5252461094149239, {{.006174139281545251, 6.6839032963270095e-06, -.08065576737435959}},
     {{.0006194779907355286, -4.116832552228634e-07, -3.5862947680295854e-05, -4.116832552228634e-07, .0003337154126119054, -2.0686029272035328e-08, -3.5862947680295854e-05, -2.0686029272035328e-08, .0004867067624532618}}, -1.3956, 1.3956},
};

const std::array<JointSpec, kDof> kRight = {
    JointSpec{{0, -1, 0}, {5e-05, -.16699, .63371},
     {{1, 0, 0, 0, .9659258262890676, .2588190451025233, 0, -.2588190451025233, .9659258262890676}},
     .48001, {{.0023086, -.047309, -.00070626}}, {{.00052943, -9.5749e-08, 1.5354e-07, -9.5749e-08, .00035158, -5.477e-06, 1.5354e-07, -5.477e-06, .00031895}}, -2.745, 2.745},
    JointSpec{{1, 0, 0}, {-5e-05, -.058, 0},
     {{1, 0, 0, 0, .9659258262890676, -.2588190451025233, 0, .2588190451025233, .9659258262890676}},
     .58736, {{.011672, -.0081585, -.073683}}, {{.00025571, -1.2484e-06, -2.2208e-09, -1.2484e-06, .00023221, 3.4222e-06, -2.2208e-09, 3.4222e-06, .00024057}}, -2.2678, .3489},
    JointSpec{{0, 0, -1}, {.0043414, -.008, -.11925}, kIdentity,
     .611557224869322, {{.00475653763931137, .000909331136429409, -.103630304411}}, {{.000451307360064067, 1.65228566697274e-07, -1.16041204000365e-05, 1.65228566697274e-07, .00051510133238372, 9.43935847327683e-08, -1.16041204000365e-05, 9.43935847327683e-08, .000240668338849631}}, -2.5294, 2.5294},
    JointSpec{{0, -1, 0}, {.0071957, 0, -.134}, kIdentity,
     .195867906346611, {{-.00621157811955259, .038997859836709, -.0284254443400442}}, {{7.84741501398083e-05, 2.89748318044577e-13, 1.24343661647735e-05, 2.89748318044577e-13, 9.48662248078727e-05, 3.01451928731592e-08, 1.24343661647735e-05, 3.01451928731592e-08, 3.62455472671027e-05}}, -.1744, 2.2678},
    JointSpec{{0, 0, 1}, {-.00983268534763703, -.000874999999999626, -.059}, kIdentity,
     .699252923462321, {{-4.054896968108458e-05, -.00023143739073341893, -.05927540371626118}}, {{.000279517457503872, -6.75250011359066e-09, -3.90058392089654e-07, -6.75250011359066e-09, .000323432935036869, -4.41063467050085e-07, -3.90058392089654e-07, -4.41063467050085e-07, .000234928631330703}}, -2.0933, 2.0933},
    JointSpec{{0, -1, 0}, {0, 0, -.155}, kIdentity,
     .320964693534996, {{-.00124686275519197, 9.49678438605384e-05, 2.53694008134353e-07}}, {{.000138390245498164, 4.57886915220354e-09, -1.37148669611488e-09, 4.57886915220354e-09, 9.30327520955922e-05, -2.58789838486845e-08, -1.37148669611488e-09, -2.58789838486845e-08, .000106865810438436}}, -.785, .785},
    JointSpec{{1, 0, 0}, {0, 0, 0}, kIdentity,
     .23506944299988497, {{.01891088880205441, 1.4637138359972421e-05, -.0917595702750599}}, {{.00048561345166645204, -4.587719953503175e-07, -7.42820904606914e-05, -4.587719953503175e-07, .00023295121727036605, -4.6004475662129356e-08, -7.42820904606914e-05, -4.6004475662129356e-08, .00040455032567747733}}, -1.3956, 1.3956},
};

template <size_t Rows, size_t Cols>
Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor> row_major(
    const std::array<double, Rows * Cols>& values) {
  return Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor>>(
      values.data());
}

void copy_string(char* output, size_t size, const std::string& value) {
  std::snprintf(output, size, "%s", value.c_str());
}

void require_output(const void* output, uint32_t actual, uint32_t expected,
                    const char* name) {
  if (!output) throw std::invalid_argument(std::string(name) + " is null");
  if (actual != expected) {
    throw std::invalid_argument(std::string(name) + " has incorrect element count");
  }
}

}  // namespace

struct RobotModel::Impl {
  std::string product_id;
  uint32_t side;
  std::string prefix;
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> realtime_data;
  pinocchio::JointIndex end_joint = 0;
  pinocchio::FrameIndex end_frame = 0;
  pinocchio::SE3 end_placement = pinocchio::SE3::Identity();
  bool uses_gripper_tool_frame = false;

  Impl(std::string product, uint32_t selected_side,
       bool use_gripper_tool_frame)
      : product_id(std::move(product)), side(selected_side),
        prefix(side == ARTICORE_ROBOT_LEFT ? "l" : "r"),
        uses_gripper_tool_frame(use_gripper_tool_frame) {
    if (product_id != "yunyi_v1_0") {
      throw std::invalid_argument("unsupported robot product_id: " + product_id);
    }
    if (side != ARTICORE_ROBOT_LEFT && side != ARTICORE_ROBOT_RIGHT) {
      throw std::invalid_argument("robot side must be LEFT(0) or RIGHT(1)");
    }
    const auto& specs = side == ARTICORE_ROBOT_LEFT ? kLeft : kRight;
    pinocchio::JointIndex parent = 0;
    for (uint32_t i = 0; i < kDof; ++i) {
      const auto& spec = specs[i];
      const Eigen::Vector3d axis(spec.axis[0], spec.axis[1], spec.axis[2]);
      const Eigen::Vector3d translation(spec.translation[0], spec.translation[1],
                                        spec.translation[2]);
      const Eigen::Matrix3d rotation = row_major<3, 3>(spec.rotation);
      const std::string name = prefix + "-joint" + std::to_string(i + 1);
      const auto joint = model.addJoint(
          parent, pinocchio::JointModelRevoluteUnaligned(axis),
          pinocchio::SE3(rotation, translation), name);
      model.appendBodyToJoint(
          joint,
          pinocchio::Inertia(spec.mass,
                             Eigen::Vector3d(spec.com[0], spec.com[1], spec.com[2]),
                             row_major<3, 3>(spec.inertia)),
          pinocchio::SE3::Identity());
      model.lowerPositionLimit[static_cast<Eigen::Index>(i)] = spec.lower;
      model.upperPositionLimit[static_cast<Eigen::Index>(i)] = spec.upper;
      parent = joint;
    }
    end_joint = parent;
    if (uses_gripper_tool_frame) {
      end_placement.translation() = Eigen::Vector3d(
          kGripperToolTranslation[0], kGripperToolTranslation[1],
          kGripperToolTranslation[2]);
    }
    end_frame = model.addFrame(pinocchio::Frame(
        prefix + (uses_gripper_tool_frame ? "-tool0" : "-link7"),
        end_joint, end_placement, pinocchio::OP_FRAME));
    model.gravity.linear() = Eigen::Vector3d(0, 0, -9.81);
    realtime_data = std::make_unique<pinocchio::Data>(model);
  }

  Eigen::VectorXd vector(const double* values, uint32_t count,
                         const char* name) const {
    require_output(values, count, kDof, name);
    const auto mapped = Eigen::Map<const Eigen::VectorXd>(values, kDof);
    if (!mapped.allFinite()) {
      throw std::invalid_argument(std::string(name) + " contains non-finite values");
    }
    return mapped;
  }
};

RobotModel::RobotModel(std::string product_id, uint32_t side,
                       bool use_gripper_tool_frame)
    : impl_(std::make_unique<Impl>(
          std::move(product_id), side, use_gripper_tool_frame)) {}
RobotModel::~RobotModel() = default;

void RobotModel::get_info(ArticoreRobotModelInfo* info) const {
  if (!info || info->struct_size < sizeof(*info)) {
    throw std::invalid_argument("robot model info is null or too small");
  }
  const uint32_t size = info->struct_size;
  *info = {};
  info->struct_size = size;
  info->dof = kDof;
  info->side = impl_->side;
  copy_string(info->product_id, sizeof(info->product_id), impl_->product_id);
  copy_string(info->end_effector_frame, sizeof(info->end_effector_frame),
              impl_->prefix +
                  (impl_->uses_gripper_tool_frame ? "-tool0" : "-link7"));
  for (uint32_t i = 0; i < kDof; ++i) {
    copy_string(info->joint_names[i], sizeof(info->joint_names[i]),
                impl_->prefix + "-joint" + std::to_string(i + 1));
    info->lower_limits[i] = impl_->model.lowerPositionLimit[i];
    info->upper_limits[i] = impl_->model.upperPositionLimit[i];
  }
}

void RobotModel::fk(const double* q, uint32_t count, ArticoreRobotPose* pose) const {
  if (!pose || pose->struct_size < sizeof(*pose)) {
    throw std::invalid_argument("robot pose is null or too small");
  }
  const auto configuration = impl_->vector(q, count, "q");
  pinocchio::Data data(impl_->model);
  pinocchio::forwardKinematics(impl_->model, data, configuration);
  const pinocchio::SE3 transform =
      data.oMi[impl_->end_joint] * impl_->end_placement;
  for (int i = 0; i < 3; ++i) pose->position[i] = transform.translation()[i];
  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation = transform.rotation();
  std::copy(rotation.data(), rotation.data() + 9, pose->rotation);
  const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> homogeneous = transform.toHomogeneousMatrix();
  std::copy(homogeneous.data(), homogeneous.data() + 16, pose->homogeneous);
}

void RobotModel::jacobian(const double* q, uint32_t count, uint32_t reference,
                          double* output, uint32_t output_count) const {
  require_output(output, output_count, 6 * kDof, "jacobian output");
  const auto configuration = impl_->vector(q, count, "q");
  pinocchio::ReferenceFrame frame;
  switch (reference) {
    case ARTICORE_JACOBIAN_LOCAL: frame = pinocchio::LOCAL; break;
    case ARTICORE_JACOBIAN_WORLD: frame = pinocchio::WORLD; break;
    case ARTICORE_JACOBIAN_LOCAL_WORLD_ALIGNED: frame = pinocchio::LOCAL_WORLD_ALIGNED; break;
    default: throw std::invalid_argument("unknown Jacobian reference frame");
  }
  pinocchio::Data data(impl_->model);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kDof);
  jacobian.setZero();
  pinocchio::computeFrameJacobian(
      impl_->model, data, configuration, impl_->end_frame, frame, jacobian);
  Eigen::Map<Eigen::Matrix<double, 6, kDof, Eigen::RowMajor>> mapped(output);
  mapped = jacobian;
}

void RobotModel::gravity(const double* q, uint32_t count, double* output,
                         uint32_t output_count) const {
  require_output(output, output_count, kDof, "gravity output");
  pinocchio::Data data(impl_->model);
  Eigen::Map<Eigen::VectorXd>(output, kDof) = pinocchio::computeGeneralizedGravity(
      impl_->model, data, impl_->vector(q, count, "q"));
}

void RobotModel::gravity_realtime(const std::array<double, 7>& q,
                                  std::array<double, 7>& output) {
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> configuration(q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> torque(output.data());
  torque = pinocchio::computeGeneralizedGravity(
      impl_->model, *impl_->realtime_data, configuration);
}

void RobotModel::mass_matrix(const double* q, uint32_t count, double* output,
                             uint32_t output_count) const {
  require_output(output, output_count, kDof * kDof, "mass matrix output");
  pinocchio::Data data(impl_->model);
  Eigen::MatrixXd mass = pinocchio::crba(impl_->model, data, impl_->vector(q, count, "q"));
  mass.triangularView<Eigen::StrictlyLower>() = mass.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::Map<Eigen::Matrix<double, kDof, kDof, Eigen::RowMajor>> mapped(output);
  mapped = mass;
}

void RobotModel::coriolis_matrix(const double* q, uint32_t q_count,
                                 const double* dq, uint32_t dq_count,
                                 double* output, uint32_t output_count) const {
  require_output(output, output_count, kDof * kDof, "coriolis matrix output");
  pinocchio::Data data(impl_->model);
  const Eigen::MatrixXd matrix = pinocchio::computeCoriolisMatrix(
      impl_->model, data, impl_->vector(q, q_count, "q"),
      impl_->vector(dq, dq_count, "dq"));
  Eigen::Map<Eigen::Matrix<double, kDof, kDof, Eigen::RowMajor>> mapped(output);
  mapped = matrix;
}

void RobotModel::nonlinear_effects(const double* q, uint32_t q_count,
                                   const double* dq, uint32_t dq_count,
                                   double* output, uint32_t output_count) const {
  require_output(output, output_count, kDof, "nonlinear effects output");
  pinocchio::Data data(impl_->model);
  Eigen::Map<Eigen::VectorXd>(output, kDof) = pinocchio::nonLinearEffects(
      impl_->model, data, impl_->vector(q, q_count, "q"),
      impl_->vector(dq, dq_count, "dq"));
}

void RobotModel::rnea(const double* q, uint32_t q_count, const double* dq,
                      uint32_t dq_count, const double* ddq, uint32_t ddq_count,
                      double* output, uint32_t output_count) const {
  require_output(output, output_count, kDof, "RNEA output");
  pinocchio::Data data(impl_->model);
  Eigen::Map<Eigen::VectorXd>(output, kDof) = pinocchio::rnea(
      impl_->model, data, impl_->vector(q, q_count, "q"),
      impl_->vector(dq, dq_count, "dq"), impl_->vector(ddq, ddq_count, "ddq"));
}

void RobotModel::aba(const double* q, uint32_t q_count, const double* dq,
                     uint32_t dq_count, const double* torque,
                     uint32_t torque_count, double* output,
                     uint32_t output_count) const {
  require_output(output, output_count, kDof, "ABA output");
  pinocchio::Data data(impl_->model);
  Eigen::Map<Eigen::VectorXd>(output, kDof) = pinocchio::aba(
      impl_->model, data, impl_->vector(q, q_count, "q"),
      impl_->vector(dq, dq_count, "dq"), impl_->vector(torque, torque_count, "torque"));
}

void RobotModel::ik(const ArticoreRobotPose* target, const double* initial_q,
                    uint32_t initial_q_count, const ArticoreIkOptions* options,
                    ArticoreIkResult* result) const {
  if (!target || target->struct_size < sizeof(*target))
    throw std::invalid_argument("IK target pose is null or too small");
  if (!result || result->struct_size < sizeof(*result))
    throw std::invalid_argument("IK result is null or too small");
  if (options && options->struct_size < sizeof(*options))
    throw std::invalid_argument("IK options struct is too small");
  const uint32_t max_iterations = options && options->max_iterations ? options->max_iterations : 1000;
  const uint32_t max_retries = options && options->max_retries
      ? options->max_retries
      : 8;
  const double tolerance = options && options->tolerance > 0 ? options->tolerance : 1e-4;
  const double step_size = options && options->step_size > 0 ? options->step_size : .5;
  const double damping = options && options->damping > 0 ? options->damping : 1e-6;
  if (max_iterations > 1'000'000 || max_retries > 1'000) {
    throw std::invalid_argument("IK iteration or retry count is unreasonably large");
  }
  if (!std::isfinite(tolerance) || !std::isfinite(step_size) ||
      !std::isfinite(damping)) {
    throw std::invalid_argument("IK options must be finite");
  }
  const Eigen::Matrix3d rotation = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(target->rotation);
  const Eigen::Vector3d translation(target->position[0], target->position[1], target->position[2]);
  if (!rotation.allFinite() || !translation.allFinite() ||
      !rotation.transpose().isApprox(rotation.inverse(), 1e-8) ||
      rotation.determinant() <= 0.0) {
    throw std::invalid_argument("IK target must contain a finite proper rotation");
  }
  // Keep the iterative solve at joint7. A gripper product converts its public
  // tool-center target into the equivalent link7 target first.
  const pinocchio::SE3 desired =
      pinocchio::SE3(rotation, translation) * impl_->end_placement.inverse();
  Eigen::VectorXd seed = impl_->vector(initial_q, initial_q_count, "initial_q");
  struct Attempt { Eigen::VectorXd q; double error; uint32_t iterations; bool success; };
  auto solve = [&](Eigen::VectorXd q) {
    pinocchio::Data data(impl_->model);
    Attempt attempt{q, std::numeric_limits<double>::infinity(), 0, false};
    for (uint32_t iteration = 0; iteration < max_iterations; ++iteration) {
      pinocchio::forwardKinematics(impl_->model, data, attempt.q);
      const pinocchio::Motion error = pinocchio::log6(data.oMi[impl_->end_joint].actInv(desired));
      attempt.error = error.toVector().norm();
      attempt.iterations = iteration;
      if (attempt.error < tolerance) { attempt.success = true; return attempt; }
      pinocchio::computeJointJacobians(impl_->model, data, attempt.q);
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, kDof);
      jacobian.setZero();
      pinocchio::getJointJacobian(impl_->model, data, impl_->end_joint, pinocchio::LOCAL, jacobian);
      Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose();
      normal.diagonal().array() += damping * std::max(1.0, attempt.error * 10.0);
      const Eigen::VectorXd delta = step_size * jacobian.transpose() * normal.ldlt().solve(error.toVector());
      double alpha = 1.0;
      for (int line = 0; line < 4; ++line) {
        Eigen::VectorXd candidate = pinocchio::integrate(impl_->model, attempt.q, alpha * delta);
        candidate = candidate.cwiseMax(impl_->model.lowerPositionLimit).cwiseMin(impl_->model.upperPositionLimit);
        pinocchio::forwardKinematics(impl_->model, data, candidate);
        const double candidate_error = pinocchio::log6(data.oMi[impl_->end_joint].actInv(desired)).toVector().norm();
        if (candidate_error < attempt.error) { attempt.q = candidate; break; }
        alpha *= .5;
      }
    }
    pinocchio::forwardKinematics(impl_->model, data, attempt.q);
    attempt.error = pinocchio::log6(data.oMi[impl_->end_joint].actInv(desired)).toVector().norm();
    attempt.iterations = max_iterations;
    attempt.success = attempt.error < tolerance;
    return attempt;
  };
  Attempt best = solve(seed);
  std::mt19937_64 rng(options ? options->random_seed : 0);
  for (uint32_t retry = 0; retry < max_retries && !best.success; ++retry) {
    Eigen::VectorXd random_q(kDof);
    for (uint32_t i = 0; i < kDof; ++i) {
      std::uniform_real_distribution<double> distribution(
          impl_->model.lowerPositionLimit[i], impl_->model.upperPositionLimit[i]);
      random_q[i] = distribution(rng);
    }
    Attempt candidate = solve(random_q);
    if (candidate.error < best.error) best = std::move(candidate);
  }
  const uint32_t size = result->struct_size;
  *result = {};
  result->struct_size = size;
  result->success = best.success ? 1 : 0;
  result->iterations = best.iterations;
  result->dof = kDof;
  result->error_norm = best.error;
  std::copy(best.q.data(), best.q.data() + kDof, result->q);
}

}  // namespace articore
