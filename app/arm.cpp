// Created by zyfdx on 2026/10/30.

#include "arm.h"

#include "main.h"
#include "base/common/math.h"
#include <math.h>

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits), feedforward_(feedforward) {
}

bool Arm::feedbackReady() {
  return joint0_.connect_.check() && joint1_.connect_.check();
}

bool Arm::withinLimits(const float q[2]) const {
  for (int i = 0; i < 2; i++) {
    if (q[i] < limits_.q_min[i] || q[i] > limits_.q_max[i]) {
      return false;
    }
  }
  return true;
}

bool Arm::trajectoryReady() {
  return mode_ == Mode::Joint && feedbackReady();
}

bool Arm::calibrate(float q0_rad, float q1_rad) {
  if (mode_ != Mode::Disabled) {
    return false;
  }
  float q[2] = {q0_rad, q1_rad};
  if (!feedbackReady() || !withinLimits(q)) {
    return false;
  }
  joint0_.resetFeedbackAngle(math::rad2deg(q0_rad));
  joint1_.resetFeedbackAngle(math::rad2deg(q1_rad));
  state_.q[0] = q0_rad;
  state_.q[1] = q1_rad;
  state_.dq[0] = state_.dq[1] = 0.f;
  reference_.q[0] = q0_rad;
  reference_.q[1] = q1_rad;
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;
  calibrated_ = true;
  return true;
}

bool Arm::enable() {
  if (!feedbackReady()) {
    return false;
  }
  updateFeedback();
  reference_ = {};
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  mode_ = Mode::Joint;
  joint0_.mode_ = Motor::WORKING;
  joint1_.mode_ = Motor::WORKING;
  return true;
}

void Arm::disable() {
  mode_ = Mode::Disabled;
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  joint0_.mode_ = Motor::POWEROFF;
  joint1_.mode_ = Motor::POWEROFF;
}

bool Arm::setReference(const Reference& reference) {
  if (trajRunning()) {
    return false;
  }
  return applyReference(reference);
}

bool Arm::applyReference(const Reference& reference) {
  if (mode_ != Mode::Joint) {
    return false;
  }
  if (!withinLimits(reference.q)) {
    return false;
  }
  reference_ = reference;
  return true;
}

bool Arm::addJointReference(float speed0, float speed1, float dt_s) {
  if (!trajectoryReady() || trajRunning()) {
    return false;
  }
  Reference ref = reference_;
  ref.q[0] += speed0 * dt_s;
  ref.q[1] += speed1 * dt_s;
  ref.dq[0] = speed0;
  ref.dq[1] = speed1;
  ref.ddq[0] = ref.ddq[1] = 0.f;
  return applyReference(ref);
}

bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (!trajectoryReady()) {
    return false;
  }
  CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference target{};
  target.x = x_m;
  target.y = y_m;
  if (!ArmKinematics::lineReachable(start, target)) {
    return false;
  }
  float q[2];
  if (!ArmKinematics::inverse(target, reference_.q, trajectory_elbow_, q)) {
    return false;
  }
  if (!withinLimits(q)) {
    return false;
  }
  if (!trajectory_.start(start, target, duration_s)) {
    return false;
  }
  trajectory_type_ = TrajectoryType::Cartesian;
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
  return true;
}

bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
  if (!trajectoryReady()) {
    return false;
  }
  float q_start[2] = {state_.q[0], state_.q[1]};
  float q_end[2] = {q0_rad, q1_rad};
  if (!withinLimits(q_start) || !withinLimits(q_end)) {
    return false;
  }
  CartesianReference p_start = ArmKinematics::forward(q_start);
  CartesianReference p_end = ArmKinematics::forward(q_end);
  if (!trajectory_.start(p_start, p_end, duration_s)) {
    return false;
  }
  joint_trajectory_start_[0] = q_start[0];
  joint_trajectory_start_[1] = q_start[1];
  joint_trajectory_end_[0] = q_end[0];
  joint_trajectory_end_[1] = q_end[1];
  joint_trajectory_duration_ = duration_s;
  trajectory_type_ = TrajectoryType::Joint;
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
  return true;
}

void Arm::trajAbort() {
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;
}

void Arm::updateFeedback() {
  state_.q[0] = math::deg2rad(joint0_.realAngle());
  state_.q[1] = math::deg2rad(joint1_.realAngle());
  state_.dq[0] = math::deg2rad(joint0_.realSpeed());
  state_.dq[1] = math::deg2rad(joint1_.realSpeed());
}

bool Arm::trajStartCircle(float cx_m, float cy_m, float radius_m, float period_s,
                          int turns) {
  if (!trajectoryReady()) {
    return false;
  }
  if (radius_m <= 0.f || period_s <= 0.f || turns < 1) {
    return false;
  }
  const float d_oc = sqrtf(cx_m * cx_m + cy_m * cy_m);
  const float d_max = d_oc + radius_m;
  const float d_min = fabsf(d_oc - radius_m);
  if (d_max > ArmKinematics::l0 + ArmKinematics::l1 ||
      d_min < fabsf(ArmKinematics::l1 - ArmKinematics::l0)) {
    return false;
  }
  const CartesianReference now = ArmKinematics::forward(reference_.q);
  circle_cx_ = cx_m;
  circle_cy_ = cy_m;
  circle_r_ = radius_m;
  circle_omega_ = 6.2831853f / period_s;
  circle_theta0_ = atan2f(now.y - cy_m, now.x - cx_m);
  circle_r0_ = sqrtf((now.x - cx_m) * (now.x - cx_m) +
                     (now.y - cy_m) * (now.y - cy_m));
  circle_blend_ = 0.6f;
  circle_duration_ = period_s * (float)turns;
  trajectory_.abort();
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Circle;
  return true;
}

bool Arm::trajStartSquare(float cx_m, float cy_m, float half_m, float corner_r_m,
                          float duration_s, int dir) {
  if (!trajectoryReady()) {
    return false;
  }
  if (duration_s <= 0.f || half_m <= 0.f) {
    return false;
  }
  path_.init(cx_m, cy_m, half_m, corner_r_m, dir);
  path_len_ = path_.length();
  if (path_len_ <= 1e-4f) {
    return false;
  }
  float px = 0.f, py = 0.f, tx = 0.f, ty = 0.f, kappa = 0.f;
  path_.at(0.f, px, py, tx, ty, kappa);
  CartesianReference start{};
  start.x = px;
  start.y = py;
  {
    CartesianReference prev = start;
    const int kSamples = 64;
    for (int i = 1; i <= kSamples; ++i) {
      float x = 0.f, y = 0.f, t1 = 0.f, t2 = 0.f, k = 0.f;
      path_.at(path_len_ * (float)i / (float)kSamples, x, y, t1, t2, k);
      CartesianReference cur{};
      cur.x = x;
      cur.y = y;
      if (!ArmKinematics::lineReachable(prev, cur)) {
        return false;
      }
      prev = cur;
    }
  }
  float q[2];
  if (!ArmKinematics::inverse(start, reference_.q, trajectory_elbow_, q)) {
    return false;
  }
  if (!withinLimits(q)) {
    return false;
  }
  trajectory_.abort();
  path_duration_ = duration_s;
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Path;
  return true;
}

void Arm::trajectoryPlanner() {
  if (trajectory_type_ == TrajectoryType::None) {
    return;
  }
  float elapsed_s = (float)(HAL_GetTick() - trajectory_start_tick_) * 0.001f;

  Reference ref;
  if (trajectory_type_ == TrajectoryType::Cartesian) {
    CartesianReference point = trajectory_.sample(elapsed_s);
    float q[2];
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, q)) {
      trajAbort();
      return;
    }
    if (!withinLimits(q)) {
      trajAbort();
      return;
    }
    ref.q[0] = q[0];
    ref.q[1] = q[1];
    float dq[2], ddq[2];
    if (ArmKinematics::derivatives(point, q, dq, ddq)) {
      ref.dq[0] = dq[0];
      ref.dq[1] = dq[1];
      ref.ddq[0] = ddq[0];
      ref.ddq[1] = ddq[1];
    } else {
      ref.dq[0] = ref.dq[1] = 0.f;
      ref.ddq[0] = ref.ddq[1] = 0.f;
    }
    reference_ = ref;
    if (!trajectory_.running()) {
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
  } else if (trajectory_type_ == TrajectoryType::Circle) {
    const float theta = circle_theta0_ + circle_omega_ * elapsed_s;
    float r = circle_r_;
    float rdot = 0.f;
    float rddot = 0.f;
    if (circle_blend_ > 0.f && elapsed_s < circle_blend_) {
      const float s = elapsed_s / circle_blend_;
      const float w = s * s * s * (10.f + s * (-15.f + 6.f * s));
      const float w1 = 30.f * s * s * (1.f - s) * (1.f - s);
      const float w2 = 60.f * s * (1.f - s) * (1.f - 2.f * s);
      const float dr = circle_r_ - circle_r0_;
      r = circle_r0_ + dr * w;
      rdot = dr * w1 / circle_blend_;
      rddot = dr * w2 / (circle_blend_ * circle_blend_);
    }
    const float ct = cosf(theta);
    const float st = sinf(theta);
    const float v = r * circle_omega_;
    CartesianReference point;
    point.x = circle_cx_ + r * ct;
    point.y = circle_cy_ + r * st;
    point.dx = rdot * ct - v * st;
    point.dy = rdot * st + v * ct;
    point.ddx = (rddot - v * circle_omega_) * ct - 2.f * rdot * circle_omega_ * st;
    point.ddy = (rddot - v * circle_omega_) * st + 2.f * rdot * circle_omega_ * ct;
    float q[2];
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, q)) {
      trajAbort();
      return;
    }
    if (!withinLimits(q)) {
      trajAbort();
      return;
    }
    ref.q[0] = q[0];
    ref.q[1] = q[1];
    float dq[2], ddq[2];
    if (ArmKinematics::derivatives(point, q, dq, ddq)) {
      ref.dq[0] = dq[0];
      ref.dq[1] = dq[1];
      ref.ddq[0] = ddq[0];
      ref.ddq[1] = ddq[1];
    } else {
      ref.dq[0] = ref.dq[1] = 0.f;
      ref.ddq[0] = ref.ddq[1] = 0.f;
    }
    reference_ = ref;
    if (elapsed_s >= circle_duration_) {
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
  } else if (trajectory_type_ == TrajectoryType::Path) {
    const float tau = (path_duration_ > 0.f)
                          ? (elapsed_s / path_duration_ > 1.f ? 1.f : elapsed_s / path_duration_)
                          : 1.f;
    const float t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
    const float u = 10.f * t3 - 15.f * t4 + 6.f * t5;
    const float du = 30.f * t2 - 60.f * t3 + 30.f * t4;
    const float ddu = 60.f * tau - 180.f * t2 + 120.f * t3;
    const float s = path_len_ * u;
    const float sd = path_len_ * du / path_duration_;
    const float sdd = path_len_ * ddu / (path_duration_ * path_duration_);
    float px = 0.f, py = 0.f, tx = 1.f, ty = 0.f, kappa = 0.f;
    path_.at(s, px, py, tx, ty, kappa);
    CartesianReference point;
    point.x = px;
    point.y = py;
    point.dx = tx * sd;
    point.dy = ty * sd;
    const float an = kappa * sd * sd;
    point.ddx = tx * sdd - ty * an;
    point.ddy = ty * sdd + tx * an;
    float q[2];
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, q)) {
      trajAbort();
      return;
    }
    if (!withinLimits(q)) {
      trajAbort();
      return;
    }
    ref.q[0] = q[0];
    ref.q[1] = q[1];
    float dq[2], ddq[2];
    if (ArmKinematics::derivatives(point, q, dq, ddq)) {
      ref.dq[0] = dq[0];
      ref.dq[1] = dq[1];
      ref.ddq[0] = ddq[0];
      ref.ddq[1] = ddq[1];
    } else {
      ref.dq[0] = ref.dq[1] = 0.f;
      ref.ddq[0] = ref.ddq[1] = 0.f;
    }
    reference_ = ref;
    if (elapsed_s >= path_duration_) {
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
  } else {
    float q[2], dq[2], ddq[2];
    for (int i = 0; i < 2; i++) {
      ArmTrajectory::interpolate(joint_trajectory_start_[i],
                                 joint_trajectory_end_[i], elapsed_s,
                                 joint_trajectory_duration_, q[i], dq[i],
                                 ddq[i]);
    }
    if (!withinLimits(q)) {
      trajAbort();
      return;
    }
    ref.q[0] = q[0];
    ref.q[1] = q[1];
    ref.dq[0] = dq[0];
    ref.dq[1] = dq[1];
    ref.ddq[0] = ddq[0];
    ref.ddq[1] = ddq[1];
    reference_ = ref;
    if (elapsed_s >= joint_trajectory_duration_) {
      trajectory_.abort();
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
  }
}

// 写位置目标、速度前馈、加速度前馈进control_data
void Arm::writeMotorCommands() {
  if (mode_ != Mode::Joint) {
    return;
  }
  joint0_.setAngleSpeed(math::rad2deg(reference_.q[0]),
                        math::rad2deg(reference_.dq[0]),
                        feedforward_.k_acc[0] * reference_.ddq[0]);
  joint1_.setAngleSpeed(math::rad2deg(reference_.q[1]),
                        math::rad2deg(reference_.dq[1]),
                        feedforward_.k_acc[1] * reference_.ddq[1]);
}

void Arm::handle() {
  updateFeedback();
  if (mode_ != Mode::Joint) {
    return;
  }
  trajectoryPlanner();
  writeMotorCommands();
}
