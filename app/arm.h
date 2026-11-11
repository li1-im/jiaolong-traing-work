// Created by NoSignal on 2026/10/16.

#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_ARM_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_ARM_H

#include "base/motor/motor.h"
#include "arm_trajectory.h"
#include "arm_path.h"

class Arm {
 public:
  struct State {
    float q[2]{};
    float dq[2]{};
  };

  struct Reference {
    float q[2]{};
    float dq[2]{};
    float ddq[2]{};
  };

  struct Limits {
    float q_min[2];
    float q_max[2];
  };

  struct Feedforward {
    float k_acc[2];
  };

  enum class Mode { Disabled, Joint };

  Arm(Motor& joint0, Motor& joint1, const Limits& limits,
      const Feedforward& feedforward);

  bool calibrate(float q0_rad, float q1_rad);
  bool enable();
  void disable();
  bool setReference(const Reference& reference);
  bool addJointReference(float speed0, float speed1, float dt_s = 0.001f);
  void setJointAngleDeg(uint8_t joint, float deg) {
    if (joint > 1 || !trajectoryReady()) return;
    const float q = deg * 0.0174532925f;
    float qq[2] = {reference_.q[0], reference_.q[1]};
    qq[joint] = q;
    if (!withinLimits(qq)) return;   
    reference_.q[joint] = q;
    reference_.dq[joint] = 0.f;
    reference_.ddq[joint] = 0.f;
  }
  void setJointSpeedDps(uint8_t joint, float dps) {
    if (joint > 1 || !trajectoryReady()) return;
    reference_.dq[joint] = dps * 0.0174532925f;
    reference_.ddq[joint] = 0.f;
  }
  void setAccFeedforward(uint8_t joint, float k_acc) {
    if (joint > 1) return;
    feedforward_.k_acc[joint] = k_acc;
  }
  void nudgeReferenceToActual(uint8_t joint, float alpha) {
    if (joint > 1) return;
    reference_.q[joint] += alpha * (state_.q[joint] - reference_.q[joint]);
  }
  bool trajStart(float x_m, float y_m, float duration_s);
  bool trajStartJoint(float q0_rad, float q1_rad, float duration_s);
  bool trajStartCircle(float cx_m, float cy_m, float radius_m, float period_s,
                       int turns = 1);
  bool trajStartSquare(float cx_m, float cy_m, float half_m, float corner_r_m,
                       float duration_s, int dir = 1);
  void trajAbort();
  bool trajRunning() const { return trajectory_type_ != TrajectoryType::None; }
  bool trajFinished() const { return trajectory_finished_; }
  void handle();

  const State& state() const { return state_; }
  const Reference& reference() const { return reference_; }
  Mode mode() const { return mode_; }
  bool calibrated() const { return calibrated_; }
  bool feedbackReady();

 private:
  enum class TrajectoryType { None, Cartesian, Joint, Circle, Path };
  bool trajectoryReady();
  bool withinLimits(const float q[2]) const;
  bool applyReference(const Reference& reference);
  void updateFeedback();
  void trajectoryPlanner();
  void writeMotorCommands();

  Motor& joint0_;
  Motor& joint1_;
  const Limits limits_;
  Feedforward feedforward_;
  State state_{};
  Reference reference_{};
  Mode mode_{Mode::Disabled};
  bool calibrated_{false};
  ArmTrajectory trajectory_{};
  TrajectoryType trajectory_type_{TrajectoryType::None};
  bool trajectory_finished_{false};
  uint32_t trajectory_start_tick_{};
  int trajectory_elbow_{1};
  float joint_trajectory_start_[2]{};
  float joint_trajectory_end_[2]{};
  float joint_trajectory_duration_{};
  float circle_cx_{};
  float circle_cy_{};
  float circle_r_{};
  float circle_omega_{};
  float circle_theta0_{};
  float circle_duration_{};
  float circle_r0_{};
  float circle_blend_{};
  RoundedSquarePath path_{};
  float path_duration_{};
  float path_len_{};
};

#endif
