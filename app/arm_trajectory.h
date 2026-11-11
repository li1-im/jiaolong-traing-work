#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_TRAJECTORY_H

#include "arm_kinematics.h"

class ArmTrajectory {
 public:
  bool start(const CartesianReference& start, const CartesianReference& end,
             float duration_s);
  bool start(const CartesianReference& start, const CartesianReference& end,
             float duration_s, float v0x, float v0y);
  CartesianReference sample(float elapsed_s);
  static void interpolate(float start, float end, float elapsed_s,
                          float duration_s, float& position, float& velocity,
                          float& acceleration);
  static void interpolateV(float start, float end, float v0, float elapsed_s,
                           float duration_s, float& position,
                           float& velocity, float& acceleration);
  void abort();
  bool running() const { return running_; }

 private:
  CartesianReference start_{}, end_{};
  float duration_s_{};
  float v0x_{0.f}, v0y_{0.f};
  bool running_{false};
};

#endif
