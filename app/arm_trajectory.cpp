// Created by zyfdx on 2026/10/30.

#include "arm_trajectory.h"

#include <cmath>

static float timeRatio(float elapsed_s, float duration_s) {
  float tau = elapsed_s / duration_s;
  if (tau < 0.f) tau = 0.f;
  if (tau > 1.f) tau = 1.f;
  return tau;
}

bool ArmTrajectory::start(const CartesianReference& start,
                          const CartesianReference& end, float duration_s) {
  return this->start(start, end, duration_s, 0.f, 0.f);
}

bool ArmTrajectory::start(const CartesianReference& start,
                          const CartesianReference& end, float duration_s,
                          float v0x, float v0y) {
  if (duration_s <= 0.f) {
    return false;
  }
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  v0x_ = v0x;
  v0y_ = v0y;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
  CartesianReference point{};
  if (!running_) {
    return point;
  }
  float vel_x = 0.f, vel_y = 0.f, acc_x = 0.f, acc_y = 0.f;
  interpolateV(start_.x, end_.x, v0x_, elapsed_s, duration_s_, point.x, vel_x, acc_x);
  interpolateV(start_.y, end_.y, v0y_, elapsed_s, duration_s_, point.y, vel_y, acc_y);
  point.dx = vel_x;
  point.dy = vel_y;
  point.ddx = acc_x;
  point.ddy = acc_y;
  if (elapsed_s >= duration_s_) {
    running_ = false;
  }
  return point;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s,
                                float duration_s, float& position,
                                float& velocity, float& acceleration) {
  position = 0.f;
  velocity = 0.f;
  acceleration = 0.f;
  if (duration_s <= 0.f) {
    position = end;
    return;
  }
  float tau = timeRatio(elapsed_s, duration_s);
  float span = end - start;
  float tau2 = tau * tau;
  float tau3 = tau2 * tau;
  float tau4 = tau2 * tau2;
  position = start + span * (10.f * tau3 - 15.f * tau4 + 6.f * tau3 * tau2);
  velocity =
      span * (30.f * tau2 - 60.f * tau3 + 30.f * tau4) / duration_s;
  acceleration =
      span * (60.f * tau - 180.f * tau2 + 120.f * tau3) / (duration_s * duration_s);
}

void ArmTrajectory::abort() { running_ = false; }

void ArmTrajectory::interpolateV(float start, float end, float v0,
                                 float elapsed_s, float duration_s,
                                 float& position, float& velocity,
                                 float& acceleration) {
  position = start;
  velocity = 0.f;
  acceleration = 0.f;
  if (duration_s <= 0.f) {
    position = end;
    return;
  }
  const float tau = timeRatio(elapsed_s, duration_s);
  const float span = end - start;
  const float A = (fabsf(span) < 1e-9f) ? 0.f : v0 * duration_s / span;
  const float a1 = A;
  const float a3 = 10.f - 6.f * A;
  const float a4 = 8.f * A - 15.f;
  const float a5 = 6.f - 3.f * A;
  const float t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
  const float u = a1 * tau + a3 * t3 + a4 * t4 + a5 * t5;
  const float du = a1 + 3.f * a3 * t2 + 4.f * a4 * t3 + 5.f * a5 * t4;
  const float ddu = 6.f * a3 * tau + 12.f * a4 * t2 + 20.f * a5 * t3;
  position = start + span * u;
  velocity = span * du / duration_s;
  acceleration = span * ddu / (duration_s * duration_s);
}
