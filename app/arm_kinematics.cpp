// Created by zyfdx on 2026/10/30.

#include "arm_kinematics.h"

#include <cmath>

static constexpr float kReachOuter =
    ArmKinematics::l0 + ArmKinematics::l1;
static constexpr float kReachInner =
    (ArmKinematics::l0 > ArmKinematics::l1 ? ArmKinematics::l0 - ArmKinematics::l1
                                           : ArmKinematics::l1 - ArmKinematics::l0);
static constexpr float kEps = 1e-4f;
static constexpr float kTwoPi = 6.2831853f;

CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference point{};
  float s0 = sinf(q[0]), c0 = cosf(q[0]);
  float s01 = sinf(q[0] + q[1]), c01 = cosf(q[0] + q[1]);
  point.x = l0 * c0 + l1 * c01;
  point.y = l0 * s0 + l1 * s01;
  return point;
}

bool ArmKinematics::inverse(const CartesianReference& point,
                            const float previous_q[2], int elbow, float q[2]) {
  float r2 = point.x * point.x + point.y * point.y;
  float r = sqrtf(r2);
  if (r > kReachOuter + kEps || r < kReachInner - kEps) {
    return false;
  }
  float cos_q1 = (r2 - l0 * l0 - l1 * l1) / (2.f * l0 * l1);
  if (cos_q1 > 1.f) cos_q1 = 1.f;
  if (cos_q1 < -1.f) cos_q1 = -1.f;
  float sin_q1 = sqrtf(1.f - cos_q1 * cos_q1);
  if (elbow < 0) {
    sin_q1 = -sin_q1;
  }
  float q1 = atan2f(sin_q1, cos_q1);
  float q0 = atan2f(point.y, point.x) - atan2f(l1 * sin_q1, l0 + l1 * cos_q1);
  q[0] = q0 + kTwoPi * roundf((previous_q[0] - q0) / kTwoPi);
  q[1] = q1 + kTwoPi * roundf((previous_q[1] - q1) / kTwoPi);
  return true;
}

bool ArmKinematics::lineReachable(const CartesianReference& start,
                                  const CartesianReference& end) {
  float d0 = sqrtf(start.x * start.x + start.y * start.y);
  float d1 = sqrtf(end.x * end.x + end.y * end.y);
  float dmax = d0 > d1 ? d0 : d1;
  float ex = end.x - start.x, ey = end.y - start.y;
  float len2 = ex * ex + ey * ey;
  float dmin;
  if (len2 < 1e-12f) {
    dmin = d0;
  } else {
    float t = -(start.x * ex + start.y * ey) / len2;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    float px = start.x + t * ex, py = start.y + t * ey;
    dmin = sqrtf(px * px + py * py);
  }
  return dmin >= kReachInner - kEps && dmax <= kReachOuter + kEps;
}

bool ArmKinematics::derivatives(const CartesianReference& point,
                                const float q[2], float dq[2], float ddq[2]) {
  dq[0] = dq[1] = 0.f;
  ddq[0] = ddq[1] = 0.f;
  float s0 = sinf(q[0]), c0 = cosf(q[0]);
  float s1 = sinf(q[1]);
  float s01 = sinf(q[0] + q[1]), c01 = cosf(q[0] + q[1]);
  float j11 = -l0 * s0 - l1 * s01;
  float j12 = -l1 * s01;
  float j21 = l0 * c0 + l1 * c01;
  float j22 = l1 * c01;
  float det = j11 * j22 - j12 * j21;
  // 两杆快伸直或快重合时逆解不可靠，return false
  if (fabsf(s1) < min_sin_q1) {
    return false;
  }
  dq[0] = (j22 * point.dx - j12 * point.dy) / det;
  dq[1] = (-j21 * point.dx + j11 * point.dy) / det;
  float dq_sum = dq[0] + dq[1];
  float cent0 = -l0 * c0 * dq[0] * dq[0] - l1 * c01 * dq_sum * dq_sum;
  float cent1 = -l0 * s0 * dq[0] * dq[0] - l1 * s01 * dq_sum * dq_sum;
  ddq[0] = (j22 * (point.ddx - cent0) - j12 * (point.ddy - cent1)) / det;
  ddq[1] = (-j21 * (point.ddx - cent0) + j11 * (point.ddy - cent1)) / det;
  return true;
}
