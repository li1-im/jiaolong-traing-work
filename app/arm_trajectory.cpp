//
// Created by zyfdx on 2026/10/30.
//

#include "arm_trajectory.h"

// 时间比例 τ = elapsed / duration，限制在 [0, 1]
// 小白提示：先把“走了全程的百分之多少”算出来，后面所有公式都只用这个 τ
static float timeRatio(float elapsed_s, float duration_s) {
  float tau = elapsed_s / duration_s;
  if (tau < 0.f) tau = 0.f;
  if (tau > 1.f) tau = 1.f;
  return tau;
}

// 启动一段“起点静止 → 终点静止”的直线插值
bool ArmTrajectory::start(const CartesianReference& start,
                          const CartesianReference& end, float duration_s) {
  // 时长必须是正数，否则后面除以时长会变成除零
  if (duration_s <= 0.f) {
    return false;
  }
  // 把起点和终点完整保存下来（位置 dx/dy/ddx/ddy 都拷，结构体直接赋值最省事）
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  running_ = true;
  return true;
}

// 采样：给出“轨迹启动后经过的秒数”，返回这一时刻末端的位置、速度、加速度
CartesianReference ArmTrajectory::sample(float elapsed_s) {
  CartesianReference point{};
  // 轨迹没在跑就直接返回零参考，调用方不应使用这个返回值
  if (!running_) {
    return point;
  }
  // x、y 两个方向各自做一次“两端静止”的五次多项式插值
  // 直线 = 两个坐标轴各自独立地平滑移动，合成出来就是直线段
  float vel_x = 0.f, vel_y = 0.f, acc_x = 0.f, acc_y = 0.f;
  interpolate(start_.x, end_.x, elapsed_s, duration_s_, point.x, vel_x, acc_x);
  interpolate(start_.y, end_.y, elapsed_s, duration_s_, point.y, vel_y, acc_y);
  point.dx = vel_x;
  point.dy = vel_y;
  point.ddx = acc_x;
  point.ddy = acc_y;
  // 时间走完：把轨迹标记为结束，返回值恰好是终点
  // Arm::trajectoryPlanner() 会检测 running() 变 false，从而置 trajFinished()
  if (elapsed_s >= duration_s_) {
    running_ = false;
  }
  return point;
}

// 两端静止的单轴五次插值（位置或关节角通用）
// 归一化五次多项式 s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ 满足：
//   s(0)=0, s(1)=1，且 s'(0)=s'(1)=0、s''(0)=s''(1)=0
// 也就是起点和终点的位置、速度、加速度全部对得上，衔接处不会“顿挫”
void ArmTrajectory::interpolate(float start, float end, float elapsed_s,
                                float duration_s, float& position,
                                float& velocity, float& acceleration) {
  position = 0.f;
  velocity = 0.f;
  acceleration = 0.f;
  if (duration_s <= 0.f) {
    // 时长非法时直接跳到终点（防御性写法，start() 已保证时长为正）
    position = end;
    return;
  }
  float tau = timeRatio(elapsed_s, duration_s);
  // 位移缩放系数：全程长度 (end - start)，多项式只负责“形状”
  float span = end - start;
  // 链式法则：对时间求导 = 对 τ 求导 × dτ/dt = 对 τ 求导 / duration
  float tau2 = tau * tau;
  float tau3 = tau2 * tau;
  float tau4 = tau2 * tau2;
  position = start + span * (10.f * tau3 - 15.f * tau4 + 6.f * tau3 * tau2);
  velocity =
      span * (30.f * tau2 - 60.f * tau3 + 30.f * tau4) / duration_s;
  acceleration =
      span * (60.f * tau - 180.f * tau2 + 120.f * tau3) / (duration_s * duration_s);
}

// 中途取消：不再推进，调用方（Arm）会保持最后一次位置参考
void ArmTrajectory::abort() { running_ = false; }
