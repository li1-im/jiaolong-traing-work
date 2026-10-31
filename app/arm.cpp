//
// Created by zyfdx on 2026/10/30.
//

#include "arm.h"

#include "main.h"  // HAL_GetTick()：毫秒时基，给轨迹计时用
#include "base/common/math.h"  // 角度制/弧度制换算
#include <math.h>              // sqrtf/atan2f/cosf/sinf（圆轨迹解析采样）

// 构造：机械臂只是“引用”两个电机 + 记住限位和前馈系数
// 引用成员必须在初始化列表里绑定，不能在函数体里赋值
Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits), feedforward_(feedforward) {
}

// 反馈是否可用：两个电机的通信都要在线（CAN 心跳超时会自动断开）
bool Arm::feedbackReady() {
  return joint0_.connect_.check() && joint1_.connect_.check();
}

// 关节角是否都在限位内（任务书：±10 圈 = ±3600°，robot.cpp 已换算成弧度传入）
bool Arm::withinLimits(const float q[2]) const {
  for (int i = 0; i < 2; i++) {
    if (q[i] < limits_.q_min[i] || q[i] > limits_.q_max[i]) {
      return false;
    }
  }
  return true;
}

// 轨迹 / 手动控制的前置条件：已使能（Joint 模式）且电机反馈在线
bool Arm::trajectoryReady() {
  return mode_ == Mode::Joint && feedbackReady();
}

// 零位标定：只有“失能 + 反馈在线”才允许
// 含义：把电机当前的累计反馈角度直接“改写”成已知的摆好姿态 q0/q1（弧度）
// 注意 resetFeedbackAngle 改的是电机的“控制反馈角”，不影响电机实际位置
bool Arm::calibrate(float q0_rad, float q1_rad) {
  if (mode_ != Mode::Disabled) {
    return false;
  }
  float q[2] = {q0_rad, q1_rad};
  if (!feedbackReady() || !withinLimits(q)) {
    return false;
  }
  // 电机层统一用“度”，机械臂内部统一用“弧度”，这里做换算后写入电机
  joint0_.resetFeedbackAngle(math::rad2deg(q0_rad));
  joint1_.resetFeedbackAngle(math::rad2deg(q1_rad));
  // 同步内部状态和参考：标定完，机械臂认为自己就在这姿态上
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

// 使能：以“当前实际位置”为目标起步，之后两个电机进入 WORKING 闭环
bool Arm::enable() {
  if (!feedbackReady()) {
    return false;
  }
  updateFeedback();  // 先读一次真实位置
  reference_ = {};   // 清零速度/加速度前馈
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  // 使能时清掉一切旧轨迹状态
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  mode_ = Mode::Joint;
  joint0_.mode_ = Motor::WORKING;
  joint1_.mode_ = Motor::WORKING;
  return true;
}

// 失能：断电，所有轨迹作废
void Arm::disable() {
  mode_ = Mode::Disabled;
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  joint0_.mode_ = Motor::POWEROFF;
  joint1_.mode_ = Motor::POWEROFF;
}

// 外部写入位置参考：轨迹正在跑时一律拒绝（防止手动信号把轨迹目标顶飞）
bool Arm::setReference(const Reference& reference) {
  if (trajRunning()) {
    return false;
  }
  return applyReference(reference);
}

// 真正落笔写参考：只在 Joint 模式、且不越限时接受
bool Arm::applyReference(const Reference& reference) {
  if (mode_ != Mode::Joint) {
    return false;
  }
  if (!withinLimits(reference.q)) {
    return false;
  }
  reference_ = reference;  // 结构体赋值，q/dq/ddq 数组一起拷贝
  return true;
}

// 手动关节速度积分：把“摇杆速度指令”积分成“位置目标”
// ref.q += speed*dt —— 摇杆不动时 dt 再多次加，目标原地不动 = 保持当前位置
// dq 填速度做前馈：让内环速度环“提前知道”期望速度，跟踪更跟手
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
  // 越限则 applyReference 返回 false 且不写参考（目标被“顶”在限位处）
  return applyReference(ref);
}

// 笛卡尔轨迹：从现在的“规划目标”(reference_.q 的末端位置)直线走到 (x_m, y_m)
// 因为上一段结束时 reference_.q 就是段终点，各段自动首尾相接（参考连续）
// 肘支 trajectory_elbow_ 与关节累计圈数（inverse 内部展开）都被沿用
bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (!trajectoryReady()) {
    return false;
  }
  CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference target{};
  target.x = x_m;
  target.y = y_m;
  // 任务书要求对“整条线段”做可达性检查，只检查端点是不够的
  if (!ArmKinematics::lineReachable(start, target)) {
    return false;
  }
  float q[2];
  // 用当前规划目标当 previous_q：解会展开到当前圈数附近，角度不跳变
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

// 关节空间同步点到点：从“实际反馈位置”出发，两关节同时按五次曲线走到目标累计角
// 起止都用弧度；两轴共用同一时长 => 同时到位（同步）
bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
  if (!trajectoryReady()) {
    return false;
  }
  float q_start[2] = {state_.q[0], state_.q[1]};
  float q_end[2] = {q0_rad, q1_rad};
  if (!withinLimits(q_start) || !withinLimits(q_end)) {
    return false;
  }
  // ArmTrajectory 仍然 start 一份（管时长/运行标志），关节轨迹只是不用它的采样
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

// 中途取消：停止推进；位置参考保持在“最后写出的那个点”，速度/加速度前馈清零
// （不清零的话电机会带着一个过期的速度前馈继续冲）
void Arm::trajAbort() {
  trajectory_.abort();
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;
}

// 把电机累计反馈（度、度/秒）换算进机械臂状态（弧度、弧度/秒）
// realAngle() 是电机层已经解算 + 展开圈数 + 按传动比定好方向的累计角度，
// 方向与关节坐标系一致，机械臂这层只需要做度↔弧度换算，不用再处理传动比
void Arm::updateFeedback() {
  state_.q[0] = math::deg2rad(joint0_.realAngle());
  state_.q[1] = math::deg2rad(joint1_.realAngle());
  state_.dq[0] = math::deg2rad(joint0_.realSpeed());
  state_.dq[1] = math::deg2rad(joint1_.realSpeed());
}

// ===== 可选图形：正圆（任务书 §5.3 更多图形）=====
// 位置/速度/加速度全部按圆的解析式给出（不是数值差分），所以 dq/ddq 前馈精确；
// 每 1 ms 重新逆解一次，圈数由 previous_q 连续展开。
// 可达性：圆上各点到原点的距离范围是 [|d_oc-R|, d_oc+R]（d_oc = 圆心到原点距离），
// 必须整体落在环带 [|l1-l0|, l0+l1] = [0.07, 0.16] m 内。
// 特别地：圆以原点为心时 d_oc=0，末端半径恒为 R，只要 R 在环带内就完全可行
//（此时只有 J0 转动、J1 保持 —— 圆度最好，也是评价标准里的"正圆"画法）。
bool Arm::trajStartCircle(float cx_m, float cy_m, float radius_m, float period_s,
                          int turns) {
  if (!trajectoryReady()) {
    return false;  // 未使能 / 反馈掉线
  }
  if (radius_m <= 0.f || period_s <= 0.f || turns < 1) {
    return false;
  }
  const float d_oc = sqrtf(cx_m * cx_m + cy_m * cy_m);  // 圆心到原点距离
  const float d_max = d_oc + radius_m;
  const float d_min = fabsf(d_oc - radius_m);  // ← 圆心在原点时 = R，不是 0
  if (d_max > ArmKinematics::l0 + ArmKinematics::l1 ||
      d_min < fabsf(ArmKinematics::l1 - ArmKinematics::l0)) {
    return false;  // 整圆越出可达环带（内死区/外边界）
  }
  // 起点相位对齐当前规划末端 => 位置无跳变，从当前姿态平滑接入
  const CartesianReference now = ArmKinematics::forward(reference_.q);
  circle_cx_ = cx_m;
  circle_cy_ = cy_m;
  circle_r_ = radius_m;
  circle_omega_ = 6.2831853f / period_s;          // rad/s
  circle_theta0_ = atan2f(now.y - cy_m, now.x - cx_m);
  circle_duration_ = period_s * (float)turns;     // s
  trajectory_.abort();                            // 直线缓存不再使用
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Circle;      // 肘支沿用上一段
  return true;
}

// 轨迹规划：每个 1 ms 周期算出“这一毫秒末端/关节应该在哪、多快、多加速度”
// 注意起点是上一个周期写好的参考，天生连续；时间到 = 正常结束（trajFinished）
void Arm::trajectoryPlanner() {
  if (trajectory_type_ == TrajectoryType::None) {
    return;
  }
  // 无符号减法自动处理 HAL_GetTick 溢出回绕；毫秒 → 秒
  float elapsed_s = (float)(HAL_GetTick() - trajectory_start_tick_) * 0.001f;

  Reference ref;
  if (trajectory_type_ == TrajectoryType::Cartesian) {
    // ---- 笛卡尔直线：采样直线上的点，再做逆解换成关节目标 ----
    CartesianReference point = trajectory_.sample(elapsed_s);
    float q[2];
    // 逆解用“上一周期的参考角”展开圈数 + 固定肘支 => 全程唯一连续解
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, q)) {
      trajAbort();  // 中途进奇异/出工作空间：原地保持，安全优先
      return;
    }
    if (!withinLimits(q)) {
      trajAbort();
      return;
    }
    ref.q[0] = q[0];
    ref.q[1] = q[1];
    // 解析速度/加速度（进阶项）：雅可比求逆映射；奇异附近退回 0 防止爆值
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
    // sample() 发现 elapsed >= 时长会把 running_ 置 false：这就是“正常走完”
    if (!trajectory_.running()) {
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
  } else if (trajectory_type_ == TrajectoryType::Circle) {
    // ---- 正圆：位置/速度/加速度全部解析给出 ----
    const float theta = circle_theta0_ + circle_omega_ * elapsed_s;
    const float ct = cosf(theta);
    const float st = sinf(theta);
    CartesianReference point;
    point.x = circle_cx_ + circle_r_ * ct;
    point.y = circle_cy_ + circle_r_ * st;
    point.dx = -circle_r_ * circle_omega_ * st;
    point.dy = circle_r_ * circle_omega_ * ct;
    point.ddx = -circle_r_ * circle_omega_ * circle_omega_ * ct;
    point.ddy = -circle_r_ * circle_omega_ * circle_omega_ * st;
    float q[2];
    if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, q)) {
      trajAbort();  // 中途进奇异/出工作空间：原地保持
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
  } else {
    // ---- 关节空间点到点：两个关节各自同一条五次曲线（共用时长=同步到位） ----
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

// 把参考翻译成电机指令：目标角度（度）+ 速度前馈（度/秒）+ 力矩前馈
// k_acc 单位是 “输出指令 / (rad/s²)”：feedforward_ 在 robot.cpp 里默认 0，
// 想做动力学前馈时只需要改那个系数，本函数完全不用动
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

// 机械臂主处理：controlLoop() 里每 1 ms 调一次，且在 allMotorsHandle() 之前
// 顺序：读反馈 → 规划轨迹（刷新 reference_）→ 写电机指令
// 失能状态下只更新反馈，不规划也不写指令（电机此时是 POWEROFF，写了也没输出）
void Arm::handle() {
  updateFeedback();
  if (mode_ != Mode::Joint) {
    return;
  }
  trajectoryPlanner();
  writeMotorCommands();
}
