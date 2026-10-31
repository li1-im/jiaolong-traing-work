//
// Created by zyfdx on 2026/10/30.
//

#include "arm_kinematics.h"

#include <cmath>

// 两杆完全伸直时的最远 reach，和两杆折满时的最近 reach
// 工作空间是一个 “圆环”：内径 |l0-l1| = 0.07 m，外径 l0+l1 = 0.16 m
static constexpr float kReachOuter =
    ArmKinematics::l0 + ArmKinematics::l1;
static constexpr float kReachInner =
    (ArmKinematics::l0 > ArmKinematics::l1 ? ArmKinematics::l0 - ArmKinematics::l1
                                           : ArmKinematics::l1 - ArmKinematics::l0);
// 数值容差：float 计算误差 1e-4 m 级别，判可达时留一点余量
static constexpr float kEps = 1e-4f;
static constexpr float kTwoPi = 6.2831853f;

// 正运动学：由两个关节角算出末端坐标
// q[0] = J0 第一杆角度（相对 +X 轴），q[1] = J1 第二杆相对第一杆的角度
// 向量三角形：原点 → 第一杆末端 → 末端发光点，两段向量直接相加
//   x = l0*cos(q0) + l1*cos(q0+q1)
//   y = l0*sin(q0) + l1*sin(q0+q1)
CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference point{};
  float s0 = sinf(q[0]), c0 = cosf(q[0]);
  // 第二杆的绝对角度是 q0 + q1（q1 是相对第一杆转的）
  float s01 = sinf(q[0] + q[1]), c01 = cosf(q[0] + q[1]);
  point.x = l0 * c0 + l1 * c01;
  point.y = l0 * s0 + l1 * s01;
  // forward 只负责位置，速度/加速度字段保持 0，由调用方或 derivatives() 给出
  return point;
}

// 逆运动学：给定点 (x, y)，解出两个关节角
// previous_q：上一周期的关节角（累计值），用于把解“展开”到它附近，保证圈数连续
// elbow：+1 正肘（第二杆向逆时针方向折），-1 逆肘；同一点两解就是常说的“肘上/肘下”
bool ArmKinematics::inverse(const CartesianReference& point,
                            const float previous_q[2], int elbow, float q[2]) {
  // 余弦定理：cos(q1) = (r² − l0² − l1²) / (2 l0 l1)，r 是点到原点的距离
  float r2 = point.x * point.x + point.y * point.y;
  float r = sqrtf(r2);
  // 目标在圆环工作空间之外就无解
  if (r > kReachOuter + kEps || r < kReachInner - kEps) {
    return false;
  }
  float cos_q1 = (r2 - l0 * l0 - l1 * l1) / (2.f * l0 * l1);
  // acos 的输入必须在 [-1, 1]，把边缘的浮点误差夹回来
  if (cos_q1 > 1.f) cos_q1 = 1.f;
  if (cos_q1 < -1.f) cos_q1 = -1.f;
  // 同一条 cos 对应两个 sin（±），正好对应肘上 / 肘下两种姿势
  float sin_q1 = sqrtf(1.f - cos_q1 * cos_q1);
  if (elbow < 0) {
    sin_q1 = -sin_q1;
  }
  float q1 = atan2f(sin_q1, cos_q1);
  // q0：目标方向角 atan2(y, x)，减去第一杆需要“让开”的角度
  //     atan2(l1*sin(q1), l0 + l1*cos(q1)) 是向量三角形的偏置角
  float q0 = atan2f(point.y, point.x) - atan2f(l1 * sin_q1, l0 + l1 * cos_q1);
  // 上面用 atan2 得到的 q0/q1 都被压回 (-pi, pi]。
  // 机械臂可能已经转了好几圈（累计角），要把解加上最近的整数圈，
  // 让它贴到 previous_q 附近，这样关节角不会每圈“跳”回 0。
  // roundf(x) 取最近整数：2π × round((previous − q) / 2π) 就是“最近的整圈数”
  q[0] = q0 + kTwoPi * roundf((previous_q[0] - q0) / kTwoPi);
  q[1] = q1 + kTwoPi * roundf((previous_q[1] - q1) / kTwoPi);
  return true;
}

// 判断整条直线段是否都在工作空间圆环内
// 点到原点的距离 |s + t*(e − s)| 是凸函数：最小值只可能在
//   “原点到线段的垂足”（垂足在段内）或两个端点处取得；最大值一定在端点处。
// 因此只要检查 dmin ≥ 内径 且 dmax ≤ 外径，整条线段就都可达。
bool ArmKinematics::lineReachable(const CartesianReference& start,
                                  const CartesianReference& end) {
  float d0 = sqrtf(start.x * start.x + start.y * start.y);
  float d1 = sqrtf(end.x * end.x + end.y * end.y);
  float dmax = d0 > d1 ? d0 : d1;
  float ex = end.x - start.x, ey = end.y - start.y;
  float len2 = ex * ex + ey * ey;
  float dmin;
  if (len2 < 1e-12f) {
    dmin = d0;  // 退化成一点
  } else {
    // 垂足参数 t = −(s·e')/|e'|²，夹到 [0, 1] 就是线段上离原点最近的点
    float t = -(start.x * ex + start.y * ey) / len2;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    float px = start.x + t * ex, py = start.y + t * ey;
    dmin = sqrtf(px * px + py * py);
  }
  return dmin >= kReachInner - kEps && dmax <= kReachOuter + kEps;
}

// 速度 / 加速度映射：已知末端轨迹点的位置 + 速度(dx,dy) + 加速度(ddx,ddy)
// 和当前位置 q，解出关节速度 dq 和关节加速度 ddq
// 微分关系：  [dx; dy]      = J · [dq0; dq1]
//            [ddx; ddy]    = J · [ddq0; ddq1] + dJ/dt · [dq0; dq1]
// J 是 2x2 雅可比矩阵，det(J) = l0·l1·sin(q1)；q1 接近 0（伸直）时 det→0 奇异，
// 无法求逆，用 min_sin_q1 做阈值，奇异直接返回 false。
bool ArmKinematics::derivatives(const CartesianReference& point,
                                const float q[2], float dq[2], float ddq[2]) {
  dq[0] = dq[1] = 0.f;
  ddq[0] = ddq[1] = 0.f;
  float s0 = sinf(q[0]), c0 = cosf(q[0]);
  float s1 = sinf(q[1]);
  float s01 = sinf(q[0] + q[1]), c01 = cosf(q[0] + q[1]);
  // 雅可比四个元素（对 x = l0 c0 + l1 c01、y = l0 s0 + l1 s01 求偏导）
  float j11 = -l0 * s0 - l1 * s01;  // dx / dq0
  float j12 = -l1 * s01;            // dx / dq1
  float j21 = l0 * c0 + l1 * c01;   // dy / dq0
  float j22 = l1 * c01;             // dy / dq1
  float det = j11 * j22 - j12 * j21;  // = l0*l1*sin(q1)
  // |sin(q1)| 小于阈值认为两杆接近伸直，处于奇异位置
  if (fabsf(s1) < min_sin_q1) {
    return false;
  }
  // 2x2 逆矩阵直接手算： J⁻¹ = 1/det · [j22, −j12; −j21, j11]
  dq[0] = (j22 * point.dx - j12 * point.dy) / det;
  dq[1] = (-j21 * point.dx + j11 * point.dy) / det;
  // 向心项 dJ/dt·dq：J 里的角度随时间转，对时间再求导多出来的这一坨
  // 对 x = l0 c0 + l1 c01、y = l0 s0 + l1 s01 求两次导数即可得到：
  //   cent_x = −l0 c0 ω0² − l1 c01 (ω0+ω1)²
  //   cent_y = −l0 s0 ω0² − l1 s01 (ω0+ω1)²
  float dq_sum = dq[0] + dq[1];
  float cent0 = -l0 * c0 * dq[0] * dq[0] - l1 * c01 * dq_sum * dq_sum;
  float cent1 = -l0 * s0 * dq[0] * dq[0] - l1 * s01 * dq_sum * dq_sum;
  // ddq = J⁻¹ · (末端加速度 − 向心项)
  ddq[0] = (j22 * (point.ddx - cent0) - j12 * (point.ddy - cent1)) / det;
  ddq[1] = (-j21 * (point.ddx - cent0) + j11 * (point.ddy - cent1)) / det;
  return true;
}
