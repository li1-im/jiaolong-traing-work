#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H

struct CartesianReference {
  float x{}, y{};
  float dx{}, dy{};
  float ddx{}, ddy{};
};

class ArmKinematics {
 public:
  static constexpr float  l0 = 0.105f;
  static constexpr float l1 = 0.100f;
  static constexpr float min_sin_q1 = 0.01f;

  static CartesianReference forward(const float q[2]);
  static bool lineReachable(const CartesianReference& start,
                            const CartesianReference& end);
  static bool inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]);
  static bool derivatives(const CartesianReference& point, const float q[2],
                          float dq[2], float ddq[2]);
};

#endif
