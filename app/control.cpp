/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"
#include "app/arm.h"

#include "iwdg.h"

// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
// #include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"
#include "cmath"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

#ifdef RF206S
extern WFLY rc;
#elif defined(DR16)
extern RC rc;
#endif

volatile float g_deg_ref0 = 0.f;
volatile float g_deg_ref1 = 0.f;
volatile float g_deg_fdb0 = 0.f;
volatile float g_deg_fdb1 = 0.f;
volatile float g_err0 = 0.f;
volatile float g_err1 = 0.f;

extern Arm arm;
BoardLed led;

enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
bool robot_init_flag = false;
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif

uint8_t arm_step = 0;
volatile bool arm_start_ok = false;
static uint16_t settle_ticks = 0;
static uint16_t runaway_ticks = 0;

volatile uint8_t g_dbg_state = 0;
volatile uint8_t g_dbg_arm_mode = 0;
volatile uint8_t g_dbg_fdb_ok = 0;
volatile uint8_t g_dbg_traj = 0;
volatile uint8_t g_dbg_blend = 0;
volatile uint8_t g_dbg_step = 0;
volatile uint8_t g_dbg_runaway = 0;
static float corner_boost0 = 0.f;
static constexpr float kBreakawayCounts = 0.f;
volatile uint8_t g_reset_pos = 0;
volatile uint8_t g_dbg_resets = 0;
static constexpr float kRunawayDeg = 45.f;
volatile float g_dbg_xref = 0.f, g_dbg_yref = 0.f;
volatile float g_dbg_xfdb = 0.f, g_dbg_yfdb = 0.f;
volatile float g_dbg_qref0 = 0.f, g_dbg_qref1 = 0.f;
volatile float g_dbg_qfdb0 = 0.f, g_dbg_qfdb1 = 0.f;

volatile uint8_t g_test_enable = 0;
volatile uint8_t g_test_joint = 0;
volatile float g_test_angle_deg = 0.f;

volatile uint8_t g_test_speed_enable = 0;
volatile float g_test_speed_dps = 0.f;

volatile float g_ff0p_c[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_ff0p_s[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_ff0n_c[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_ff0n_s[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_ff1_c[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_ff1_s[4] = {0.f, 0.f, 0.f, 0.f};
volatile float g_break0 = 0.f;
static float sq_dx = 0.f, sq_dy = 0.f;
volatile float g_harm0_c = 0.f, g_harm0_s = 0.f;
volatile float g_harm1_c = 0.f, g_harm1_s = 0.f;

volatile float g_fric0 = 440.f;
volatile float g_fric1 = 400.f;

volatile float g_kacc0 = 0.f;
volatile float g_kacc1 = 0.f;
volatile float g_grav0 = 0.f;
volatile float g_grav1 = 0.f;

// 控制初始化
void controlInit(void) {
  (RC_UART)->Init.BaudRate = 100000;
  (RC_UART)->Init.WordLength = UART_WORDLENGTH_9B;
#ifdef RF206S
  (RC_UART)->Init.StopBits = UART_STOPBITS_2;
#endif
#ifdef DR16
  (RC_UART)->Init.StopBits = UART_STOPBITS_1;
#endif
  (RC_UART)->Init.Parity = UART_PARITY_EVEN;
  (RC_UART)->Init.Mode = UART_MODE_RX;
  (RC_UART)->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  (RC_UART)->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(RC_UART) != HAL_OK)
  {
      Error_Handler();
  }
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == decltype(rc)::DOWN);

  if (robot_state == STOP) {
    robotReset();
  } else if (robot_state == STARTUP) {
    robot_init_flag = robotStartup();
  } else if (robot_state == WORKING) {
    robotControl();
  }
  {
    static uint8_t reset_prev = 0;
    if (g_reset_pos != 0 && reset_prev == 0) {
      arm.trajAbort();
      arm_step = 0;
      arm_start_ok = false;
      settle_ticks = 0;
      runaway_ticks = 0;
      arm.setJointAngleDeg(0, arm.state().q[0] * 57.29578f);
      arm.setJointAngleDeg(1, arm.state().q[1] * 57.29578f);
      ++g_dbg_resets;
      g_reset_pos = 0;
    }
    reset_prev = g_reset_pos;
  }

  arm.handle();

  g_dbg_state = (uint8_t)robot_state;
  g_dbg_arm_mode = (uint8_t)arm.mode();
  g_dbg_fdb_ok = arm.feedbackReady() ? 1 : 0;
  g_dbg_traj = arm.trajRunning() ? 1 : 0;
  g_dbg_step = arm_step;
  {
    const CartesianReference pr = ArmKinematics::forward(arm.reference().q);
    const CartesianReference pf = ArmKinematics::forward(arm.state().q);
    g_dbg_xref = pr.x;
    g_dbg_yref = pr.y;
    g_dbg_xfdb = pf.x;
    g_dbg_yfdb = pf.y;
    g_dbg_qref0 = arm.reference().q[0] * 57.29578f;
    g_dbg_qref1 = arm.reference().q[1] * 57.29578f;
    g_dbg_qfdb0 = arm.state().q[0] * 57.29578f;
    g_dbg_qfdb1 = arm.state().q[1] * 57.29578f;
  }

  arm.setAccFeedforward(0, g_kacc0);
  arm.setAccFeedforward(1, g_kacc1);
  {
    const float v0 = arm.reference().dq[0] * 57.29578f;
    const float v1 = arm.reference().dq[1] * 57.29578f;
    float x0 = v0 / 5.f, x1 = v1 / 5.f;
    if (x0 > 1.f) x0 = 1.f;  if (x0 < -1.f) x0 = -1.f;
    if (x1 > 1.f) x1 = 1.f;  if (x1 < -1.f) x1 = -1.f;
    const float q0 = arm.state().q[0];
    const float q1 = arm.state().q[1];
    static float ff_lp[2] = {0.f, 0.f};
    const float kAlpha = 0.001f / 0.030f;
    auto ffOfAngle = [](const volatile float* c, const volatile float* s, float qdeg) {
      const float q = qdeg * 0.017453292f;
      float v = c[0];
      for (int k = 1; k <= 3; ++k) {
        v += c[k] * cosf(k * q) + s[k] * sinf(k * q);
      }
      return v;
    };
    const float q0deg = q0 * 57.29578f;
    const float q1deg = (q0 + q1) * 57.29578f;
    const bool fwd = (arm.reference().dq[0] >= 0.f);
    const float ff_dir0 = fwd ? ffOfAngle(g_ff0p_c, g_ff0p_s, q0deg)
                              : ffOfAngle(g_ff0n_c, g_ff0n_s, q0deg);
    const float kf0 = g_fric0 + g_harm0_c * cosf(q0) + g_harm0_s * sinf(q0) + ff_dir0;
    const float kf1 = g_fric1 + g_harm1_c * cosf(q0 + q1) + g_harm1_s * sinf(q0 + q1) +
                      ffOfAngle(g_ff1_c, g_ff1_s, q1deg);
    const float ff0 = kf0 * x0 + g_grav0 * cosf(q0);
    const float ff1 = kf1 * x1 + g_grav1 * cosf(q0 + q1);
    ff_lp[0] += (ff0 - ff_lp[0]) * kAlpha;
    ff_lp[1] += (ff1 - ff_lp[1]) * kAlpha;
    {
      const float v_ref_dps = arm.reference().dq[0] * 57.29578f;
      const float v_act_dps = arm.state().dq[0] * 57.29578f;
      static bool brk_active = false;
      if (!brk_active && fabsf(v_act_dps) < 5.f && fabsf(v_ref_dps) > 3.f) {
        brk_active = true;
      } else if (brk_active && fabsf(v_act_dps) > 25.f) {
        brk_active = false;
      }
      float brk = 0.f;
      if (g_break0 > 0.f && brk_active) {
        brk = (v_ref_dps > 0.f) ? g_break0 : -g_break0;
      }
      J0.control_data_.feedforward_intensity += brk;
    }
    // 前馈写在 arm.handle() 之后，否则被 setAngleSpeed 覆盖
    J0.control_data_.feedforward_intensity += ff_lp[0] + corner_boost0;
    J1.control_data_.feedforward_intensity += ff_lp[1];
  }

  if (!arm.trajRunning()) {
    const float kDeadbandDeg = 0.4f;
    for (uint8_t j = 0; j < 2; ++j) {
      const float err = (arm.reference().q[j] - arm.state().q[j]) * 57.29578f;
      if (fabsf(err) < kDeadbandDeg) {
        arm.nudgeReferenceToActual(j, 0.03f);
      }
    }
  }

  boardLedHandle();

  g_deg_ref0 = arm.reference().q[0] * 57.29578f;
  g_deg_ref1 = arm.reference().q[1] * 57.29578f;
  g_deg_fdb0 = arm.state().q[0] * 57.29578f;
  g_deg_fdb1 = arm.state().q[1] * 57.29578f;
  g_err0 = g_deg_ref0 - g_deg_fdb0;
  g_err1 = g_deg_ref1 - g_deg_fdb1;
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  static bool dbg_freeze_done = false;
  if (!dbg_freeze_done) {
    __HAL_DBGMCU_FREEZE_IWDG();
    dbg_freeze_done = true;
  }
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  arm_step = 0;
  arm_start_ok = false;
  arm.disable();
  last_rc_switch = rc.switch_;
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;

  return flag;
}

static const float kSquare[4][2] = {
  { 0.1f,  0.1f},
  { 0.1f, -0.1f},
  {-0.1f, -0.1f},
  {-0.1f,  0.1f},
};
static constexpr int kSquareReverse = 1;
static constexpr float kPrepDuration = 2.0f;
static constexpr float kEdgeDuration = 3.0f;
static constexpr float kSquareHalf = 0.1f;
static constexpr float kCornerR = 0.012f;
static constexpr float kSettleSpeedRad = 0.26f;
static constexpr float kSettleErrDeg = 2.0f;
static constexpr uint16_t kSettleMaxTicks = 150;
static constexpr float kBlendRadius = 0.0f;
static constexpr float kManualSpeedMax = 1.5f;
static constexpr float kManualDeadBand = 0.3f;

static constexpr float kCircleCx = 0.0f;
static constexpr float kCircleCy = 0.0f;
static constexpr float kCircleR = 0.10f;
static constexpr float kCirclePeriod = 2.0f;
static constexpr int kCircleTurns = 2;

// 机器人控制：遥控器挡位 → 机械臂动作（挡位表见任务书）
void robotControl(void) {
  {
    static float stuck_ref[2] = {0.f, 0.f};
    static uint16_t stuck_ticks = 0;
    static uint16_t traj_ticks = 0;
    if (arm.trajRunning()) {
      const float d0 = fabsf(arm.reference().q[0] - stuck_ref[0]);
      const float d1 = fabsf(arm.reference().q[1] - stuck_ref[1]);
      stuck_ref[0] = arm.reference().q[0];
      stuck_ref[1] = arm.reference().q[1];
      if (d0 < 2e-5f && d1 < 2e-5f) ++stuck_ticks; else stuck_ticks = 0;
      if (++traj_ticks > 8000 || stuck_ticks > 200) {
        arm.trajAbort();
        arm_step = 0;
        traj_ticks = 0;
        stuck_ticks = 0;
      }
    } else {
      stuck_ticks = 0;
      traj_ticks = 0;
    }
  }

  if (arm.mode() == Arm::Mode::Joint) {
    const float e0d = (arm.reference().q[0] - arm.state().q[0]) * 57.29578f;
    const float e1d = (arm.reference().q[1] - arm.state().q[1]) * 57.29578f;
    if (fabsf(e0d) > kRunawayDeg || fabsf(e1d) > kRunawayDeg) {
      if (++runaway_ticks > 200) {
        arm.disable();
        arm_step = 0;
        arm_start_ok = false;
        ++g_dbg_runaway;
        runaway_ticks = 0;
      }
    } else {
      runaway_ticks = 0;
    }
  }
  if (arm.mode() == Arm::Mode::Joint && !arm.feedbackReady()) {
    arm.disable();
    arm_step = 0;
  }

  if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.calibrate(0.0f, 0.0f);
    }
  }

  if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.enable();
    } else {
#ifdef RF206S
      float speed0 = (float)rc.channel_.l_row * (kManualSpeedMax / 1024.0f);
      float speed1 = (float)rc.channel_.r_col * (kManualSpeedMax / 1024.0f);
#else
      float speed0 = (float)rc.channel_.l_row * (kManualSpeedMax / 660.0f);
      float speed1 = (float)rc.channel_.r_col * (kManualSpeedMax / 660.0f);
#endif
      if (speed0 > -kManualDeadBand && speed0 < kManualDeadBand) speed0 = 0.f;
      if (speed1 > -kManualDeadBand && speed1 < kManualDeadBand) speed1 = 0.f;
      if (g_test_speed_enable != 0) {
        const uint8_t j = (g_test_joint != 0) ? 1 : 0;
        arm.setJointAngleDeg(j, arm.state().q[j] * 57.29578f);
        arm.setJointSpeedDps(j, g_test_speed_dps);
      } else if (g_test_enable != 0) {
        arm.setJointAngleDeg(g_test_joint, g_test_angle_deg);
      } else {
        arm.addJointReference(speed0, speed1);
      }
    }
  }

  bool square_gear = rc.switch_.l == decltype(rc)::MID &&
                     rc.switch_.r == decltype(rc)::UP;
  bool square_edge = square_gear && (last_rc_switch.l != decltype(rc)::MID ||
                                     last_rc_switch.r != decltype(rc)::UP);
  if (square_gear) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.enable();
    }
    if (square_edge && arm_step == 0 && arm.mode() == Arm::Mode::Joint) {
      const CartesianReference cur = ArmKinematics::forward(arm.reference().q);
      float dx = cur.x - kSquare[0][0];
      float dy = cur.y - kSquare[0][1];
      {
        const float kRmin = 0.025f;
        const float kRmax = 0.185f;
        bool reach_ok = true;
        for (int i = 0; i < 4 && reach_ok; ++i) {
          const int j = (i + 1) % 4;
          const float mx = 0.5f * (kSquare[i][0] + kSquare[j][0]) + dx;
          const float my = 0.5f * (kSquare[i][1] + kSquare[j][1]) + dy;
          const float cand[3][2] = {{kSquare[i][0] + dx, kSquare[i][1] + dy},
                                    {mx, my},
                                    {kSquare[j][0] + dx, kSquare[j][1] + dy}};
          for (int k = 0; k < 3; ++k) {
            const float rr = sqrtf(cand[k][0] * cand[k][0] + cand[k][1] * cand[k][1]);
            if (rr < kRmin || rr > kRmax) { reach_ok = false; break; }
          }
        }
        if (!reach_ok) { dx = 0.f; dy = 0.f; }
      }
      sq_dx = dx;
      sq_dy = dy;
      const float near = fabsf(sq_dx) + fabsf(sq_dy);
      const float prep_s = (near < 0.01f) ? 0.2f : kPrepDuration;
      if (arm.trajStart(kSquare[0][0] + sq_dx, kSquare[0][1] + sq_dy, prep_s)) {
        arm_step = 1;
        arm_start_ok = true;
      }
    } else if (arm_step >= 1 && !arm.trajRunning()) {
      if (!arm.trajFinished()) {
        arm_step = 0;
        arm_start_ok = false;
      } else if (arm_step < 5) {
        const float v_now0 = fabsf(arm.state().dq[0]);
        const float v_now1 = fabsf(arm.state().dq[1]);
        const float e_now0 = fabsf((arm.reference().q[0] - arm.state().q[0]) * 57.29578f);
        const float e_now1 = fabsf((arm.reference().q[1] - arm.state().q[1]) * 57.29578f);
        const bool pos_ok = (e_now0 < kSettleErrDeg) && (e_now1 < kSettleErrDeg);
        const bool settled = (v_now0 < kSettleSpeedRad) && (v_now1 < kSettleSpeedRad) && pos_ok;
        // 角上先等停稳再换段，超时就直接走下一段
        if (!settled && settle_ticks < kSettleMaxTicks) {
          ++settle_ticks;
          if (settle_ticks > 100 && kBreakawayCounts > 0.f) {
            const float dir0 = (arm.reference().q[0] > arm.state().q[0]) ? 1.f : -1.f;
            corner_boost0 = dir0 * kBreakawayCounts;
          }
        } else {
          settle_ticks = 0;
          corner_boost0 = 0.f;
          const int idx = (kSquareReverse != 0) ? ((4 - arm_step) % 4)
                                                : (arm_step % 4);
          if (arm.trajStart(kSquare[idx][0] + sq_dx, kSquare[idx][1] + sq_dy, kEdgeDuration)) {
            arm_step = (uint8_t)(arm_step + 1);
            arm_start_ok = true;
          } else {
            arm_step = 0;
            arm_start_ok = false;
          }
        }
      } else {
        arm_step = 0;
      }
    }
  }

  if (!square_gear && arm_step >= 1 && arm_step <= 5) {
    arm.trajAbort();
    arm_step = 0;
    arm_start_ok = false;
  }

  bool circle_gear = rc.switch_.l == decltype(rc)::DOWN &&
                     rc.switch_.r == decltype(rc)::UP;
  bool circle_edge = circle_gear && (last_rc_switch.l != decltype(rc)::DOWN ||
                                     last_rc_switch.r != decltype(rc)::UP);
  if (circle_gear) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.enable();
    }
    if (circle_edge && arm_step == 0 && arm.mode() == Arm::Mode::Joint) {
      if (arm.trajStartCircle(kCircleCx, kCircleCy, kCircleR, kCirclePeriod,
                              kCircleTurns)) {
        arm_step = 6;
        arm_start_ok = true;
      }
    } else if (arm_step == 6 && !arm.trajRunning()) {
      arm_step = 0;
      if (!arm.trajFinished()) {
        arm_start_ok = false;
      }
    }
  }


  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      led.setModeBlink(4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      led.setModeBlink(5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      led.setLED(false, true, 4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      led.setLED(false, true, 5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      led.setLED(false, true, 6);
    }
  }
#endif
}
