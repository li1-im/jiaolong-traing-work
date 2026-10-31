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
// extern IMU board_imu;

extern Arm arm;
BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
#ifdef RF206S
WFLY::RCSwitch last_rc_switch;
#endif
#ifdef DR16
RC::RCSwitch last_rc_switch;
#endif

uint8_t arm_step = 0;  // 0 空闲，1 预备，2~5 依次走向 B、C、D、A
volatile bool arm_start_ok = false;  // 最近一次轨迹是否启动成功

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
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    robotControl();  // 机器人控制
  }
  arm.handle();
  boardLedHandle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
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

// ===================== 任务自定义参数 =====================
// 正方形四个顶点，单位 m。任务书给的是 mm：
//   A(100,100) B(100,-100) C(-100,-100) D(-100,100)，A→B→C→D→A 顺时针绕原点
static const float kSquare[4][2] = {
  { 0.1f,  0.1f},  // A
  { 0.1f, -0.1f},  // B
  {-0.1f, -0.1f},  // C
  {-0.1f,  0.1f},  // D
};
static constexpr float kPrepDuration = 2.0f;   // 预备段：2 s 平滑走到 A（任务书规定）
static constexpr float kEdgeDuration = 1.0f;   // 正方形每条边的用时（可按效果微调）
static constexpr float kManualSpeedMax = 1.5f; // 摇杆满舵时的关节速度 rad/s
static constexpr float kManualDeadBand = 0.15f;  // 摇杆归一化死区，防抖动

// ---- 可选图形：正圆（大项目说明的验收项 b：正圆圆度）----
// 可达环带是 0.07~0.16 m（= |l1-l0| ~ l0+l1）。以原点为心、半径 R 的圆，
// 末端到原点距离恒等于 R，只要 R 在环带内就可行 —— 这是圆度最好的画法
//（此时 J1 几乎不动，只有 J0 匀速转一圈）。
static constexpr float kCircleCx = 0.0f;     // m，圆心（原点）
static constexpr float kCircleCy = 0.0f;     // m
static constexpr float kCircleR = 0.10f;     // m，半径 100 mm => 直径 200 mm
static constexpr float kCirclePeriod = 4.0f; // s，一圈用时（越长越慢、圆度越好）
static constexpr int kCircleTurns = 2;       // 连续画几圈

// 机器人控制：遥控器挡位 → 机械臂动作（挡位表见任务书）
// 注意：“右下”挡已经被 robotPowerStateFSM 的 stop_flag 覆盖
//（r == DOWN 会切到 STOP 状态 → robotReset() → arm.disable()），这里只管其余挡位。
void robotControl(void) {
  // ---------- 电机反馈掉线保护 ----------
  // 任一电机 CAN 反馈丢失（50 ms 超时）时，电机底层已自动转 POWEROFF 并清零
  // 累计角度；这里同步断开机械臂闭环：保持最后位置参考、清除前馈，arm_step
  // 回空闲。反馈恢复后，左上右上 / 左中右上挡会在下个周期自动重新 enable()，
  // 但重连后的绝对零点需要重新标定（左中右中挡）后再重新触发动作。
  if (arm.mode() == Arm::Mode::Joint && !arm.feedbackReady()) {
    arm.disable();
    arm_step = 0;
  }

  // ---------- 左中右中：失能状态下标定零位 ----------
  // 零位 = 两杆沿 +X 拉直，即 q0 = q1 = 0。
  // 只要还停留在失能状态就持续尝试（反馈没在线前按下摇杆也不会错过）：
  // calibrate 内部要求“失能 + 反馈在线”，标定成功一次后 mode 仍为 Disabled，
  // 但反复写入同一个零点值是无害的（幂等）。
  if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.calibrate(0.0f, 0.0f);
    }
  }

  // ---------- 左上右上：双关节手动控制 ----------
  if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
    if (arm.mode() == Arm::Mode::Disabled) {
      // 刚切进该挡位还没使能：以当前实际姿态为使能目标。
      // enable 成功前条件每周期重访（等电机反馈上线），成功后此分支自动不再进入。
      arm.enable();
    } else {
      // 摇杆原始通道 → 归一化 ±1 → 乘最大速度 = 关节角速度指令 rad/s
      // DR16 通道范围 ±660；RF206S(WFLY) 为 raw−1024 即 ±1024，按遥控型号折算
      // 左手上下(l_row)控 J0，右手左右(r_col)控 J1；方向若与实物相反改符号即可
#ifdef RF206S
      float speed0 = (float)rc.channel_.l_row * (kManualSpeedMax / 1024.0f);
      float speed1 = (float)rc.channel_.r_col * (kManualSpeedMax / 1024.0f);
#else
      float speed0 = (float)rc.channel_.l_row * (kManualSpeedMax / 660.0f);
      float speed1 = (float)rc.channel_.r_col * (kManualSpeedMax / 660.0f);
#endif
      // 简易死区：摇杆回中附近的小抖动一律视为 0（速度 0 → 保持当前位置）
      if (speed0 > -kManualDeadBand && speed0 < kManualDeadBand) speed0 = 0.f;
      if (speed1 > -kManualDeadBand && speed1 < kManualDeadBand) speed1 = 0.f;
      // 速度指令交给 Arm 内部积分成位置目标（限位的处理都在 Arm 里）
      arm.addJointReference(speed0, speed1);
    }
  }

  // ---------- 左中右上：预备动作 + 完整正方形（只触发一次） ----------
  bool square_gear = rc.switch_.l == decltype(rc)::MID &&
                     rc.switch_.r == decltype(rc)::UP;
  // “挡位沿”指从中立位/其他挡位拨进来的一瞬间，防止挡位停住后无限重启动作
  bool square_edge = square_gear && (last_rc_switch.l != decltype(rc)::MID ||
                                     last_rc_switch.r != decltype(rc)::UP);
  if (square_gear) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.enable();  // 该动作同样需要先使能，反馈未就绪时下周期自动再试
    }
    if (square_edge && arm_step == 0 && arm.mode() == Arm::Mode::Joint) {
      // 第 1 步（arm_step=1）：从当前规划目标用 2 s 五次曲线走到 A。
      // trajStart 内部做整线段可达性检查 + 逆解（默认正肘支，圈数连续展开）。
      if (arm.trajStart(kSquare[0][0], kSquare[0][1], kPrepDuration)) {
        arm_step = 1;
        arm_start_ok = true;
      }
    } else if (arm_step >= 1 && arm.trajFinished() && !arm.trajRunning()) {
      // 上一段“正常走完”（不是被取消）才衔接下一段：
      // trajStart 从上一段终点起步 => 相邻段参考天然连续。
      if (arm_step < 5) {
        uint8_t next = (uint8_t)(arm_step + 1);
        // next 2→B、3→C、4→D、5→回到 A 闭合整个正方形
        if (!arm.trajStart(kSquare[next - 1][0], kSquare[next - 1][1],
                           kEdgeDuration)) {
          arm_step = 0;  // 启动失败（丢反馈/越限）：回空闲，保持最后位置
        }
      } else {
        // 完整一圈结束：回到空闲，位置参考保持在 A，J0 已净转一圈；
        // 想再跑一轮需要把右开关拨离 UP 再拨回来（保证“只触发一次”）。
        arm_step = 0;
      }
    }
  }

  // ---------- 左下右上：可选图形 正圆（只触发一次） ----------
  // 与正方形同样的触发约定：挡位沿触发一次，画完保持位置，再画要把开关拨离再拨回。
  bool circle_gear = rc.switch_.l == decltype(rc)::DOWN &&
                     rc.switch_.r == decltype(rc)::UP;
  bool circle_edge = circle_gear && (last_rc_switch.l != decltype(rc)::DOWN ||
                                     last_rc_switch.r != decltype(rc)::UP);
  if (circle_gear) {
    if (arm.mode() == Arm::Mode::Disabled) {
      arm.enable();  // 反馈未就绪时下周期自动再试
    }
    if (circle_edge && arm_step == 0 && arm.mode() == Arm::Mode::Joint) {
      // 起点相位自动对齐当前末端方位角 => 不会跳一下再开始画
      if (arm.trajStartCircle(kCircleCx, kCircleCy, kCircleR, kCirclePeriod,
                              kCircleTurns)) {
        arm_step = 6;  // 6 表示“圆轨迹进行中”（1~5 已被正方形占用）
        arm_start_ok = true;
      }
    } else if (arm_step == 6 && arm.trajFinished() && !arm.trajRunning()) {
      arm_step = 0;  // 画完：保持最后位置
    }
  }

  // ---------- 其余挡位：保持当前目标 ----------
  // 什么都不做即可：arm.handle() 每个周期都会把 reference_ 继续写成电机目标，
  // 不在轨迹中时自然就是“位置保持”。

  // 记录本周期挡位，供下一周期做边沿检测（robotReset() 里也有同样的快照）
  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
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
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == decltype(rc)::UP && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == decltype(rc)::MID && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == decltype(rc)::DOWN && rc.switch_.r == decltype(rc)::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}
