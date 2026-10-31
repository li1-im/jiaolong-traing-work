/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"
#include<cmath>

// Motor parameter config
// 电机参数配置

// Chassis motor 底盘电机
// const PID chassis_wheel_spid(40, 1, 10, 1000, 16384);
// Motor CMFL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMFR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid
// Motor CMBR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
//            PID(), PID(chassis_wheel_spid));             // ppid, spid

// Gimbal motor 云台电机
// Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf
// Motor GMP(Motor::GM6020, -1, Motor::POSITION_SPEED,  // type, ratio, method
//           PID(25, 0.1, 10, 10, 1800),                // ppid
//           PID(400, 2, 0, 500, 30000),                // spid
//           true);                                     // use kf

// Friction wheel motors
// const PID fric_spid(6, 0.05, 2, 200, 16384);
// Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid
// Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
//             PID(), PID(fric_spid));         // ppid, spid

// Stir motor
// Motor STIR(Motor::M2006, 36, Motor::POSITION_SPEED,  // type, ratio, method
//            PID(20, 0.1, 10, 10, 2500),               // ppid
//            PID(60, 0.1, 200, 1000, 10000));          // spid

// 两关节配置：CAN1 上 J1 ID=1、J0 ID=2；现有 PID 参数作为调试起点。
// Other motors
// 在线 KF Q 为 deg²、(deg/s)²，R 为 deg²
const Motor::KFParam_t joint_kf_param(5.f, 6e5f, 1.f);

// ================== 机械传动说明（学长的齿轮版装置） ==================
// 这是【第二套项目】：适配学长那台「肩部齿轮减速 + 肘部电机直驱」的装置。
// 和 Double-jointed-arm-main（你的同步带版）是两个独立工程，别混着烧。
//
// 装置判读（现场照片确认，齿数请再数一遍）：
//   两台 M3508 都是【去掉内置行星减速箱的裸电机】（电机轴直接顶着齿轮/连杆，
//   没有那节银色行星壳体）。所以 ratio_ 里【不含 19.2】，只有机械减速比：
//   J0 肩部（CAN1 ID2）：电机轴齿轮 → 肩轴齿轮（外啮合）
//        两个齿轮**直径几乎相同**（现场标注 5.5 与 5，单位应为直径）：
//        传动比 ≈ 5.5/5 = 1.1，可近似按 **1.0** 处理（不是 2 倍减速！）
//   J1 肘部（CAN1 ID1）：裸电机直接驱动第二杆（无外部减速）→ 1.0
//
// ★ 也就是说：肩部几乎没有机械减速，力矩全靠裸电机自己（≈0.23 N·m）,
//   所以"托得住手臂"是这套装置最大的风险点，先做悬停测试。
//
// 符号（转向）——实测结论（2026-10-31 实机验证）：
//   J0 = +1.0：肩部电机经外啮合齿轮驱动，实测与模型同向；
//   J1 = -1.0：肘部电机直接驱动第二杆，但碳杆装在输出轴的**另一侧**，
//              电机正转时第二杆实际是**顺时针**折（与数学模型相反）⇒ 必须取负。
//   判据（以后换机/重装都能用这套方法）：
//     ① 手动把 J1 推到 +90°，看实物是逆时针(逆) 还是顺时针(顺)；
//     ② 跑正方形（非对称图形）——符号错时正方形会崩，而圆看不出问题。
//   为什么圆看不出来：q1 与 −q1 在 q1≈0（两杆共线）附近末端位置几乎相同，
//   只有肘角大了差别才显现；而圆只依赖半径，符号错只是换个半径，仍是个圆。
constexpr float kJ0Ratio = 1.0f;   // 齿轮 1:1（5.5/5≈1.1），实测同向
constexpr float kJ1Ratio = -1.0f;  // 肘部：杆装反侧，实测必须取负

// ================== 在线调参镜像（仿 CAN 工程 g_* 写法） ==================
// 上位机(调试探针+ELF符号)直接写下面这些 volatile 全局变量即可实时生效：
// 每 1ms 的 allMotorsHandle() 会把它们整组刷进 J0/J1 的 PID 副本与 ratio_。
// 命名与 CAN 工程 callback.cpp 的 g_kp_pp/g_ki_sp… 一一对应，只是加了关节前缀。
// 注意：改了要生效——outmax/dfilt/ratio 也是镜像刷，改源码里的 const PID
// 反而会被镜像覆盖，调参一律在 scope 里做。
// --- J0 (下层电机, CAN1 ID2) ---
volatile float g_J0_kp_pp = 75.f, g_J0_ki_pp = 0.f, g_J0_kd_pp = 400.f;
volatile float g_J0_imax_pp = 0.f, g_J0_outmax_pp = 1440.f,
               g_J0_dfilt_pp = 1.f;
volatile float g_J0_kp_sp = 30.f, g_J0_ki_sp = 0.01f, g_J0_kd_sp = 0.f;
volatile float g_J0_imax_sp = 0.f, g_J0_outmax_sp = 4000.f,
               g_J0_dfilt_sp = 1.f;
volatile float g_J0_ratio = kJ0Ratio;
// --- J1 (上层电机, CAN1 ID1) ---
volatile float g_J1_kp_pp = 50.f, g_J1_ki_pp = 0.f, g_J1_kd_pp = 100.f;
volatile float g_J1_imax_pp = 0.f, g_J1_outmax_pp = 1440.f,
               g_J1_dfilt_pp = 1.f;
volatile float g_J1_kp_sp = 12.f, g_J1_ki_sp = 0.f, g_J1_kd_sp = 50.f;
volatile float g_J1_imax_sp = 0.f, g_J1_outmax_sp = 4000.f,
               g_J1_dfilt_sp = 1.f;
volatile float g_J1_ratio = kJ1Ratio;

// 把镜像刷进电机对象（1 ms 一次，在 allMotorsHandle 内调用）
static void motorsPidRefresh(void) {
  // J0 位置环
  J0.ppid_.kp_ = g_J0_kp_pp;  J0.ppid_.ki_ = g_J0_ki_pp;
  J0.ppid_.kd_ = g_J0_kd_pp;  J0.ppid_.i_max_ = g_J0_imax_pp;
  J0.ppid_.out_max_ = g_J0_outmax_pp;
  J0.ppid_.d_filter_.setK(g_J0_dfilt_pp);
  // J0 速度环
  J0.spid_.kp_ = g_J0_kp_sp;  J0.spid_.ki_ = g_J0_ki_sp;
  J0.spid_.kd_ = g_J0_kd_sp;  J0.spid_.i_max_ = g_J0_imax_sp;
  J0.spid_.out_max_ = g_J0_outmax_sp;
  J0.spid_.d_filter_.setK(g_J0_dfilt_sp);
  J0.ratio_ = g_J0_ratio;
  // J1 位置环
  J1.ppid_.kp_ = g_J1_kp_pp;  J1.ppid_.ki_ = g_J1_ki_pp;
  J1.ppid_.kd_ = g_J1_kd_pp;  J1.ppid_.i_max_ = g_J1_imax_pp;
  J1.ppid_.out_max_ = g_J1_outmax_pp;
  J1.ppid_.d_filter_.setK(g_J1_dfilt_pp);
  // J1 速度环
  J1.spid_.kp_ = g_J1_kp_sp;  J1.spid_.ki_ = g_J1_ki_sp;
  J1.spid_.kd_ = g_J1_kd_sp;  J1.spid_.i_max_ = g_J1_imax_sp;
  J1.spid_.out_max_ = g_J1_outmax_sp;
  J1.spid_.d_filter_.setK(g_J1_dfilt_sp);
  J1.ratio_ = g_J1_ratio;
}

// 调参备忘：两组 PID 的 i_max 均为 0 ⇒ 积分项被限幅为 0，ki 实际不参与；
// ppid 的 d_filter_k=1 是纯直通（filter.cpp: output·(1−k)+input·k），kd 作用在
// 原始 1 ms 编码器差分上，若实测输出抖动可在实机上把 d_filter_k 降到 0.2~0.5
// （现在直接在 scope 里写 g_J*_dfilt_pp 即可，无需重编译）。
const PID J0_spid(30,0.01,0,0,16384,1);
const PID J0_ppid(75,0,400,0,1440,1);
// 带传动减速：ratio_ = 总减速比(带轮齿数比)，把转子侧 ecd/rpm 折算到关节侧
Motor J0(Motor::M3508, kJ0Ratio, Motor::POSITION_SPEED,  // type, ratio, method
         J0_ppid,           // ppid
         J0_spid,           // spid
         true, joint_kf_param);  // 启用在线 KF

const PID J1_spid(12,0,50,0,16384,1);
const PID J1_ppid( 50,0,100,0,1440,1);
Motor J1(Motor::M3508, kJ1Ratio, Motor::POSITION_SPEED,
         J1_ppid,
         J1_spid,
         true, joint_kf_param);

// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &J1,   // id:1
    &J0,   // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,    // id:5
    nullptr,     // id:6
    nullptr,  // id:7
    nullptr,      // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,    // id:1
    nullptr,    // id:2
    nullptr,    // id:3
    nullptr,    // id:4
    nullptr,     // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

MITMotorDriver mit_motor_driver[] = {}; // slave_id必须小于0x0F

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::STOP;
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::WORKING;
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->reset();
    i.setCmd(mitmotor::MOTOR_MODE);
  }
  // KF 输出仍使用 deg、deg/s；位置环和速度环都使用滤波反馈
  J0.setFdbSrc(&J0.kfAngle(), &J0.kfSpeed());
  J1.setFdbSrc(&J1.kfAngle(), &J1.kfSpeed());
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  // 先刷新在线调参镜像（volatile g_* → PID增益/限幅/滤波k/ratio），1ms 生效
  motorsPidRefresh();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (auto & i : mit_motor_driver) {
    // call motor::handle() through MITMotorDriver::handle()
    i.handle();
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (auto & i : mit_motor_driver) {
    if (i.canRxMsgCheck(hcan, rx_header)) {
      i.canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}
