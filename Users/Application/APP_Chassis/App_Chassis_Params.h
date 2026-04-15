#ifndef INFANTRY_ROBOT_APP_CHASSIS_PARAMS_H
#define INFANTRY_ROBOT_APP_CHASSIS_PARAMS_H

/*
 * 底盘运动参数集中定义文件。
 *
 * 使用约定：
 * 1) 本文件只放参数，不放运行逻辑。
 * 2) 调参时优先修改这里，确保任务层与行为层使用同一组数值。
 * 3) 设备层协议缩放参数仍保留在 Dev_Can_Receive.h，不在此文件迁移。
 *
 * 单位约定：
 * - 线速度：m/s
 * - 角速度：rad/s
 * - 加速度/斜坡率：m/s^2 或 rad/s^2
 * - 角度/阈值：rad
 */

/* -------------------- 指令速度配置 -------------------- */
// 行为层 v_set 目标的主限幅。
#define CHASSIS_KEY_MAX_SPEED 2.0f

/* -------------------- 小陀螺基础参数 -------------------- */
// 小陀螺角速度下限。
#define CHASSIS_SPIN_LOW_SPEED 1.5f

// 小陀螺基础角速度（线速度耦合衰减前）。
#define CHASSIS_SPIN_MAIN_SPEED 50.0f

// 耦合系数：线速度越高，小陀螺目标角速度越低。
#define CHASSIS_SPIN_LOW_SEN 0.6f

/* -------------------- 腿长斜坡参数 -------------------- */
// 跨步/状态切换时的通用腿长变化速率。
#define CHASSIS_LEG_STEP_RAMP_SPEED 0.10f

// 跳跃空中/落地阶段的腿长恢复速率。
#define CHASSIS_JUMP_AIRBORNE_LEG_RAMP_SPEED 0.80f

/* -------------------- 初始化自扶正参数 -------------------- */
// 初始化自扶正阶段的腿部旋转速度设定。
#define CHASSIS_INIT_LEVEL_ROTATE_SPEED 0.5f

// 初始化自扶正阶段的腿部旋转速度硬限幅。
#define CHASSIS_INIT_LEVEL_SPEED_LIMIT 1.5f

/* -------------------- 初始化流程开关 -------------------- */
// 1: 绕过 CHASSIS_INIT 模式，姿态正常后直接进入 pending_state。
// 0: 保留原始 INIT 流程。
#define CHASSIS_BYPASS_INIT_MODE 0

/* -------------------- 行为层 yaw/朝向切换参数 -------------------- */
// 组合 yaw 指令输出的全局角速度保护上限。
#define CHASSIS_SMALL_GYRO_D_YAW_MAX 1000.0f

// 小陀螺角速度斜坡：开启加速率 / 关闭减速率。
#define CHASSIS_SMALL_GYRO_RAMP_UP_RATE 24.0f
#define CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE 18.0f

// 方向切换时，heading 目标角的斜坡速率。
#define CHASSIS_DIRECTION_YAW_RAMP_RATE 4.0f

// heading 斜坡生效时的 yaw 角速度限幅。
#define CHASSIS_DIRECTION_D_YAW_MAX 8.0f

// 行为层 v_set 滤波使用的线速度加速/制动斜坡。
#define CHASSIS_DIRECTION_VX_ACCEL 4.5f
#define CHASSIS_DIRECTION_VX_BRAKE_ACCEL 5.0f

// 方向解算迟滞，避免 180 度附近频繁翻转。
#define CHASSIS_HEADING_SWITCH_HYSTERESIS 0.03f

// 侧向朝向判定阈值（用于侧对到前后缓转锁存）。
#define CHASSIS_SIDE_HEADING_THRESHOLD 0.35f

// 侧对到前后缓转锁存的释放阈值。
#define CHASSIS_FRONT_BACK_RAMP_RELEASE_ERR 0.12f

// 方向向量归一化时的零向量保护阈值。
#define CHASSIS_DIRECTION_EPSILON 1e-6f

/* -------------------- 兼容保留（当前代码未引用） -------------------- */
// 以下参数在当前控制链路中未被代码直接使用，保留用于旧逻辑兼容或后续扩展。

// 旧版普通模式速度边界。
#define NORMAL_MAX_CHASSIS_SPEED_V 0.5f
#define NORMAL_MIN_CHASSIS_SPEED_V -0.5f

// 旧版 X/Y 绝对速度上限。
#define NORMAL_MAX_CHASSIS_SPEED_X 4.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 4.5f

// 旧版小陀螺模式线速度边界。
#define ROTATION_MAX_CHASSIS_SPEED_V 0.5f
#define ROTATION_MIN_CHASSIS_SPEED_V -0.5f

// 旧版遥控器 yaw 灵敏度与几何补偿参数。
#define CHASSIS_WZ_RC_SEN 0.01f
#define CHASSIS_WZ_SET_SCALE 0.0f

// 旧版速度档位/斜坡参数。
#define CHASSIS_KEY_MAX_SPEED_UP 1.0f
#define CHASSIS_KEY_ACCEL 1.8f
#define CHASSIS_KEY_RAMP_STEP 0.004f

#endif
