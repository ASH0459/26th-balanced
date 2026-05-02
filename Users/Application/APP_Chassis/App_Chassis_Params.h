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
#define CHASSIS_KEY_BACK_MAX_SPEED 1.5f
#define CHASSIS_LEG_MAX_SPEED 1.5f
#define CHASSIS_STEP_UP_MAX_SPEED 0.95f

/* -------------------- 腿长斜坡参数 -------------------- */
// 跨步/状态切换时的通用腿长变化速率。
#define CHASSIS_LEG_STEP_RAMP_SPEED 0.3f
// 上台阶收腿阶段回到 normal 腿长的恢复速率。
#define CHASSIS_STEP_UP_RETRACT_LEG_RAMP_SPEED 0.6f

/* -------------------- Normal切入触地保持 -------------------- */
// 刚切入 CHASSIS_NORMAL 后，短时间内强制认为双腿触地，抑制支持力判定抖动。
#define CHASSIS_NORMAL_FORCE_TOUCH_GROUND_TICKS 1500U

// 跳跃空中/落地阶段的腿长恢复速率。
#define CHASSIS_JUMP_AIRBORNE_LEG_RAMP_SPEED 0.80f

/* -------------------- 初始化自扶正参数 -------------------- */
// 初始化自扶正阶段的腿部旋转速度设定。
#define CHASSIS_INIT_LEVEL_ROTATE_SPEED 0.8f

// 初始化自扶正阶段的腿部旋转速度硬限幅。
#define CHASSIS_INIT_LEVEL_SPEED_LIMIT 1.5f

/* -------------------- 初始化流程开关 -------------------- */
// 1: 绕过 CHASSIS_INIT 模式，姿态正常后直接进入 pending_state。
// 0: 保留原始 INIT 流程
#define CHASSIS_BYPASS_INIT_MODE 1

/* -------------------- 输出调试开关 -------------------- */
// 1: 腿部仅保留垂直支持力（禁用 Tbl_t 横向输出）并关闭轮子输出。
// 0: 保持轮腿协同输出。
#define CHASSIS_VERTICAL_SUPPORT_ONLY_MODE 1

// 1: 强制所有底盘电机输出清零（关节 + 轮子），用于紧急停机/联调保护。
// 0: 使用正常控制输出。
#define CHASSIS_FORCE_ALL_MOTOR_ZERO_OUTPUT 0


/* -------------------- 腿部水平输出角度衰减 -------------------- */
// 1: 按摆杆角度衰减 Tbl_t，摆杆角越大，水平输出越小。
// 0: 关闭该衰减逻辑。
#define CHASSIS_LEG_TBL_ANGLE_ATTENUATION_ENABLE 0

// Sigmoid 中心点 (rad): |theta_l| == center 时，缩放系数为 0.5。
#define CHASSIS_LEG_TBL_ANGLE_ATTENUATION_CENTER 0.2f

// Sigmoid 陡峭度: 数值越大，中心点附近衰减越快。
#define CHASSIS_LEG_TBL_ANGLE_ATTENUATION_SHARPNESS 4.0f

/* -------------------- INIT起身驱动偏置 -------------------- */
// 1: INIT_STAND 阶段按摆杆角放大腿水平输出并抑制轮子输出。
// 0: 关闭该偏置逻辑。
#define CHASSIS_INIT_STAND_DRIVE_BIAS_ENABLE 0

// 摆杆角偏置中心点 (rad): |theta_l| == center 时偏置强度约为中等。
#define CHASSIS_INIT_STAND_DRIVE_BIAS_CENTER 0.1f

// 偏置陡峭度: 数值越大，中心点附近增益变化越快。
#define CHASSIS_INIT_STAND_DRIVE_BIAS_SHARPNESS 7.0f

// INIT_STAND 时腿部 Tbl_t 缩放范围，角度越大越接近 MAX。
#define CHASSIS_INIT_STAND_LEG_TBL_SCALE_MIN 1.0f
#define CHASSIS_INIT_STAND_LEG_TBL_SCALE_MAX 3.0f

// INIT_STAND 时轮子 wheel_T 缩放范围，角度越大越接近 MIN。
#define CHASSIS_INIT_STAND_WHEEL_SCALE_MIN 0.2f
#define CHASSIS_INIT_STAND_WHEEL_SCALE_MAX 1.0f

/* -------------------- 行为层 yaw 与速度参数 -------------------- */
// 组合 yaw 指令输出的全局角速度保护上限。
#define CHASSIS_D_YAW_MAX 1000.0f

/* -------------------- 小陀螺参数 -------------------- */
// gyro_enable=1 时，底盘以恒定角速度旋转；正负号决定旋转方向。
#define CHASSIS_SMALL_GYRO_D_YAW_SET 40.0f

// 小陀螺启停斜坡速率（rad/s^2）。
#define CHASSIS_SMALL_GYRO_RAMP_UP_RATE 40.0f
#define CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE 40.0f

// 小陀螺模式下认为“无平移输入”的阈值（m/s）。
#define CHASSIS_SMALL_GYRO_ZERO_V_INPUT_EPS 0.03f

// 小陀螺无平移输入时，用 v_set 抵消 v_filter 扰动的补偿增益与限幅。
#define CHASSIS_SMALL_GYRO_V_COMPENSATION_GAIN 1.0f
#define CHASSIS_SMALL_GYRO_V_COMPENSATION_MAX 1.0f

// 小陀螺平移补偿前馈：按 relative_angle 生成正弦补偿，便于抵消固定方向漂移。
#define CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_GAIN  0.0f
#define CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_PHASE 0.7853982f

// 小陀螺平移控制：将标量 v_tmp 按当前 relative_angle 投影到期望平移方向。
// 约定：小陀螺开启时，yaw_set 作为“期望平移方向”使用。
#define CHASSIS_SMALL_GYRO_MOVE_GAIN  1.0f
#define CHASSIS_SMALL_GYRO_MOVE_PHASE 0.0f
#define CHASSIS_SMALL_GYRO_MOVE_SPEED_SCALE 0.45f

// 行为层 v_set 滤波使用的线速度加速/制动斜坡。
#define CHASSIS_DIRECTION_VX_ACCEL 0.5f
#define CHASSIS_DIRECTION_VX_BRAKE_ACCEL 3.0f

// v_set 加速阶段增益: 先急后缓（起步用 FAST，接近目标逐步过渡到 SLOW）。
#define CHASSIS_DIRECTION_VX_ACCEL_FAST_GAIN 10.0f
#define CHASSIS_DIRECTION_VX_ACCEL_SLOW_GAIN 0.5f

/* -------------------- 兼容保留（当前代码未引用） -------------------- */
// 以下参数在当前控制链路中未被代码直接使用，保留用于旧逻辑兼容或后续扩展。

// 旧版普通模式速度边界。
#define NORMAL_MAX_CHASSIS_SPEED_V 0.5f
#define NORMAL_MIN_CHASSIS_SPEED_V -0.5f

// 旧版 X/Y 绝对速度上限。
#define NORMAL_MAX_CHASSIS_SPEED_X 4.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 4.5f

// 旧版旋转模式线速度边界。
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
