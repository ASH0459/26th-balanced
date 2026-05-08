/** ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
* @file  Dev_Can_Receive.h
* @brief
* @note
* @history
* Version  Date                    Author   Modification
* V1.0.0   10-01-2025 无垠    1. done
*
@verbatim
==============================================================================


==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2025 Robot_Z ****************************
*/
#pragma once
/** * @brief 头文件 */
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>

#include "arm_math.h"

/** * @brief 宏定义 */
#define CHASSIS_JOINT_CAN1 hfdcan1
#define CHASSIS_JOINT_CAN2 hfdcan2
#define CHASSIS_WHEEL_CAN hfdcan3

// 转矩系数
#define CONSTANT_OF_TORQUE 3057 // 3291.065087

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -40.0f
#define T_MAX 40.0f

// 云台 YAW 轴"朝前"机械中值（rad）。
// 相对角计算: relative_yaw = wrapToPi(yaw_motor_pos - YAW_CENTER_POS_RAD)
#define YAW_CENTER_POS_RAD -0.5f

// yaw 电机挂在底盘 CAN3 时的反馈 ID（按实际电机 ID 调整）。
#define CHASSIS_YAW_MOTOR_CAN_ID 0x11

// yaw 电机角度方向修正（若方向相反改为 -1.0f）。
#define CHASSIS_YAW_MOTOR_DIR_SIGN 1.0f

/** * @brief 枚举 */
typedef enum
{
    CAN_CHASSIS_JOINT_LEFT_1_ID = 0x01,
    CAN_CHASSIS_JOINT_LEFT_2_ID = 0x02,
    CAN_CHASSIS_JOINT_RIGHT_1_ID = 0x03,
    CAN_CHASSIS_JOINT_RIGHT_2_ID = 0x04,

    CAN_CHASSIS_WHEEL_ALL_ID = 0x200,
    CAN_CHASSIS_WHEEL_LEFT_ID = 0x201,
    CAN_CHASSIS_WHEEL_RIGHT_ID = 0x202,
    CAN_CHASSIS_YAW_MOTOR_ID = CHASSIS_YAW_MOTOR_CAN_ID,

    GIMBAL_ID = 0x302,
    CAN_VT_ID = 0x303,
    CAN_ATOM_ID = 0X2BC,
} can_msg_id_e;

typedef enum
{
    CHASSIS_MODE_NO_FORCE = 0,
    CHASSIS_MODE_RESERVED = 1,
    CHASSIS_MODE_NORMAL = 2,
    CHASSIS_MODE_JUMP = 3, // 保留协议位，当前不作为底盘动作入口
    CHASSIS_MODE_STEP_1 = 4,
    CHASSIS_MODE_STEP_2 = 5,
} chassis_mode_e;

typedef enum
{
    FRIC_OFF = 0,
    FRIC_ON = 1,
    FRIC_ERROR = 2,
} Fric_State_e;

typedef enum
{
    CHASSIS_AUTO_AIM_STATE_NO_TARGET = 0,
    CHASSIS_AUTO_AIM_STATE_MANUAL_FIRE_TARGET = 1,
    CHASSIS_AUTO_AIM_STATE_AUTO_FIRE_TARGET = 2,
} chassis_auto_aim_state_e;

typedef enum
{
    CHASSIS_FEATURE_FLAG_GYRO_ENABLE = 0x01,
    CHASSIS_FEATURE_FLAG_STEP = 0x02,
    CHASSIS_FEATURE_FLAG_UI_RESET = 0x04,
} chassis_feature_flag_e;

/** * @brief 结构体 */
typedef struct
{
    int16_t v_set;
    uint8_t auto_aim_state;
    uint8_t chassis_feature_flags;
    uint8_t mode;
    uint8_t fric_state;
    int16_t turn_set;
} gimbal_can_cmd_frame_t;

/** * @brief 类声明 */
#ifdef __cplusplus

class Wheel_Motor_Measure
{
public:
    uint16_t id;
    FDCAN_HandleTypeDef *hfdcan;
    Wheel_Motor_Measure(const uint16_t id, FDCAN_HandleTypeDef *hfdcan)
        : id(id), hfdcan(hfdcan) {}

    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int32_t total_ecd;
    int32_t count;

    void get_motor_measure(uint8_t data[8]);

    const Wheel_Motor_Measure *get_chassis_motor_measure_point() const
    {
        return this;
    }

private:
};

class Joint_Motor_Measure
{
public:
    uint8_t id;
    FDCAN_HandleTypeDef *hfdcan;
    Joint_Motor_Measure(const uint8_t id, FDCAN_HandleTypeDef *hfdcan)
        : id(id), hfdcan(hfdcan) {}

    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    int32_t count;
    fp32 total_pos;
    fp32 pos;
    fp32 last_pos;
    fp32 vel;
    fp32 tor;

    void Enable_Joint_Motor() const;
    void Joint_MIT_Control(const float pos, const float vel, const float kp, const float kd, float torq) const;
    void Set_Mit_Zero_Point() const;
    void get_joint_motor_measure(uint8_t data[8]);

    const Joint_Motor_Measure *get_chassis_motor_measure_point() const
    {
        return this;
    }

private:
};

class Gimbal_Data
{
public:
    fp32 v_tmp;
    fp32 chassis_relative_angle;
    fp32 yaw_set;
    uint8_t auto_aim_state;
    uint8_t chassis_feature_flags;
    uint8_t gyro_enable;
    uint8_t step_enable;
    uint8_t protocol_valid;
    Fric_State_e fric_state;
    chassis_mode_e chassis_behaviour_mode;
    uint16_t reserved_flags;  // 高字节: Byte0, 低字节: Byte1

    const Gimbal_Data *get_gimbal_point() const
    {
        return this;
    }

private:
};

class Chassis_Data
{
public:
    int16_t heat;
    int16_t shoot_num;
    int16_t money;
    uint8_t live_state;
    uint8_t id;

private:
};

#endif // __cplusplus

/** * @brief 外部变量声明与函数声明 */
#ifdef __cplusplus

extern Joint_Motor_Measure chassis_joint[4];
extern Wheel_Motor_Measure chassis_wheel[2];
extern Joint_Motor_Measure chassis_yaw_motor;
extern Gimbal_Data gimbal_data;

extern "C"
{
#endif

bool_t chassis_mode_is_valid(const uint8_t mode_byte);
void CAN_cmd_gimbal_receive(const uint8_t *received_data);
void CAN_cmd_wheel(const fp32 motor1, const fp32 motor2);
void CAN_cmd_steering(const int16_t motor1, const int16_t motor2);

#ifdef __cplusplus
}
#endif
