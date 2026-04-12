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

#define YAW_MECHANICAL_ZERO 0.32f

/** * @brief 结构体 */
typedef enum
{
    CAN_CHASSIS_JOINT_LEFT_1_ID = 0x01,
    CAN_CHASSIS_JOINT_LEFT_2_ID = 0x02,
    CAN_CHASSIS_JOINT_RIGHT_1_ID = 0x03,
    CAN_CHASSIS_JOINT_RIGHT_2_ID = 0x04,

    CAN_CHASSIS_WHEEL_ALL_ID = 0x200,
    CAN_CHASSIS_WHEEL_LEFT_ID = 0x201,
    CAN_CHASSIS_WHEEL_RIGHT_ID = 0x202,

    GIMBAL_ID = 0x302,
    CAN_VT_ID = 0x303,
    CAN_ATOM_ID = 0X2BC,
} can_msg_id_e;

typedef enum
{
    CHASSIS_MODE_NO_FORCE = 0,
    CHASSIS_MODE_RESERVED = 1,
    CHASSIS_MODE_NORMAL = 2,
    CHASSIS_MODE_JUMP = 3,
    CHASSIS_MODE_STEP_1 = 4,
    CHASSIS_MODE_STEP_2 = 5,
} chassis_mode_e;

/** * @brief 变量外部声明 */

/** * @brief CPP部分 */
#ifdef __cplusplus

static uint16_t float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, uint16_t bits)
{
    fp32 span = x_max - x_min;
    fp32 offset = x_min;
    if (x_float < x_min)
        x_float = x_min;
    else if (x_float > x_max)
        x_float = x_max;

    return (int32_t)((x_float - offset) * (fp32)((1 << bits) - 1) / span);
}

class Wheel_Motor_Measure
{
public:
    uint8_t id;                  // 电机id
    FDCAN_HandleTypeDef *hfdcan; // 电机对应的fdcan
    Wheel_Motor_Measure(const uint8_t id, FDCAN_HandleTypeDef *hfdcan)
    {
        this->id = id;
        this->hfdcan = hfdcan;
    };

    uint16_t ecd;          // 编码值
    int16_t speed_rpm;     // 原始速度数据 换算后为Rpm
    int16_t given_current; // 电流值
    uint8_t temperate;     // 温度值
    int16_t last_ecd;      // 上一次编码值
    int32_t total_ecd;     // 总编码值
    int32_t count;

    /**
     * @brief 获取该对象电机测量数据
     * @param data 接收到的CAN数据
     */
    void get_motor_measure(uint8_t data[8])
    {
        this->last_ecd = this->ecd;
        this->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
        if (this->ecd - this->last_ecd > 4096)
        {
            this->count--;
        }
        else if (this->ecd - this->last_ecd < -4096)
        {
            this->count++;
        }
        this->total_ecd = (int32_t)(8192 * this->count + this->ecd);
        this->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
        this->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
        this->temperate = (data)[6];
    }

    /**
     * @brief          获取电机对象指针
     * @param[in]      none
     * @return			返回对应电机对象的指针
     * @retval         none
     */
    const Wheel_Motor_Measure *get_chassis_motor_measure_point() const
    {
        return this;
    }

private:
};

class Joint_Motor_Measure
{
public:
    uint8_t id;                  // 电机id
    FDCAN_HandleTypeDef *hfdcan; // 电机对应的fdcan
    Joint_Motor_Measure(const uint8_t id, FDCAN_HandleTypeDef *hfdcan)
    {
        this->id = id;
        this->hfdcan = hfdcan;
    };

    uint16_t p_int; // 位置控制值
    uint16_t v_int; // 速度控制值
    uint16_t t_int; // 力矩控制值
    int32_t count;  // 计数
    fp32 total_pos; // 总位置
    fp32 pos;       // 当前位置
    fp32 last_pos;  // 上一次位置
    fp32 vel;       // 速度
    fp32 tor;       // 扭矩

    void Enable_Joint_Motor() const
    {
        static FDCAN_TxHeaderTypeDef joint_tx_message;
        static uint8_t joint_fdcan_send_data[8];

        joint_tx_message.Identifier = this->id;
        joint_tx_message.IdType = FDCAN_STANDARD_ID;
        joint_tx_message.TxFrameType = FDCAN_DATA_FRAME;
        joint_tx_message.DataLength = FDCAN_DLC_BYTES_8;
        joint_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        joint_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
        joint_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
        joint_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        joint_tx_message.MessageMarker = 0;
        joint_fdcan_send_data[0] = 0xFF;
        joint_fdcan_send_data[1] = 0xFF;
        joint_fdcan_send_data[2] = 0xFF;
        joint_fdcan_send_data[3] = 0xFF;
        joint_fdcan_send_data[4] = 0xFF;
        joint_fdcan_send_data[5] = 0xFF;
        joint_fdcan_send_data[6] = 0xFF;
        joint_fdcan_send_data[7] = 0xFC;

        HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &joint_tx_message, joint_fdcan_send_data);
    }

    /**
    @brief 电机控制发送函数
    @param pos 位置
    @param vel 速度
    @param kp P参数
    @param kd D参数
    @param torq 扭矩
    @retval none
    */
    void Joint_MIT_Control(const float pos, const float vel, const float kp, const float kd, float torq) const
    {
        static FDCAN_TxHeaderTypeDef joint_tx_message;
        static uint8_t joint_fdcan_send_data[8];
        const uint16_t pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
        const uint16_t vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
        const uint16_t kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
        const uint16_t kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
        const uint16_t tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);
        joint_tx_message.Identifier = this->id;
        joint_tx_message.IdType = FDCAN_STANDARD_ID;
        joint_tx_message.TxFrameType = FDCAN_DATA_FRAME;
        joint_tx_message.DataLength = 0x08;
        joint_fdcan_send_data[0] = (pos_tmp >> 8) & 0xFF;
        joint_fdcan_send_data[1] = pos_tmp & 0xFF;
        joint_fdcan_send_data[2] = (vel_tmp >> 4) & 0xFF;
        joint_fdcan_send_data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
        joint_fdcan_send_data[4] = kp_tmp & 0xFF;
        joint_fdcan_send_data[5] = (kd_tmp >> 4) & 0xFF;
        joint_fdcan_send_data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
        joint_fdcan_send_data[7] = tor_tmp & 0xFF;
        HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &joint_tx_message, joint_fdcan_send_data);
    }

    /**
    @brief 设置电机零点
    @retval none
    */
    void Set_Mit_Zero_Point() const
    {

        static FDCAN_TxHeaderTypeDef joint_tx_message;
        static uint8_t joint_fdcan_send_data[8];
        joint_tx_message.Identifier = this->id;
        joint_tx_message.IdType = FDCAN_STANDARD_ID;
        joint_tx_message.TxFrameType = FDCAN_DATA_FRAME;
        joint_tx_message.DataLength = 0x08;
        joint_fdcan_send_data[0] = 0xFF;
        joint_fdcan_send_data[1] = 0xFF;
        joint_fdcan_send_data[2] = 0xFF;
        joint_fdcan_send_data[3] = 0xFF;
        joint_fdcan_send_data[4] = 0xFF;
        joint_fdcan_send_data[5] = 0xFF;
        joint_fdcan_send_data[6] = 0xFF;
        joint_fdcan_send_data[7] = 0xFE;

        HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &joint_tx_message, joint_fdcan_send_data);
    }

    /**
     * @brief 获取该对象电机测量数据
     * @param data 接收到的CAN数据
     */
    void get_joint_motor_measure(uint8_t data[8])
    {
        this->last_pos = this->pos;
        this->p_int = (uint16_t)((data)[1] << 8 | (data)[2]);
        this->v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));
        this->t_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
        this->pos = (fp32)(this->p_int / 65535.0f) * 2 * P_MAX - P_MAX;
        if (this->pos - this->last_pos >= P_MAX)
        {
            this->count--;
        }
        else if (this->pos - this->last_pos <= P_MIN)
        {
            this->count++;
        }
        this->total_pos = 2 * P_MAX * this->count + this->pos;
        this->vel = (fp32)(this->v_int / 4095.0f) * 2 * V_MAX - V_MAX;
        this->tor = (fp32)(this->t_int / 4095.0f) * 2 * T_MAX - T_MAX;
    }

    /**
     * @brief          获取电机对象指针
     * @param[in]      none
     * @return			返回对应电机对象的指针
     * @retval         none
     */
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
    fp32 chassis_yaw_set;
    fp32 chassis_relative_angle;
    uint8_t fric_state;
    chassis_mode_e chassis_behaviour_mode;

    /**
     * @brief          获取云台对象指针
     * @param[in]      none
     * @return			返回对应云台对象的指针
     * @retval         none
     */
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

extern Joint_Motor_Measure chassis_joint[4];

extern Wheel_Motor_Measure chassis_wheel[2];

extern Gimbal_Data gimbal_data;

/**
 * @brief          接收云台数据并解包
 * @param[in]      received_data: 云台发送的8字节CAN数据
 * @retval         none
 * @note           已在FDCAN中断中接入，打包协议:
 *                 [0..1] v_set, [2..3] turn_set, [4] mode_byte,
 *                 [5] fric_state, [6..7] yaw_err_mrad.
 *                 yaw_err_mrad = forward_middle_yaw - current_yaw,
 *                 unit: 0.001 rad, range: [-PI, PI].
 */
inline void CAN_cmd_gimbal_receive(const uint8_t *received_data)
{
    if (received_data == nullptr)
    {
        return;
    }

    const int16_t v_set_tmp = static_cast<int16_t>((received_data[0] << 8) | received_data[1]);
    const int16_t yaw_set_tmp = static_cast<int16_t>((received_data[2] << 8) | received_data[3]);
    const chassis_mode_e chassis_behaviour = static_cast<chassis_mode_e>(received_data[4]);
    const uint8_t fric_state = received_data[5];
    const int16_t relative_angle_tmp = static_cast<int16_t>((received_data[6] << 8) | received_data[7]);

    gimbal_data.v_tmp = static_cast<fp32>(v_set_tmp) / 1000.0f;
    gimbal_data.chassis_yaw_set = -static_cast<fp32>(yaw_set_tmp) / 1000.0f;
    gimbal_data.chassis_relative_angle = static_cast<fp32>(relative_angle_tmp) / 1000.0f;
    if (gimbal_data.chassis_relative_angle > PI)
    {
        gimbal_data.chassis_relative_angle = PI;
    }
    else if (gimbal_data.chassis_relative_angle < -PI)
    {
        gimbal_data.chassis_relative_angle = -PI;
    }
    gimbal_data.fric_state = fric_state;
    gimbal_data.chassis_behaviour_mode = chassis_behaviour;
}

extern void CAN_cmd_wheel(const fp32 motor1, const fp32 motor2);

extern void CAN_cmd_steering(const int16_t motor1, const int16_t motor2);

#endif

/** * @brief 函数外部声明 */
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
