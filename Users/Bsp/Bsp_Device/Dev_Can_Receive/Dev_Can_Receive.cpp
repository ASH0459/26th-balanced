/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file   Dev_Can_Receive.cpp
  * @brief
  * @note
  * @history
  * Version   Date   Author   Modification
  * V2.0.0    11-22-2025   无垠   1. continue
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
/** * @brief 头文件 */
#include "Dev_Can_Receive.h"
#include "App_Detect_Task.h"
#include "App_Chassis_Task.h"

#include "main.h"
#include "HDL_SuperCap.hpp"
#include "UC_Referee.h"
#include "arm_math.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/******************************* 工具函数 *******************************/

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

// 将 4 位无符号值符号扩展为 int8_t (0x0~0xF -> -8~7)
static int8_t sign_extend_4bit(uint8_t val4)
{
    return static_cast<int8_t>(val4 | ((val4 & 0x08) ? 0xF0 : 0x00));
}

/******************************* 类方法实现 *******************************/

void Wheel_Motor_Measure::get_motor_measure(uint8_t data[8])
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

void Joint_Motor_Measure::Enable_Joint_Motor() const
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

void Joint_Motor_Measure::Joint_MIT_Control(const float pos, const float vel, const float kp, const float kd, float torq) const
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

void Joint_Motor_Measure::Set_Mit_Zero_Point() const
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

void Joint_Motor_Measure::get_joint_motor_measure(uint8_t data[8])
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

/******************************* 协议解析 *******************************/

bool_t chassis_mode_is_valid(const uint8_t mode_byte)
{
    return mode_byte <= CHASSIS_MODE_STEP_2;
}

void CAN_cmd_gimbal_receive(const uint8_t *received_data)
{
    if (received_data == nullptr)
    {
        return;
    }

    gimbal_can_cmd_frame_t frame{};
    frame.v_set = static_cast<int16_t>((received_data[0] << 8) | received_data[1]);
    frame.auto_aim_state = received_data[2];
    frame.chassis_feature_flags = received_data[3];
    frame.mode = (received_data[4] >> 4) & 0x0F;
    frame.fric_state = received_data[4] & 0x0F;
    frame.yaw_offset = sign_extend_4bit((received_data[5] >> 4) & 0x0F);
    frame.pitch_offset = sign_extend_4bit(received_data[5] & 0x0F);
	// frame.mode = received_data[4];
	// frame.fric_state = received_data[5];
    frame.turn_set = static_cast<int16_t>((received_data[6] << 8) | received_data[7]);

    const bool_t mode_valid = chassis_mode_is_valid(frame.mode);
    const bool_t auto_aim_valid =
        (frame.auto_aim_state == CHASSIS_AUTO_AIM_STATE_NO_TARGET ||
         frame.auto_aim_state == CHASSIS_AUTO_AIM_STATE_MANUAL_FIRE_TARGET ||
         frame.auto_aim_state == CHASSIS_AUTO_AIM_STATE_AUTO_FIRE_TARGET);
    gimbal_data.protocol_valid = static_cast<uint8_t>(mode_valid && auto_aim_valid);

    const Fric_State_e fric_state =
        (frame.fric_state == FRIC_OFF || frame.fric_state == FRIC_ON ||
         frame.fric_state == FRIC_ERROR)
            ? static_cast<Fric_State_e>(frame.fric_state)
            : FRIC_ERROR;

    gimbal_data.v_tmp = static_cast<fp32>(frame.v_set) / 1000.0f;
    gimbal_data.yaw_set = static_cast<fp32>(frame.turn_set) / 1000.0f;
    if (gimbal_data.yaw_set > PI)
    {
        gimbal_data.yaw_set = PI;
    }
    else if (gimbal_data.yaw_set < -PI)
    {
        gimbal_data.yaw_set = -PI;
    }
    gimbal_data.auto_aim_state =
        auto_aim_valid ? frame.auto_aim_state : CHASSIS_AUTO_AIM_STATE_NO_TARGET;
    gimbal_data.chassis_feature_flags = frame.chassis_feature_flags;
    gimbal_data.gyro_enable =
        ((frame.chassis_feature_flags & CHASSIS_FEATURE_FLAG_GYRO_ENABLE) != 0U) ? 1U : 0U;
    gimbal_data.step_enable =
        ((frame.chassis_feature_flags & CHASSIS_FEATURE_FLAG_STEP) != 0U) ? 1U : 0U;
    gimbal_data.step_count =
        ((frame.chassis_feature_flags & CHASSIS_FEATURE_FLAG_STEP_COUNT) != 0U) ? 1U : 0U;
    gimbal_data.chassis_reset =
        ((frame.chassis_feature_flags & CHASSIS_FEATURE_FLAG_CHASSIS_RESET) != 0U) ? 1U : 0U;
    gimbal_data.chassis_behaviour_mode =
        mode_valid ? static_cast<chassis_mode_e>(frame.mode) : CHASSIS_MODE_NO_FORCE;
    gimbal_data.fric_state = fric_state;
    gimbal_data.yaw_offset = static_cast<fp32>(frame.yaw_offset) / 10.0f;
    gimbal_data.pitch_offset = static_cast<fp32>(frame.pitch_offset) / 10.0f;

    // RESERVED 模式：v_set 拆成两个标志字节，速度/转向/腿长由行为层根据标志位生成。
    if (gimbal_data.chassis_behaviour_mode == CHASSIS_MODE_RESERVED)
    {
        gimbal_data.reserved_flags = (static_cast<uint16_t>(received_data[0]) << 8) | received_data[1];
        gimbal_data.v_tmp = 0.0f;
        gimbal_data.yaw_set = 0.0f;
    }
    else
    {
        gimbal_data.reserved_flags = 0U;
    }

    if (gimbal_data.protocol_valid == 0U)
    {
        gimbal_data.v_tmp = 0.0f;
        gimbal_data.yaw_set = gimbal_data.chassis_relative_angle;
        gimbal_data.auto_aim_state = CHASSIS_AUTO_AIM_STATE_NO_TARGET;
        gimbal_data.chassis_feature_flags = 0U;
        gimbal_data.gyro_enable = 0U;
        gimbal_data.step_enable = 0U;
        gimbal_data.step_count = 0U;
        gimbal_data.chassis_reset = 0U;
        gimbal_data.chassis_behaviour_mode = CHASSIS_MODE_NO_FORCE;
    }
}

/*******************************底盘电机*******************************/

// 电机数据
/* 0，1，2，3 左1 左2 右1 右2*/
Joint_Motor_Measure chassis_joint[4] = {
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_LEFT_1_ID, &CHASSIS_JOINT_CAN1),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_LEFT_2_ID, &CHASSIS_JOINT_CAN1),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_RIGHT_1_ID, &CHASSIS_JOINT_CAN2),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_RIGHT_2_ID, &CHASSIS_JOINT_CAN2),
};

/* 0，1 左 右*/
Wheel_Motor_Measure chassis_wheel[2] = {
	Wheel_Motor_Measure(CAN_CHASSIS_WHEEL_LEFT_ID, &CHASSIS_WHEEL_CAN),	 // 0号轮电机
	Wheel_Motor_Measure(CAN_CHASSIS_WHEEL_RIGHT_ID, &CHASSIS_WHEEL_CAN), // 1号轮电机
};

Joint_Motor_Measure chassis_yaw_motor(
	CAN_CHASSIS_YAW_MOTOR_ID,
	&CHASSIS_WHEEL_CAN);

Gimbal_Data gimbal_data;

namespace
{
	static fp32 wrap_angle_to_pi(fp32 angle)
	{
		while (angle > PI)
		{
			angle -= 2.0f * PI;
		}
		while (angle < -PI)
		{
			angle += 2.0f * PI;
		}
		return angle;
	}

	static fp32 calc_relative_yaw_from_can3_motor(const Joint_Motor_Measure *yaw_motor)
	{
		if (yaw_motor == nullptr)
		{
			return 0.0f;
		}

		// DM/MIT回传pos范围是[-12.5, 12.5]，先缩放到[-PI, PI]再参与相对角计算。
		const fp32 yaw_motor_pos_rad =
			CHASSIS_YAW_MOTOR_DIR_SIGN * yaw_motor->pos * (PI / P_MAX);
		return wrap_angle_to_pi(yaw_motor_pos_rad - YAW_CENTER_POS_RAD);
	}

	constexpr uint8_t JOINT_COMM_LOST_ERR_CODE = 0x0DU;
	constexpr uint32_t JOINT_CLEAR_ERR_GAP_MS = 10U;
	constexpr uint16_t JOINT_CLEAR_ERR_MODE_ID = 0U;

	static void clear_err(FDCAN_HandleTypeDef *hcan, const uint16_t motor_id, const uint16_t mode_id)
	{
		if (hcan == nullptr)
		{
			return;
		}

		uint8_t data[8];
		const uint16_t id = static_cast<uint16_t>(motor_id + mode_id);
		data[0] = 0xFF;
		data[1] = 0xFF;
		data[2] = 0xFF;
		data[3] = 0xFF;
		data[4] = 0xFF;
		data[5] = 0xFF;
		data[6] = 0xFF;
		data[7] = 0xFB;

		FDCAN_TxHeaderTypeDef tx_header;
		tx_header.Identifier = id;
		tx_header.IdType = FDCAN_STANDARD_ID;
		tx_header.TxFrameType = FDCAN_DATA_FRAME;
		tx_header.DataLength = FDCAN_DLC_BYTES_8;
		tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		tx_header.MessageMarker = 0;

		HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data);
	}

	static bool_t is_joint_comm_lost_error(const uint8_t status_byte)
	{
		const uint8_t err_code_low = status_byte & 0x0FU;
		const uint8_t err_code_high = (status_byte >> 4) & 0x0FU;
		return (err_code_low == JOINT_COMM_LOST_ERR_CODE) || (err_code_high == JOINT_COMM_LOST_ERR_CODE);
	}

	static bool_t can_clear_joint_motor_error_now(void)
	{
		return (chassis_move.posture == CHASSIS_POSTURE_UP) &&
			   (gimbal_data.protocol_valid != 0U) &&
			   (gimbal_data.chassis_behaviour_mode == CHASSIS_MODE_NO_FORCE);
	}

	static void try_clear_joint_motor_error_if_needed(const uint8_t joint_index, const uint8_t status_byte)
	{
		static uint32_t last_clear_err_tick[4] = {0U};
		if (joint_index >= 4U)
		{
			return;
		}

		if (is_joint_comm_lost_error(status_byte) == 0U)
		{
			return;
		}

		if (can_clear_joint_motor_error_now() == 0U)
		{
			return;
		}

		const uint32_t now = HAL_GetTick();
		if ((now - last_clear_err_tick[joint_index]) < JOINT_CLEAR_ERR_GAP_MS)
		{
			return;
		}

		clear_err(chassis_joint[joint_index].hfdcan,
				  chassis_joint[joint_index].id,
				  JOINT_CLEAR_ERR_MODE_ID);
		last_clear_err_tick[joint_index] = now;
	}
} // namespace

/**
 * @brief          发送轮电机控制电流(0x201,0x202)
 * @param[in]      motor1: (0x201) 左边轮电机控制扭矩, 范围 [-5,5]
 * @param[in]      motor2: (0x202) 右边轮电机控制电流, 范围 [-5,5]
 * @retval         none
 */
void CAN_cmd_wheel(const fp32 motor1, const fp32 motor2)
{
	int16_t motor1_current = (int16_t)(motor1 * CONSTANT_OF_TORQUE);
	int16_t motor2_current = (int16_t)(motor2 * CONSTANT_OF_TORQUE);

	uint16_t shoot_heat = 0;
	uint16_t shoot_heat_limit = 0;
	uint16_t shoot_cooling_value = 0;
	float bullet_speed_f = 0.0f;
	get_shoot_heat1_limit_and_heat1(&shoot_heat_limit, &shoot_heat);
	get_shoot_cooling_value1(&shoot_cooling_value);
	get_bullet_speed(&bullet_speed_f);

	const uint16_t packed_heat = (shoot_heat > 0x01FFU) ? 0x01FFU : shoot_heat;
	const uint16_t packed_heat_limit = (shoot_heat_limit > 0x01FFU) ? 0x01FFU : shoot_heat_limit;
	const uint16_t packed_bullet_speed = ((uint16_t)(bullet_speed_f * 5.0f) > 0x00FFU) ? 0x00FFU : (uint16_t)(bullet_speed_f * 5.0f);
	const uint8_t packed_cooling_value =
		(shoot_cooling_value > 0x003FU) ? 0x003FU : (uint8_t)shoot_cooling_value;

	// bytes[4..7] bit layout: [heat_limit(9b) | heat(9b) | bullet_speed(8b, x0.2m/s) | cooling_value(6b)]
	const uint32_t heat_payload =
		(((uint32_t)packed_heat_limit) << 23) |
		(((uint32_t)packed_heat) << 14) |
		(((uint32_t)packed_bullet_speed) << 6) |
		((uint32_t)packed_cooling_value);

	FDCAN_TxHeaderTypeDef TxHeader;
	static uint8_t chassis_can_send_data[8];

	TxHeader.Identifier = CAN_CHASSIS_WHEEL_ALL_ID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	chassis_can_send_data[0] = motor1_current >> 8;
	chassis_can_send_data[1] = motor1_current;
	chassis_can_send_data[2] = motor2_current >> 8;
	chassis_can_send_data[3] = motor2_current;
	chassis_can_send_data[4] = (uint8_t)(heat_payload >> 24);
	chassis_can_send_data[5] = (uint8_t)(heat_payload >> 16);
	chassis_can_send_data[6] = (uint8_t)(heat_payload >> 8);
	chassis_can_send_data[7] = (uint8_t)(heat_payload);

	HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_WHEEL_CAN, &TxHeader, chassis_can_send_data);
}

/*******************************中断*******************************/

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (hfdcan->Instance == FDCAN1)
		{
			FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.Identifier)
			{
			case CAN_CHASSIS_JOINT_LEFT_1_ID:
			case CAN_CHASSIS_JOINT_LEFT_2_ID:
			{
				const uint8_t i =
					(rx_header.Identifier == CAN_CHASSIS_JOINT_LEFT_1_ID) ? 0U : 1U;
				chassis_joint[i].get_joint_motor_measure(rx_data); // 对应电机获取对应值
				try_clear_joint_motor_error_if_needed(i, rx_data[0]);
				// D[0]高4位为电机ID，低4位为错误码；错误码为0时才认为电机在线
				uint8_t err_code = rx_data[0] & 0xF0;
				if (err_code == 0x10)
				{
					detect_hook(CHASSIS_JOINT1_TOE + i); // 设备检测函数，检测设备是否掉线
				}
				break;
			}
			case SuperCap_1ID:
			case SuperCap_2ID:
			{
				SuperCap_RX_Callback(rx_data);
				detect_hook(SuperCap_TOE);
			}

			default:
			{
				break;
			}
			}
		}

		else if (hfdcan->Instance == FDCAN3)
		{
			FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.Identifier)
			{
			case CAN_CHASSIS_WHEEL_LEFT_ID:
			case CAN_CHASSIS_WHEEL_RIGHT_ID: // 轮电机数据
			{
				static uint8_t i = 0;
				// get motor id
				i = rx_header.Identifier - CAN_CHASSIS_WHEEL_LEFT_ID;
				chassis_wheel[i].get_motor_measure(rx_data); // 对应电机获取对应值
				detect_hook(CHASSIS_WHEEL1_TOE + i);		 // 设备检测函数，检测设备是否掉线
				break;
			}
			case CAN_CHASSIS_YAW_MOTOR_ID:
			{
				chassis_yaw_motor.get_joint_motor_measure(rx_data);
				gimbal_data.chassis_relative_angle = calc_relative_yaw_from_can3_motor(&chassis_yaw_motor);
				break;
			}
			case CAN_VT_ID: // 云台数据(新双板CAN协议)
			case GIMBAL_ID:
			{
				CAN_cmd_gimbal_receive(rx_data);
				detect_hook(VT_TOE);
				break;
			}

			default:
			{
				break;
			}
			}
		}
		else
		{
			Error_Handler();
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if (hfdcan->Instance == FDCAN2)
		{
			FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
			switch (rx_header.Identifier)
			{
			case CAN_CHASSIS_JOINT_RIGHT_1_ID:
			case CAN_CHASSIS_JOINT_RIGHT_2_ID:
			{
				const uint8_t i =
					(rx_header.Identifier == CAN_CHASSIS_JOINT_RIGHT_1_ID) ? 2U : 3U;
				chassis_joint[i].get_joint_motor_measure(rx_data); // 对应电机获取对应值
				try_clear_joint_motor_error_if_needed(i, rx_data[0]);
				// D[0]高4位为电机ID，低4位为错误码；错误码为0时才认为电机在线
				uint8_t err_code = rx_data[0] & 0xF0;
				if (err_code == 0x10)
				{
					detect_hook(CHASSIS_JOINT1_TOE + i); // 设备检测函数，检测设备是否掉线
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		else
		{
			Error_Handler();
		}
	}
}
