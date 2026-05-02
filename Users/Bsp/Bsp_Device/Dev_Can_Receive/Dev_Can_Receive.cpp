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
	const uint16_t shoot_heat = power_heat_data_t.shooter_17mm_barrel_heat;
	const uint16_t shoot_heat_limit = robot_state.shooter_barrel_heat_limit;
	const uint16_t shoot_cooling_value = robot_state.shooter_barrel_cooling_value;

	const uint16_t packed_heat = (shoot_heat > 0x0FFFU) ? 0x0FFFU : shoot_heat;
	const uint16_t packed_heat_limit = (shoot_heat_limit > 0x0FFFU) ? 0x0FFFU : shoot_heat_limit;
	const uint8_t packed_cooling_value =
		(shoot_cooling_value > 0x00FFU) ? 0x00FFU : (uint8_t)shoot_cooling_value;

	// bytes[4..7] bit layout: [heat(12b) | heat_limit(12b) | cooling_value(8b)]
	const uint32_t heat_payload =
		(((uint32_t)packed_heat) << 20) |
		(((uint32_t)packed_heat_limit) << 8) |
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
