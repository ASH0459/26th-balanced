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

#include <bits/range_access.h>

#include "main.h"
#include "arm_math.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/*******************************底盘电机*******************************/

// 电机数据
/* 0，1，2，3 左1 左2 右1 右2*/
Joint_Motor_Measure chassis_joint[4] = {
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_LEFT_1_ID, &CHASSIS_JOINT_CAN),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_LEFT_2_ID, &CHASSIS_JOINT_CAN),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_RIGHT_1_ID, &CHASSIS_JOINT_CAN),
	Joint_Motor_Measure(CAN_CHASSIS_JOINT_RIGHT_2_ID, &CHASSIS_JOINT_CAN),
};

/* 0，1 左 右*/
Wheel_Motor_Measure chassis_wheel[2] = {
	Wheel_Motor_Measure(CAN_CHASSIS_WHEEL_LEFT_ID, &CHASSIS_WHEEL_CAN), //0号轮电机
	Wheel_Motor_Measure(CAN_CHASSIS_WHEEL_RIGHT_ID, &CHASSIS_WHEEL_CAN), //1号轮电机
};

Gimbal_Data gimbal_data;

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
		chassis_can_send_data[4] = 0;
		chassis_can_send_data[5] = 0;
		chassis_can_send_data[6] = 0;
		chassis_can_send_data[7] = 0;

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
			switch (rx_header.Identifier) {
				case CAN_CHASSIS_WHEEL_LEFT_ID :
				case CAN_CHASSIS_WHEEL_RIGHT_ID :
				{
					static uint8_t i = 0;
					//get motor id
					i = rx_header.Identifier - CAN_CHASSIS_WHEEL_LEFT_ID;
					chassis_wheel[i].get_motor_measure(rx_data); //对应电机获取对应值
					//detect_hook(CHASSIS_MOTOR1_TOE + i); // 设备检测函数，检测设备是否掉线
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
				case CAN_CHASSIS_JOINT_LEFT_1_ID:
				case CAN_CHASSIS_JOINT_LEFT_2_ID:
				case CAN_CHASSIS_JOINT_RIGHT_1_ID:
				case CAN_CHASSIS_JOINT_RIGHT_2_ID:
				{
					static uint8_t i = 0;
					//get motor id
					i = rx_header.Identifier - CAN_CHASSIS_JOINT_LEFT_1_ID;
					chassis_joint[i].get_joint_motor_measure(rx_data); //对应电机获取对应值
					//detect_hook(CHASSIS_MOTOR1_TOE + i); // 设备检测函数，检测设备是否掉线
					break;
				}
				case CAN_GIMBAL_ID:
				{
					gimbal_data.get_Gimbal_Data(rx_data);
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


