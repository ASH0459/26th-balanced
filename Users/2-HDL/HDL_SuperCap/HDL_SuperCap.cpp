/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_SuperCap.cpp
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-2-2      FenZhong        1. 超级电容头文件
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "HDL_SuperCap.hpp"

/* 存超电数据的全局变量 */
SuperCap SuperCap_Data;

/* 超电 CAN 数据发送句柄 */
static FDCAN_TxHeaderTypeDef  SuperCap_tx_message;

/**
************************************************************************
* @brief:      	SuperCap_Init: 超级电容初始化
* @param[in]:   void
* @retval:     	void
* @details:    	初始化超级电容的发送配置参数
************************************************************************
**/
void SuperCap_Init() {
        SuperCap_TX_Config(&SuperCap_Data,
                           1,
                           5,
                           57,
                           1,
                           0,
                           0
                           );
}

/**
************************************************************************
* @brief:      	SuperCap_TX_Config: 配置发送给超级电容的数据
* @param[out]:  data: 超级电容结构体指针 (&SuperCap_Data)
* @param[in]:   enable: 是否开启DCDC (1:开启, 0:关闭) -> 正常工作填 1
* @param[in]:   power_limit: 裁判系统限制功率 (单位: W) -> 填裁判系统反馈回来的数值
* @param[in]:   energy_buffer: 裁判系统剩余缓冲 (单位: J) -> 填裁判系统反馈回来的数值
* @param[in]:   use_new_msg: 是否使用新协议 (1:新, 0:旧) -> 平时填 1
* @param[in]:   clear_err: 是否清除错误 (1:清除, 0:不清除) -> 平时填 0
* @param[in]:   restart: 是否重启超电 (1:重启, 0:不重启) -> 平时填 0
* @retval:     	void
* @details:    	配置要发送给超级电容的控制参数
************************************************************************
**/
void SuperCap_TX_Config(SuperCap* data,
                        uint8_t enable,
                        uint16_t power_limit,
                        uint16_t energy_buffer,
                        uint8_t use_new_msg,
                        uint8_t clear_err,
                        uint8_t restart)
{
    // 基础控制开关
    data->Tx.enableDCDC = enable;               // 允许DCDC启动
    data->Tx.systemRestart = restart;           // 系统重启信号
    data->Tx.clearError = clear_err;            // 清除错误信号

    // 协议选择
    data->Tx.useNewFeedbackMessage = use_new_msg; // 决定回传数据包是 0x51 还是 0x52

    // 裁判系统关键数据
    data->Tx.refereePowerLimit = power_limit;     // 告诉超电当前的功率上限
    data->Tx.refereeEnergyBuffer = energy_buffer; // 告诉超电当前的缓冲能量

    // 默认设置
    data->Tx.resv0 = 0;                         // 保留位
    data->Tx.resv2 = 0;                         // 保留位

    // 默认关闭“主动充电限制”功能，让超电自己管理充电
    // 如果你有特殊需求（比如发射时不想让超电抢功率），可以单独写代码修改这两个值
    data->Tx.enableActiveChargingLimit = 0;
    data->Tx.activeChargingLimitRatio = 0;
}

/**
************************************************************************
* @brief:      	ParseSuperCapRx: 解析超电接收数据包(旧协议 0x51)
* @param[in]:   rx_data: 接收到的CAN数据
* @param[out]:  data: 超级电容数据结构体指针
* @retval:     	void
* @details:    	解析旧协议格式的超级电容反馈数据
************************************************************************
**/
void ParseSuperCapRx(volatile const uint8_t *rx_data, SuperCap* data) {

    // 解析接收到的数据
    data->Rx.statusCode        = rx_data[0];
    memcpy(&data->Rx.chassisPower, (const void*)&rx_data[1], 4);
    data->Rx.chassisPowerLimit = (uint16_t)(rx_data[5] | (rx_data[6] << 8));
    data->Rx.capEnergy         = rx_data[7];

    // 转换为实际物理量
    data->Data.chassisPower      = data->Rx.chassisPower;
    data->Data.refereePower      = 0.0f; // 旧协议无裁判功率，置0
    data->Data.chassisPowerLimit = (float)data->Rx.chassisPowerLimit;
    data->Data.capEnergy         = ((float)data->Rx.capEnergy * 100.0f) / 255.0f;
}

/**
************************************************************************
* @brief:      	ParseSuperCapRxNew: 解析超电接收数据包(新协议 0x52)
* @param[in]:   rx_data: 接收到的CAN数据
* @param[out]:  data: 超级电容数据结构体指针
* @retval:     	void
* @details:    	解析新协议格式的超级电容反馈数据
************************************************************************
**/
void ParseSuperCapRxNew(volatile const uint8_t *rx_data, SuperCap* data) {

    // 解析接收到的数据
    data->RxNew.statusCode        = rx_data[0];
    data->RxNew.chassisPower      = (uint16_t)(rx_data[1] | (rx_data[2] << 8));
    data->RxNew.refereePower      = (uint16_t)(rx_data[3] | (rx_data[4] << 8));
    data->RxNew.chassisPowerLimit = (uint16_t)(rx_data[5] | (rx_data[6] << 8));
    data->RxNew.capEnergy         = rx_data[7];

    // 转换为实际物理量
    data->Data.chassisPower      = ((float)data->RxNew.chassisPower - 16384) / 64.0f;
    data->Data.refereePower      = ((float)data->RxNew.refereePower - 16384) / 64.0f;
    data->Data.chassisPowerLimit = ((float)data->RxNew.chassisPowerLimit - 16384) / 64.0f;
    data->Data.capEnergy         = ((float)data->RxNew.capEnergy * 100.0f) / 255.0f;
}

/**
************************************************************************
* @brief:      	SuperCap_RX_Callback: 超级电容接收回调
* @param[in]:   void
* @retval:     	void
* @details:    	处理超级电容的CAN接收中断，根据ID选择对应的解析函数
************************************************************************
**/
void SuperCap_RX_Callback(uint8_t rx_data[8])
{
    /* 将接收的数据存储到缓冲区 */
    // FDCAN_RxHeaderTypeDef rx_header;
    // //uint8_t rx_data[64];
    // HAL_FDCAN_GetRxMessage(&SuperCap_CAN, FDCAN_RX_FIFO0, &rx_header, rx_data);

    // switch (rx_header.Identifier)
    // {
    //     case SuperCap_1ID:
    //     {
    //         // 旧协议 0x51 数据解析
    //         ParseSuperCapRx(rx_data, &SuperCap_Data);
    //         break;
    //     }
    //     case SuperCap_2ID:
    //     {
            // 新协议 0x52 数据解析
    ParseSuperCapRxNew(rx_data, &SuperCap_Data);
    //         break;
    //     }
    //     default: {
    //         break;
    //     }
    // }
}

/**
************************************************************************
* @brief:      	Get_SuperCap_Data_Point: 获取超级电容数据结构体指针
* @param[in]:   void
* @retval:     	SuperCap*: 超级电容数据结构体指针
* @details:    	返回超级电容全局数据结构体地址
************************************************************************
**/
SuperCap *Get_SuperCap_Data_Point(void)
{
    return &SuperCap_Data;
}

/**
************************************************************************
* @brief:      	FDCAN_cmd_SuperCap: 超电数据发送函数
* @param[in]:   hfdcan: FDCAN句柄
* @retval:     	void
* @details:    	通过FDCAN发送控制指令给超级电容
************************************************************************
**/
void FDCAN_cmd_SuperCap (FDCAN_HandleTypeDef *hfdcan)
{
    /* 定义消息 */
    SuperCap_tx_message.Identifier  = SuperCap_CAN_ID;
    SuperCap_tx_message.IdType      = FDCAN_STANDARD_ID;
    SuperCap_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    SuperCap_tx_message.DataLength  = 0x08;

    /* 发送消息 */
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &SuperCap_tx_message, reinterpret_cast<const uint8_t*>(&SuperCap_Data.Tx));
}

/**
************************************************************************
* @brief:      	STM32_to_SuperCap: STM32发送数据到超级电容
* @param[in]:   void
* @retval:     	void
* @details:    	调用FDCAN_cmd_SuperCap发送数据
************************************************************************
**/
void STM32_to_SuperCap(void)
{
    FDCAN_cmd_SuperCap(&SuperCap_CAN);
}

