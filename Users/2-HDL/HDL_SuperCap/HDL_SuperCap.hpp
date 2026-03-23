/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_SuperCap.hpp
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
#pragma once

/**
 * @brief 头文件
 */
#include "fdcan.h"
#include "string.h"
/**
 * @brief 宏定义
 */

#define SuperCap_CAN hfdcan1

/**
 * @brief 结构体
 */

/* 主控板接受超电的数据 */

typedef struct {                 // 0x051 (useNewFeedbackMessage = 0)
 uint8_t statusCode;             // 状态信息
 float chassisPower;             // 底盘功率，单位W
 uint16_t chassisPowerLimit;     // 底盘最大可用功率（包括裁判系统）
 uint8_t capEnergy;              // 电容现有能量，0-255
} SuperCap_Rx_t;

typedef struct {              // 0x052 (useNewFeedbackMessage = 1)
 uint8_t statusCode;          // 状态信息
 uint16_t chassisPower;       // 底盘功率，功率*64+16384 (-256W~+768W, 精度 0.015625)
 uint16_t refereePower;       // 裁判系统功率，功率*64+16384 (-256W~+768W, 精度 0.015625)
 uint16_t chassisPowerLimit;  // 底盘最大可用功率（包括裁判系统）
 uint8_t capEnergy;           // 电容现有能量，0-255
} SuperCap_RxNew_t;


/* 主控板发送给超电的数据 */
#pragma pack(1)
typedef struct {
 uint8_t enableDCDC: 1;                      // 允许启动 DCDC
 uint8_t systemRestart: 1;                   // 系统重启
 uint8_t resv0: 3;
 uint8_t clearError: 1;                      // 手动清除可清除的错误
 uint8_t enableActiveChargingLimit: 1;       // 是否启用主动充电限制
 uint8_t useNewFeedbackMessage: 1;           // 是否使用新的反馈消息格式
 uint16_t refereePowerLimit;                 // 裁判限制功率，单位 W
 uint16_t refereeEnergyBuffer;               // 裁判能量缓冲，单位 J
 uint8_t activeChargingLimitRatio;           // 主动充电限制比例（能量），0-255
 int16_t resv2;
} SuperCap_Tx_t;
#pragma pack()

/* 主控板解析超电数据之后的真实数据 */
typedef struct {
 float chassisPower;                  // 底盘功率
 float refereePower;                  // 裁判系统功率
 float chassisPowerLimit;             // 底盘最大可用功率（包括裁判系统）
 float capEnergy;                     // 电容现有能量，按百分比
} SuperCap_Data_t;

/* 超电数据结构体 */
typedef struct {
 SuperCap_Rx_t      Rx;      // 接受超电的数据  (旧协议 0x51)
 SuperCap_RxNew_t   RxNew;   // 接受超电的数据  (新协议 0x52)
 SuperCap_Tx_t      Tx;      // 发送给超电的数据
 SuperCap_Data_t    Data;    // 解析后的超电数据
} SuperCap;

typedef enum
{

 SuperCap_CAN_ID  = 0x061,

 SuperCap_1ID     = 0x51,
 SuperCap_2ID     = 0x52,

}Super_msg_id_e;

/**
 * @brief 变量外部声明
 */
extern FDCAN_RxHeaderTypeDef rx_header;
extern uint8_t rx_data[64];
extern SuperCap SuperCap_Data;

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
************************************************************************
* @brief:      	ParseSuperCapRx: 解析超级电容接收数据
* @param[in]:   rx_data: 接收到的CAN数据
* @param[out]:  data: 超级电容数据结构体指针
* @retval:     	void
* @details:    	解析从超级电容接收到的CAN数据并更新到结构体中
************************************************************************
**/
extern void ParseSuperCapRx(volatile const uint8_t *rx_data, SuperCap* data);

/**
************************************************************************
* @brief:      	SuperCap_RX_Callback: 超级电容接收回调
* @param[in]:   void
* @retval:     	void
* @details:    	处理超级电容的CAN接收中断
************************************************************************
**/
extern void SuperCap_RX_Callback(uint8_t rx_data[8]);

/**
************************************************************************
* @brief:      	SuperCap_TX_Config: 配置超级电容发送数据
* @param[out]:  data: 超级电容数据结构体指针
* @param[in]:   enable: 是否使能DCDC
* @param[in]:   power_limit: 裁判系统功率限制
* @param[in]:   energy_buffer: 裁判系统能量缓冲
* @param[in]:   use_new_msg: 是否使用新消息格式
* @param[in]:   clear_err: 是否清除错误
* @param[in]:   restart: 是否重启系统
* @retval:     	void
* @details:    	配置要发送给超级电容的控制参数
************************************************************************
**/
extern void SuperCap_TX_Config(SuperCap* data,
                         uint8_t enable,
                         uint16_t power_limit,
                         uint16_t energy_buffer,
                         uint8_t use_new_msg,
                         uint8_t clear_err,
                         uint8_t restart);

/**
************************************************************************
* @brief:      	SuperCap_Init: 超级电容初始化
* @param[in]:   void
* @retval:     	void
* @details:    	初始化超级电容相关的参数和状态
************************************************************************
**/
extern void SuperCap_Init(void);

/**
************************************************************************
* @brief:      	FDCAN_cmd_SuperCap: 发送超级电容控制指令
* @param[in]:   hfdcan: FDCAN句柄
* @retval:     	void
* @details:    	通过FDCAN发送控制指令给超级电容
************************************************************************
**/
extern void FDCAN_cmd_SuperCap (FDCAN_HandleTypeDef *hfdcan);

/**
************************************************************************
* @brief:      	STM32_to_SuperCap: STM32发送数据到超级电容
* @param[in]:   void
* @retval:     	void
* @details:    	将STM32的控制数据打包并发送给超级电容
************************************************************************
**/
extern void STM32_to_SuperCap(void);


#ifdef __cplusplus
}
#endif