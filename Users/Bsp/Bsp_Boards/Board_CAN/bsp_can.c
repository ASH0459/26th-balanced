#include "bsp_can.h"
#include "../../../../Core/Inc/main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static void FDCAN1_Config(void);
static void FDCAN2_Config(void);
static void FDCAN3_Config(void);

void can_filter_init(void)
{
		FDCAN1_Config();
		FDCAN2_Config();
		FDCAN3_Config();
}

static void FDCAN_Add_Reject_Filters(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterConfig = FDCAN_FILTER_REJECT; // 动作为拒收该ID

  // 滤波器0：拒收 0x01 和 0x11
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL; // DUAL模式：独立匹配两个ID
  sFilterConfig.FilterID1 = 0x01;
  sFilterConfig.FilterID2 = 0x11;
  if(HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // 滤波器1：拒收 0x1FF 和 0x205
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterID1 = 0x1FF;
  sFilterConfig.FilterID2 = 0x205;
  if(HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
  // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID;
    can_filter_st.FilterIndex = 0;
    can_filter_st.FilterType = FDCAN_FILTER_MASK;
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_st.FilterID1 = 0x00000000;
    can_filter_st.FilterID2 = 0x00000000;

  // // 1. 设置全局滤波器：接受所有未被拦截的标准帧到 FIFO0
  // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // // 2. 添加黑名单过滤规则
  // FDCAN_Add_Reject_Filters(&hfdcan3);

  if(HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter_st) != HAL_OK)
	{
        Error_Handler();
	}
    // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    // {
    //     Error_Handler();
    // }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}
