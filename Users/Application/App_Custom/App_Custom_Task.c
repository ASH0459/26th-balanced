/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_Custom_Task.c
  * @brief      自定义控制器通信接收与解析任务
  * @note       该任务负责解析自定义控制器发送过来的数据
  *             调试测试时使用队列与任务进行通信实现解耦
  *             赛场由于裁判系统发送数据并不是单独针对自定义控制器，不再使用队列
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-31-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include <stdio.h>
#include "Dev_Custom.h"
#include "App_Custom_Task.h"
#include "Board_CRC8_CRC16.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 发送给自定义控制器的数据 */
Data_Frame_t tx_data;
uint8_t info_data[DATA_LENGTH]; // 数据段数组

/* 函数声明 */
/**
 * @brief          自定义控制器通信接收与解析任务初始化
 * @param[in]      none
 * @retval         none
 */
static void Custom_Init(void);


/**
 * @brief          自定义控制器通信数据解析
 * @param[in]      Custom_Data_t data
 * @retval         1:解析成功
 *                 0:解析失败
 */
static _Bool custom_data_solve(Custom_Data_t *data);

/**
 * @brief          图传链路键鼠通信数据解析
 * @param[in]      Remote_Data_t *rc
 * @retval         1:解析成功
 *                 0:解析失败
 */
static _Bool rc_data_solve(Custom_Data_t *rc);

/* 变量声明 */
Custom_Controller_t custom_controller;
Remote_Controller_t RC_controller; // 解析完毕，图传链路数据结构体

/**
 * @brief          自定义控制器通信任务
 * @param[in]      none
 * @retval         none
 */
void Custom_Controller_Task(void *pvParameters)
{
    /* 延迟一段时间以待底层初始化 */
    vTaskDelay(CUSTOM_TASK_INIT_TIME);

    /* 初始化 */
    Custom_Init();

    /* 获取时间戳 */
    static TickType_t detect_time = 0;
    detect_time = xTaskGetTickCount();

    while(1)
    {


        /* 读取队列数据 */
        QueueHandle_t xCustom_Queue = xGet_Custom_Queue();
        Custom_Data_t custom_info_R;

        QueueHandle_t xRC_Queue = xGet_RC_Queue();
        Custom_Data_t rc_info_R;

        /* 进行数据解析 */
        if(pdPASS == xQueueReceive(xCustom_Queue, &custom_info_R, 0))
        {
            if(custom_data_solve(&custom_info_R) == 1)
            {

            }
        }

        if(pdPASS == xQueueReceive(xRC_Queue, &rc_info_R, 0))
        {
            if(rc_data_solve(&rc_info_R) == 1)
            {
                RC_controller.data_flag = 1; // 成功解析 将数据标志置1
            }
        }

//        /* 每过5000ms 将数据标志位置0 */
//        if(xTaskGetTickCount() - detect_time >= 500)
//        {
//            RC_controller.data_flag = 0;
//            detect_time = xTaskGetTickCount();
//        }


        vTaskDelay(CUSTOM_TASK_CONTROL_TIME);
    }
}

/**
 * @brief          自定义控制器通信接收与解析任务初始化
 * @param[in]      none
 * @retval         none
 */
static void Custom_Init(void)
{

}

/**
 * @brief          图传链路0x304键鼠信息解析
 * @param[in]      rc    Remote_Controller_t结构体数据
 *                 custom_rx_data  串口收到的数据
 * @retval         none
 */
static void rc_data_analysis(Remote_Controller_t * rc, uint8_t *custom_rx_data)
{
    rc->last_left_button_down = rc->left_button_down;
    rc->last_right_button_down = rc->right_button_down;
    rc->last_keyboard_value = rc->keyboard_value;

    memcpy(&rc->mouse_x, &custom_rx_data[0], sizeof(rc->mouse_x));
    memcpy(&rc->mouse_y, &custom_rx_data[2], sizeof(rc->mouse_y));
    memcpy(&rc->mouse_z, &custom_rx_data[4], sizeof(rc->mouse_z));
    memcpy(&rc->left_button_down, &custom_rx_data[6], sizeof(rc->keyboard_value));
    memcpy(&rc->right_button_down, &custom_rx_data[7], sizeof(rc->keyboard_value));
    memcpy(&rc->keyboard_value, &custom_rx_data[8], sizeof(rc->keyboard_value));
    // 12以后的数据由于缓冲区大小不匹配，请不要读取使用


}


/**
 * @brief          自定义控制器通信数据解析
 * @param[in]      Custom_Data_t *custom
 * @retval         1:解析成功
 *                 0:解析失败
 */
static _Bool custom_data_solve(Custom_Data_t *custom)
{
    /* 检查帧头 */
    if(custom->frame_header.sof == 0xA5)
    {
        /* 检查数据帧长度 */
        if(custom->frame_header.data_length == 30)
        {
            /* 检查命令码 */
            if(custom->cmd_id == 0x0302)
            {
                // 添加解析
                return 1;
            }

        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          图传链路键鼠通信数据解析
 * @param[in]      Remote_Data_t *rc
 * @retval         1:解析成功
 *                 0:解析失败
 */
static _Bool rc_data_solve(Custom_Data_t *rc)
{
    /* 检查帧头 */
    if(rc->frame_header.sof == 0xA5)
    {
        /* 检查数据帧长度 */
        if(rc->frame_header.data_length == 12)
        {
            /* 检查命令码 */
            if(rc->cmd_id == 0x0304)
            {
                // 键鼠数据解析
                rc_data_analysis(&RC_controller, rc->data);
                return 1;
            }

        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头数据
    tx_data.frame_header.sof = 0xA5;                              // 数据帧起始字节，固定值为 0xA5
    tx_data.frame_header.data_length = data_lenth;                // 数据帧中数据段的长度
    tx_data.frame_header.seq = seq++;                             // 包序号
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); // 添加帧头 CRC8 校验位
    /// 命令码ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// 数据段
    memcpy(tx_data.data, data, data_lenth);
    /// 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}


/**
 * @brief          返回自定义控制器数组结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
Custom_Controller_t *Get_Custom_Controller_Point(void)
{
    return &custom_controller;
}

/**
 * @brief          返回图传链路数据结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
Remote_Controller_t *Get_Remote_Control_0x304_Point(void)
{
    return &RC_controller;
}
