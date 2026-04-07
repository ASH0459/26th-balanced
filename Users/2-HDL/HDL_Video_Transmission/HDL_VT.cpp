/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_VT.cpp
  * @brief      Video Transmission Module图传模块串口通信驱动
  * @note       在串口空闲中断中调用处理函数，处理函数拷贝原始数据之后写入队列
  *             上层解析函数接收队列数据，在任务完成解析
  *             Hardware Driver Layer 硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-27-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考中科大2024年工程机器人电控代码开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "HDL_VT.h"

/**
 * @brief 创建图传对象
 */
Class_Video_Transmission VT_Module;


/**
 * @brief          初始化图传串口、创建队列
 * @param[in]      *UART_Manage_Obj 串口实例对象
 * @retval         none
 */

void Class_Video_Transmission::Init(UART_Manage_Object_t *_UART_Manage_Obj_)
{
    this->UART_Mangae_Obj = _UART_Manage_Obj_;
    // 应在RTOS内核调度启动后创建
    this->custom_robot.xcustom_queue = xQueueCreate(QUEUE_RX_ITEM_NUM, sizeof(custom_robot.data));
    this->remote_robot.xremote_queue = xQueueCreate(QUEUE_RX_ITEM_NUM, sizeof(remote_robot.data));
    this->vt_rc_robot.xrc_vt_queue   = xQueueCreate(QUEUE_RX_ITEM_NUM, sizeof(vt_rc_robot.data ));
}


/**
 * @brief          图传链路数据处理
 * @param[in]      *data    串口数据
 * @retval         1:解析成功
 *                 0:解析失败
 */
void Class_Video_Transmission::VT_Data_Processing(uint8_t *data, uint16_t length)
{
    auto data_head = data;

    while((data - data_head) < length)
    {
        if(*data == 0xA9 && *(data+1) == 0x53) // 寻找帧头
        {
            memcpy(&vt_rc_robot.data, data, sizeof(vt_rc_robot.data));
            xQueueSendFromISR(vt_rc_robot.xrc_vt_queue, &vt_rc_robot.data, NULL);
            data++;
        }
        else if(*data == 0xA5) // 寻找帧头
        {
            memcpy(&Frame_header, data, sizeof(Frame_header));
            memcpy(&cmd_id, data + sizeof(Frame_header), sizeof(cmd_id));

            switch (cmd_id)
            {
                case 0x0302:// 自定义控制器
                    memcpy(&custom_robot.data, data + sizeof(Frame_header) + sizeof(cmd_id), sizeof(custom_robot.data));
                    xQueueSendFromISR(custom_robot.xcustom_queue, &custom_robot.data, NULL);
                    data++;
                    break;

                case 0x0304:// 键鼠数据
                    memcpy(&remote_robot.data, data + sizeof(Frame_header) + sizeof(cmd_id), sizeof(remote_robot.data));
                    xQueueSendFromISR(remote_robot.xremote_queue, &remote_robot.data, NULL);
                    data++;
                    break;

                default:
                    data++;
                    break;
            }
        }
        else
        {
            data++;
        }
    }
}


///**
// * @brief          获取自定义控制器0x302的队列句柄
// * @param[in]      none
// * @retval         对应的句柄
// */
//QueueHandle_t Class_Video_Transmission::xGet_Custom_Queue(void)
//{
//    return custom_robot.xCustom_Queue;
//}
//
///**
// * @brief          获取键鼠数据0x304的队列句柄
// * @param[in]      none
// * @retval         对应的句柄
// */
//QueueHandle_t Class_Video_Transmission::xGet_RC_Queue(void)
//{
//    return remote_control.xRC_Queue;
//}
//
//
///**
// * @brief          获取新图传键鼠+摇杆数据的队列句柄
// * @param[in]      none
// * @retval         对应的句柄
// */
//QueueHandle_t Class_Video_Transmission::xGet_RC_VT_Queue(void)
//{
//    return VT_remote_control.xRC_VT_Queue;
//}
