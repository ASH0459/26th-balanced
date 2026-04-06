#include "App_UI_Task.h"
#include "ui.h"
#include "main.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "ui_interface.h"
#include "UC_Referee.h"
#include "Dev_Can_Receive.h"
#include "HDL_SuperCap.hpp"

uint16_t last_key_ctrl = 0;

void UI_Task(void *argument)
{

  osDelay(2000);
  get_robot_id(&ui_self_id, 0);
  osDelay(100);
  ui_init_g_move();
  osDelay(100);
  _ui_init_g_text_0();
  osDelay(100);
  _ui_init_g_text_1();
  osDelay(100);
  ui_init_g_Ungroup();
  osDelay(100);

  while (1)
  {

    double capac = SuperCap_Data.Data.capEnergy * 0.01;
    ui_g_move_cap->start_x = 163.0;
    ui_g_move_cap->end_x  = 163.0 + 541.0f * capac;
    //ui_g_move_distance->number = SuperCap_Data.Data.capEnergy  * 1000;
    ui_g_move_distance->number = chassis_move.chassis_left_control.wbr_control.theta;

    if (gimbal_data.key & GYROSCOPE_KEY)
    {
      ui_g_move_gyroton->number = 1;
    }
    else
    {
      ui_g_move_gyroton->number = 0;
    }
    //  // 检测CTRL键是否被按下
    if ((gimbal_data.key & KEY_PRESSED_OFFSET_SHIFT) &&
        !(last_key_ctrl & KEY_PRESSED_OFFSET_SHIFT))
    {
            // CTRL键按下，重置UI任务
        UI_Task_Reset();
    }
        // 保存当前按键状态，用于下次检测按键变化
    //last_key_ctrl = gimbal_control.gimbal_rc_ctrl->key.v;
    last_key_ctrl = gimbal_data.key;

    osDelay(50);
    _ui_update_g_move_0();
  }
}

/**
 * @brief          重新初始化UI任务
 * @param[in]      none
 * @retval         none
 */
void UI_Task_Reset(void)
{

  osDelay(2000);
  get_robot_id(&ui_self_id, 0);
  osDelay(100);
  ui_init_g_move();
  osDelay(100);
  _ui_init_g_text_0();
  osDelay(100);
  _ui_init_g_text_1();
  osDelay(100);
  ui_init_g_Ungroup();
  osDelay(100);
  
  // 立即更新一次UI显示
  _ui_update_g_move_0();
}
