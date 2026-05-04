/** 
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file   APP_Buzzer.cpp
  * @brief 
  * @note 
  * @history 
  * Version   Date   Author   Modification 
  * V1.0.0    10-09-2025   无垠   1. done 
  * 
  @verbatim 
  ============================================================================== 
  * 
  ============================================================================== 
  @endverbatim 
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */ 
  /** * @brief 头文件 */ 
#include "APP_Buzzer.h"
#include "App_Chassis_Behaviour.h"
#include "App_Chassis_Task.h"
#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include "main.h"
#include "App_Detect_Task.h"

#ifdef __cplusplus
extern "C" {
#endif

  extern TIM_HandleTypeDef htim12;

  static void playStartupSound(void);
  static void buzzer_warn_beep(uint8_t num);

  static const error_t *error_list_buzzer = nullptr; // 设备离线检测错误列表指针
  static uint8_t show_num = 0;  // 剩余需要响的次数
  static uint8_t stop_num = 0;  // 每组响声之间的静音计数器

  /**
   * @brief 蜂鸣器任务主循环。
   * @note 优先级：抬腿状态 > 设备离线报警 > 静音。
   *       抬腿状态（step_enable=1）时持续低占空比响；
   *       否则根据第一个离线设备的编号响对应次数（1~10声）。
   */
  void buzzer_task(void * argument)
  {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    error_list_buzzer = get_error_list_point();

    playStartupSound();
    osDelay(1000);
    while (1)
    {
      // 抬腿模式：step_enable 置1时，持续低占空比响，跳过后续错误检测。
      if ((chassis_move.state == CHASSIS_LEG_1 || chassis_move.state == CHASSIS_LEG_2) &&
          chassis_move.chassis_gimbal_data != nullptr &&
          chassis_move.chassis_gimbal_data->step_enable)
      {
        buzzer_on(400, 1500);
        osDelay(10);
        continue;
      }

      // 遍历错误列表，找到第一个离线的设备。
      uint8_t error_num = 0;
      uint8_t error_found = 0;
      for (error_num = 0; error_num < ERROR_LIST_LENGHT; error_num++)
      {
        if (error_list_buzzer[error_num].error_exist)
        {
          error_found = 1;
          break;
        }
      }

      // 有错误则按设备编号响对应次数，无错误则关闭蜂鸣器并重置状态。
      if (error_found)
      {
        buzzer_warn_beep(error_num + 1);
      }
      else
      {
        show_num = 0;
        stop_num = 0;
        buzzer_off();
      }

      osDelay(10);
    }
  }

  /**
   * @brief 根据错误编号响对应次数。
   * @param num 响声次数，对应设备编号+1（DBUS=1声，VT=2声，JOINT1=3声...）。
   * @note 每声约200ms（20 ticks 静音 + 20 ticks 响），每组响声后静音约1秒（100 ticks）。
   */
  static void buzzer_warn_beep(uint8_t num)
  {
    // 首次调用：设置需要响的次数和静音计数器。
    if (show_num == 0 && stop_num == 0)
    {
      show_num = num;
      stop_num = 100;
    }
    // 静音阶段：递减计数器，蜂鸣器关闭。
    else if (show_num == 0)
    {
      stop_num--;
      buzzer_off();
    }
    // 响声阶段：tick < 20 静音，tick >= 20 响，tick >= 40 重置并减少剩余次数。
    else
    {
      static uint8_t tick = 0;
      tick++;
      if (tick < 20)
      {
        buzzer_off();
      }
      else if (tick < 40)
      {
        buzzer_on(100, 1500);
      }
      else
      {
        tick = 0;
        show_num--;
      }
    }
  }

  void playStartupSound(void)
  {
    buzzer_on(400, 1500);
    osDelay(300);
    buzzer_off();
    buzzer_on(300, 1500);
    osDelay(300);
    buzzer_off();
    buzzer_on(200, 1500);;
    osDelay(300);
    buzzer_off();
    buzzer_on(100, 1500);
    osDelay(300);
    buzzer_off();
    osDelay(300);
    buzzer_on(200, 1500);
    osDelay(200);
    buzzer_off();
  }

#ifdef __cplusplus
}
#endif
