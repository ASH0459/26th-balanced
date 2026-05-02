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

  void buzzer_task(void * argument)
  {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    playStartupSound();
    osDelay(1000);
    while (1)
    {
      //playStartupSound();
       if (toe_is_error(CHASSIS_JOINT1_TOE) || toe_is_error(CHASSIS_JOINT2_TOE) ||
             toe_is_error(CHASSIS_JOINT3_TOE) || toe_is_error(CHASSIS_JOINT4_TOE)) {
         buzzer_on(100, 1500);
         osDelay(300);
         buzzer_off();
         osDelay(300);
         buzzer_on(100, 1500);
         osDelay(300);
         buzzer_off();
         osDelay(300);
         buzzer_on(100, 1500);
         osDelay(300);
         buzzer_off();
         osDelay(2000);
       }
       // else if (toe_is_error(CHASSIS_WHEEL1_TOE) || toe_is_error(CHASSIS_WHEEL2_TOE)) {
       //   buzzer_on(100, 1500);
       //   osDelay(2000);
       //   buzzer_off();
       // }
      else if (toe_is_error(VT_TOE)) {
        buzzer_on(100, 1500);
        osDelay(300);
        buzzer_off();
        osDelay(300);
        buzzer_on(100, 1500);
        osDelay(300);
        buzzer_off();
        osDelay(2000);
      }
      else
      {
        buzzer_off();
      }

      if ((chassis_move.state == CHASSIS_LEG_1 || chassis_move.state == CHASSIS_LEG_2) &&
          chassis_move.step_up_phase != STEP_UP_DONE) {
        buzzer_on(400, 1500);
      }
      osDelay(10);
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
