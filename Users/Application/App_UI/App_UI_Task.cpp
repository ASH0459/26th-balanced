#include "App_UI_Task.h"
#include "ui.h"
#include "ui_normal.h"
#include "main.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "ui_interface.h"
#include "UC_Referee.h"
#include "Dev_Can_Receive.h"
#include "HDL_SuperCap.hpp"
#include "Board_USART.h"
#include "wbr.h"

/*UI测试用数据*/
/*test code begin*/
float cap_test = 100.0f;

/*test code end*/

static void UI_Init(void);
static void UI_data_update(void);
static void UI_updata(void);
static void cap_energy_Update(void);
static void Leg_Position_Update(void);

/* UI任务函数 */
void UI_Task(void *argument)
{
  osDelay(2000);
  UI_Init();

  while (1)
  {
    UI_data_update();
    UI_updata();
    osDelay(200);
  }
}

/* UI初始化函数，设置UI初始状态 */
static void UI_Init(void)
{
  ui_init_normal_DynamicGroup1();
  osDelay(20);
  ui_init_normal_StaticGroup1();
  osDelay(20);
  ui_init_normal_StaticTextGroup1();
  osDelay(20);
  ui_init_normal_LegDynamicGroup();
  osDelay(20);
}

/* UI数据更新函数，更新UI显示的数据 */
static void UI_data_update(void)
{
  cap_energy_Update();
  Leg_Position_Update();
}

static void UI_updata(void)
{
  ui_update_normal_DynamicGroup1();
  osDelay(20);
  ui_update_normal_LegDynamicGroup();
  osDelay(20);
}

/* 更新超级电容能量显示 */
static void cap_energy_Update(void)
{
  SuperCap *super_cap = Get_SuperCap_Data_Point();
  float cap_energy = super_cap->Data.capEnergy;

  if (cap_energy < CAP_ENERGY_LOW_THRESHOLD)
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = PURPLE_COLOR; // 红色
  }
  else if (cap_energy < CAP_ENERGY_MEDIUM_THRESHOLD)
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = YELLOW_COLOR; // 黄色
  }
  else
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = GREEN_COLOR; // 绿色
  }

  ui_normal_DynamicGroup1_SuperCapPercentage->start_x = UI_SUPERCAP_BAR_START_X;
  ui_normal_DynamicGroup1_SuperCapPercentage->start_y = UI_SUPERCAP_BAR_Y;
  ui_normal_DynamicGroup1_SuperCapPercentage->end_x =
      UI_SUPERCAP_BAR_START_X +
      (uint16_t)((UI_SUPERCAP_BAR_END_X - UI_SUPERCAP_BAR_START_X) * cap_energy / 100.0f);
  ui_normal_DynamicGroup1_SuperCapPercentage->end_y = UI_SUPERCAP_BAR_Y;
}

/*更新腿部UI位置*/
static void Leg_Position_Update(void)
{
  //获取底盘数据
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();

  static fp32 test_theta = 0.5f; // 测试用的腿部角度，实际应用中应从底盘数据获取
  ui_normal_LegDynamicGroup_L1U_left->end_x = ui_normal_LegDynamicGroup_L1U_left->start_x + (L1U * 500* sin(test_theta));
  ui_normal_LegDynamicGroup_L1U_left->end_y = ui_normal_LegDynamicGroup_L1U_left->start_y - (L1U * 500 * cos(test_theta));
  ui_normal_LegDynamicGroup_L1D_left->start_x = ui_normal_LegDynamicGroup_L1U_left->end_x;
  ui_normal_LegDynamicGroup_L1D_left->start_y = ui_normal_LegDynamicGroup_L1U_left->end_y;
  // ui_normal_LegDynamicGroup_L1D_left->end_x = ui_normal_LegDynamicGroup_L1D_left->start_x - 
  // ui_normal_LegDynamicGroup_L1U_left->end_x = ui_normal_LegDynamicGroup_L1U_left->start_x + (L1U * 100* sin(chassis_move->chassis_left_control.theta_l));
  // ui_normal_LegDynamicGroup_L1U_left->end_y = ui_normal_LegDynamicGroup_L1U_left->start_y + (L1U * 100 * cos(chassis_move->chassis_left_control.theta_l));

  // 根据wbr更新腿部UI位置

}
