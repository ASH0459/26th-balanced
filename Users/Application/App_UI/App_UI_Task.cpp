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
#include <math.h>

/* UI test data */
float cap_test = 100.0f;

static void UI_Init(void);
static void UI_data_update(void);
static void UI_updata(void);
static void cap_energy_Update(void);
static void Leg_Position_Update(void);
static uint32_t UI_Limit_Coord(fp32 value);
static void UI_Update_Leg(ui_interface_line_t *upper_line,
                          ui_interface_line_t *lower_line,
                          fp32 theta,
                          fp32 leg_length_m,
                          fp32 knee_side_sign);
static void UI_Update_Chassis_Pitch_Line(ui_interface_line_t *pitch_line, fp32 pitch);

void UI_Task(void *argument)
{
  osDelay(2000);
  UI_Init();

  while (1)
  {
    UI_data_update();
    UI_updata();
  }
}

static void UI_Init(void)
{
  ui_init_normal_DynamicGroup1();
  osDelay(200);
  ui_init_normal_StaticGroup1();
  osDelay(200);
  ui_init_normal_StaticTextGroup1();
  osDelay(200);
  ui_init_normal_LegDynamicGroup();
  osDelay(200);
}

static void UI_data_update(void)
{
  cap_energy_Update();
  Leg_Position_Update();
}

static void UI_updata(void)
{
  ui_update_normal_DynamicGroup1();
  osDelay(50);
  ui_update_normal_LegDynamicGroup();
  osDelay(50);
}

static void cap_energy_Update(void)
{
  SuperCap *super_cap = Get_SuperCap_Data_Point();
  float cap_energy = super_cap->Data.capEnergy;
  cap_energy = cap_test; // TODO: delete this line after test
  const uint32_t bar_start_x = UI_SuperCap_Bar_Start_X();
  const uint32_t bar_start_y = UI_SuperCap_Bar_Start_Y();
  const uint32_t bar_end_x = UI_SuperCap_Bar_End_X();

  if (cap_energy < CAP_ENERGY_LOW_THRESHOLD)
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = PURPLE_COLOR;
  }
  else if (cap_energy < CAP_ENERGY_MEDIUM_THRESHOLD)
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = YELLOW_COLOR;
  }
  else
  {
    ui_normal_DynamicGroup1_SuperCapPercentage->color = GREEN_COLOR;
  }

  ui_normal_DynamicGroup1_SuperCapPercentage->start_x = bar_start_x;
  ui_normal_DynamicGroup1_SuperCapPercentage->start_y = bar_start_y;
  ui_normal_DynamicGroup1_SuperCapPercentage->end_x =
      bar_start_x +
      (uint16_t)((bar_end_x - bar_start_x) * cap_energy / 100.0f);
  ui_normal_DynamicGroup1_SuperCapPercentage->end_y = bar_start_y;
}

static uint32_t UI_Limit_Coord(fp32 value)
{
  if (value < 0.0f)
  {
    return 0U;
  }

  if (value > 2047.0f)
  {
    return 2047U;
  }

  return (uint32_t)(value + 0.5f);
}

static void UI_Update_Leg(ui_interface_line_t *upper_line,
                          ui_interface_line_t *lower_line,
                          fp32 theta,
                          fp32 leg_length_m,
                          fp32 knee_side_sign)
{
  static const fp32 ui_scale = 500.0f;
  const fp32 hip_x = (fp32)upper_line->start_x;
  const fp32 hip_y = (fp32)upper_line->start_y;
  const fp32 upper_len = L1U * ui_scale;
  const fp32 lower_len = L1D * ui_scale;
  const fp32 min_leg_len = fabsf(lower_len - upper_len) + 0.001f;
  const fp32 max_leg_len = lower_len + upper_len - 0.001f;
  fp32 leg_len = leg_length_m * ui_scale;

  if (leg_len < min_leg_len)
  {
    leg_len = min_leg_len;
  }
  else if (leg_len > max_leg_len)
  {
    leg_len = max_leg_len;
  }

  const fp32 foot_x = hip_x - leg_len * sinf(theta);
  const fp32 foot_y = hip_y - leg_len * cosf(theta);
  const fp32 axis_dx = foot_x - hip_x;
  const fp32 axis_dy = foot_y - hip_y;
  const fp32 axis_ux = axis_dx / leg_len;
  const fp32 axis_uy = axis_dy / leg_len;
  const fp32 axis_proj = (upper_len * upper_len - lower_len * lower_len + leg_len * leg_len) / (2.0f * leg_len);
  fp32 knee_offset_sq = upper_len * upper_len - axis_proj * axis_proj;

  if (knee_offset_sq < 0.0f)
  {
    knee_offset_sq = 0.0f;
  }

  const fp32 knee_offset = sqrtf(knee_offset_sq);
  const fp32 axis_mid_x = hip_x + axis_proj * axis_ux;
  const fp32 axis_mid_y = hip_y + axis_proj * axis_uy;
  const fp32 knee_x = axis_mid_x - knee_side_sign * axis_uy * knee_offset;
  const fp32 knee_y = axis_mid_y + knee_side_sign * axis_ux * knee_offset;

  upper_line->end_x = UI_Limit_Coord(knee_x);
  upper_line->end_y = UI_Limit_Coord(knee_y);

  lower_line->start_x = UI_Limit_Coord(knee_x);
  lower_line->start_y = UI_Limit_Coord(knee_y);
  lower_line->end_x = UI_Limit_Coord(foot_x);
  lower_line->end_y = UI_Limit_Coord(foot_y);
}

static void UI_Update_Chassis_Pitch_Line(ui_interface_line_t *pitch_line, fp32 pitch)
{
  static fp32 center_x = 0.0f;
  static fp32 center_y = 0.0f;
  static fp32 half_length = 0.0f;
  static uint8_t init_done = 0U;

  if (init_done == 0U)
  {
    const fp32 dx = (fp32)pitch_line->start_x - (fp32)pitch_line->end_x;
    const fp32 dy = (fp32)pitch_line->start_y - (fp32)pitch_line->end_y;

    center_x = ((fp32)pitch_line->start_x + (fp32)pitch_line->end_x) * 0.5f;
    center_y = ((fp32)pitch_line->start_y + (fp32)pitch_line->end_y) * 0.5f;
    half_length = sqrtf(dx * dx + dy * dy) * 0.5f;
    init_done = 1U;
  }

  pitch_line->start_x = UI_Limit_Coord(center_x + half_length * cosf(pitch));
  pitch_line->start_y = UI_Limit_Coord(center_y - half_length * sinf(pitch));
  pitch_line->end_x = UI_Limit_Coord(center_x - half_length * cosf(pitch));
  pitch_line->end_y = UI_Limit_Coord(center_y + half_length * sinf(pitch));
}

static void Leg_Position_Update(void)
{
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();

  UI_Update_Leg(ui_normal_LegDynamicGroup_L1U_left,
                ui_normal_LegDynamicGroup_L1D_left,
                chassis_move->chassis_left_control.wbr_control.theta_l,
                chassis_move->chassis_left_control.wbr_control.L,
                1.0f);

  UI_Update_Leg(ui_normal_LegDynamicGroup_L2U_right,
                ui_normal_LegDynamicGroup_L2D_right,
                chassis_move->chassis_right_control.wbr_control.theta_l,
                chassis_move->chassis_right_control.wbr_control.L,
                1.0f);

  UI_Update_Chassis_Pitch_Line(ui_normal_LegDynamicGroup_ChassisPitch,
                               chassis_move->chassis_pitch);
}
