#include "App_UI_Task.h"
#include "ui.h"
#include "ui_normal.h"
#include "main.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "ui_interface.h"
#include "UC_Referee.h"
#include "App_Detect_Task.h"
#include "Dev_Can_Receive.h"
#include "HDL_SuperCap.hpp"
#include "Board_USART.h"
#include "wbr.h"
#include <math.h>

#define LOW_AMMO_THRESHOLD 10              // 17mm剩余弹量低于该值时显示LOW AMMO告警

// ui_normal.c由UI工具生成，头文件只暴露整组接口；这里单独调用某个字符串的新增/删除接口显示告警。
extern "C" void _ui_init_normal_DynamicTextGroup1_0(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_0(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_1(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_1(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_2(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_2(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_3(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_3(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_4(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_4(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_5(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_5(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_6(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_6(void);
extern "C" void _ui_init_normal_DynamicTextGroup1_7(void);
extern "C" void _ui_remove_normal_DynamicTextGroup1_7(void);

typedef void (*UI_Warning_Func)(void);

typedef struct
{
  uint8_t toe;           // detect模块里的设备编号
  UI_Warning_Func show;  // 发送该设备掉线文字
  UI_Warning_Func hide;  // 删除该设备掉线文字
} UI_Offline_Warning_t;

static void UI_Init(void);
static void UI_data_update(void);
static void UI_updata(void);
static void cap_energy_Update(void);
static void Gimbal_Chassis_Relative_Angle_Update(void);
static void Chassis_State_Rect_Update(void);
static void Fric_State_Rect_Update(void);
static void Fric_Target_Color_Update(void);
static void Low_Ammo_Warning_Update(void);
static void Device_Offline_Warnings_Update(void);
static void Leg_Position_Update(void);
static uint32_t UI_Limit_Coord(fp32 value);
static uint32_t UI_Normalize_Angle_Deg(fp32 angle);
static void UI_Move_Rect_To_Text(ui_interface_rect_t *rect, const ui_interface_string_t *text);
static void UI_Set_Warning_Text(uint8_t warning_active,
                                UI_Warning_Func show,
                                UI_Warning_Func hide,
                                uint8_t *warning_visible);
static void UI_Update_Leg(ui_interface_line_t *upper_line,
                          ui_interface_line_t *lower_line,
                          fp32 theta,
                          fp32 leg_length_m,
                          fp32 knee_side_sign);
static void UI_Update_Chassis_Pitch_Line(ui_interface_line_t *pitch_line, fp32 pitch);

/**
 * @brief UI任务主循环。
 * @param argument FreeRTOS任务参数，当前未使用。
 * @note 初始化UI图层后，周期性刷新数据并发送动态UI帧。
 */
void UI_Task(void *argument)
{
  osDelay(2000);
  UI_Init();
  osDelay(2000);

  while (1)
  {
    UI_data_update();
    UI_updata();
  }
}

/**
 * @brief 初始化裁判系统客户端UI图形。
 * @note 按动态图形、静态图形、静态文字、腿部动态图形的顺序发送，发送间隔用于降低串口拥塞。
 */
static void UI_Init(void)
{
  osDelay(500);
  ui_init_normal_DynamicGroup1();
  osDelay(500);
  ui_init_normal_StaticGroup1();
  osDelay(500);
  ui_init_normal_StaticTextGroup1();
  osDelay(500);
  ui_init_normal_LegDynamicGroup();
  osDelay(500);
}

/**
 * @brief 汇总更新所有会随机器人状态变化的UI数据。
 * @note 该函数只修改本地UI帧数据，真正发送由UI_updata()完成；动态文字告警会在各自函数中单独发送。
 */
static void UI_data_update(void)
{
  cap_energy_Update();
  Gimbal_Chassis_Relative_Angle_Update();
  Chassis_State_Rect_Update();
  Fric_State_Rect_Update();
  Fric_Target_Color_Update();
  Low_Ammo_Warning_Update();
  Device_Offline_Warnings_Update();
  Leg_Position_Update();
}

/**
 * @brief 发送周期性刷新的动态图形帧。
 * @note 当前发送普通动态图形组和腿部动态图形组，动态文字告警由告警函数按需新增/删除。
 */
static void UI_updata(void)
{
  ui_update_normal_DynamicGroup1();
  osDelay(20);
  ui_update_normal_LegDynamicGroup();
  osDelay(20);
}

/**
 * @brief 根据超电剩余能量更新能量条长度和颜色。
 * @note 低于低阈值显示紫色，中间区间显示黄色，高于中阈值显示绿色。
 */
static void cap_energy_Update(void)
{
  SuperCap *super_cap = Get_SuperCap_Data_Point();
  float cap_energy = super_cap->Data.capEnergy;
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

/**
 * @brief 将浮点坐标限制到裁判系统UI坐标范围。
 * @param value 待限制的坐标值。
 * @return 限制并四舍五入后的0到2047范围坐标。
 */
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

/**
 * @brief 将角度归一化到[0, 360)范围。
 * @param angle 输入角度，单位为度。
 * @return 归一化并四舍五入后的角度值。
 */
static uint32_t UI_Normalize_Angle_Deg(fp32 angle)
{
  while (angle < 0.0f)
  {
    angle += 360.0f;
  }

  while (angle >= 360.0f)
  {
    angle -= 360.0f;
  }

  uint32_t angle_int = (uint32_t)(angle + 0.5f);
  if (angle_int >= 360U)
  {
    angle_int -= 360U;
  }

  return angle_int;
}

/**
 * @brief 根据云台相对底盘角度更新中心旋转弧。
 * @note 相对角来自双板通信中的gimbal_data，用于表示云台朝向。
 */
static void Gimbal_Chassis_Relative_Angle_Update(void)
{
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();

  if (chassis_move == nullptr || chassis_move->chassis_gimbal_data == nullptr)
  {
    return;
  }

  const fp32 relative_angle = chassis_move->chassis_gimbal_data->chassis_relative_angle + PI;
  const fp32 start_angle = 135.0f - relative_angle * 57.32f;
  const fp32 end_angle = 225.0f - relative_angle * 57.32f;

  ui_normal_DynamicGroup1_SpinArc->start_angle = UI_Normalize_Angle_Deg(start_angle);
  ui_normal_DynamicGroup1_SpinArc->end_angle = UI_Normalize_Angle_Deg(end_angle);
}

/**
 * @brief 将矩形框移动到指定文字周围。
 * @param rect 要更新位置的矩形UI对象。
 * @param text 作为包围目标的文字UI对象。
 * @note 通过固定padding计算包围框，用于底盘状态框和摩擦轮状态框。
 */
static void UI_Move_Rect_To_Text(ui_interface_rect_t *rect, const ui_interface_string_t *text)
{
  static const uint32_t padding_left = 14U;
  static const uint32_t padding_right = 14U;
  static const uint32_t padding_top = 28U;
  static const uint32_t padding_bottom = 13U;

  if (rect == nullptr || text == nullptr)
  {
    return;
  }

  rect->start_x = text->start_x - padding_left;
  rect->start_y = text->start_y - padding_top;
  rect->end_x = text->start_x + text->font_size * text->str_length + padding_right;
  rect->end_y = text->start_y + padding_bottom;
}

/**
 * @brief 根据底盘状态移动底盘状态框。
 * @note NORMAL、STEP1、STEP2、JUMP分别映射到对应文字，其它状态归为STOP显示。
 */
static void Chassis_State_Rect_Update(void)
{
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();
  const ui_interface_string_t *target_text = ui_normal_StaticTextGroup1_ChassisStopText;

  if (chassis_move == nullptr)
  {
    return;
  }

  switch (chassis_move->state)
  {
  case CHASSIS_NORMAL:
    target_text = ui_normal_StaticTextGroup1_ChassisNormalText;
    break;

  case CHASSIS_LEG_1:
    target_text = ui_normal_StaticTextGroup1_ChassisStep1;
    break;

  case CHASSIS_LEG_2:
    target_text = ui_normal_StaticTextGroup1_ChassisStep2;
    break;

  case CHASSIS_JUMP:
    target_text = ui_normal_StaticTextGroup1_ChassisNormalText;
    break;

  case CHASSIS_STOP:
  case CHASSIS_FLIP:
  case CHASSIS_INIT:
  default:
    target_text = ui_normal_StaticTextGroup1_ChassisStopText;
    break;
  }

  UI_Move_Rect_To_Text(ui_normal_DynamicGroup1_ChassisStateRect, target_text);
}

/**
 * @brief 根据摩擦轮状态移动右侧摩擦轮状态框。
 * @note fric_state为4/5时表示自瞄附加状态，不改变右侧摩擦轮模式框。
 */
static void Fric_State_Rect_Update(void)
{
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();
  const ui_interface_string_t *target_text = ui_normal_StaticTextGroup1_FricErrorText;

  if (chassis_move == nullptr || chassis_move->chassis_gimbal_data == nullptr)
  {
    return;
  }

  switch (chassis_move->chassis_gimbal_data->fric_state)
  {
  case FRIC_OFF:
    target_text = ui_normal_StaticTextGroup1_FricOffText;
    break;

  case FRIC_ON:
    target_text = ui_normal_StaticTextGroup1_FricOnText;
    break;

  // 4/5表示视觉锁定目标时的附加状态，不改变右侧摩擦轮状态框的位置。
  case FRIC_AUTO_AIM_TARGET_NO_MANUAL_FIRE:
  case FRIC_AUTO_AIM_TARGET_MANUAL_FIRE:
    return;

  case FRIC_ERROR:
  default:
    target_text = ui_normal_StaticTextGroup1_FricErrorText;
    break;
  }

  UI_Move_Rect_To_Text(ui_normal_DynamicGroup1_FricStateRect, target_text);
}

/**
 * @brief 根据自瞄目标状态更新中间自瞄框颜色。
 * @note 4表示识别到目标但禁止手动开火，显示紫红色；5表示允许手动开火，显示蓝色；普通状态显示白色。
 */
static void Fric_Target_Color_Update(void)
{
  Chassis_Move *chassis_move = Get_Chassis_Move_Point();

  if (chassis_move == nullptr || chassis_move->chassis_gimbal_data == nullptr)
  {
    return;
  }

  switch (chassis_move->chassis_gimbal_data->fric_state)
  {
  // 自瞄识别到目标但禁止手动开火：中间自瞄框显示紫红色。
  case FRIC_AUTO_AIM_TARGET_NO_MANUAL_FIRE:
    ui_normal_DynamicGroup1_FricOrNot->color = PINK_COLOR;
    break;

  // 自瞄识别到目标且允许手动开火：中间自瞄框显示蓝色。
  case FRIC_AUTO_AIM_TARGET_MANUAL_FIRE:
    ui_normal_DynamicGroup1_FricOrNot->color = CYAN_COLOR;
    break;

  // 普通摩擦轮状态下不强调自瞄框。
  default:
    ui_normal_DynamicGroup1_FricOrNot->color = WHITE_COLOR;
    break;
  }
}

/**
 * @brief 按告警状态显示或删除动态文字。
 * @param warning_active 告警是否处于触发状态。
 * @param show 发送新增/显示文字的函数。
 * @param hide 发送删除文字的函数。
 * @param warning_visible 当前文字是否处于显示状态。
 * @note 告警保持期间不重复发送，只有状态变化时才占用裁判系统带宽。
 */
static void UI_Set_Warning_Text(uint8_t warning_active,
                                UI_Warning_Func show,
                                UI_Warning_Func hide,
                                uint8_t *warning_visible)
{
  if (show == nullptr || hide == nullptr || warning_visible == nullptr)
  {
    return;
  }

  if (warning_active == 0U)
  {
    if (*warning_visible != 0U)
    {
      hide();
      *warning_visible = 0U;
    }

    return;
  }

  if (*warning_visible == 0U)
  {
    show();
    *warning_visible = 1U;
  }
}

/**
 * @brief 根据17mm剩余弹量更新LOW AMMO告警。
 * @note 裁判系统离线时不显示低弹量告警；弹量低于LOW_AMMO_THRESHOLD时显示。
 */
static void Low_Ammo_Warning_Update(void)
{
  uint16_t bullet_remaining = 0U;
  static uint8_t warning_visible = 0U;

  if (toe_is_error(Referee_TOE))
  {
    UI_Set_Warning_Text(0U,
                        _ui_init_normal_DynamicTextGroup1_0,
                        _ui_remove_normal_DynamicTextGroup1_0,
                        &warning_visible);
    return;
  }

  get_ammo(&bullet_remaining);
  // 裁判系统字段是uint16_t；如果上游把负数打包成0xFFFF，这里按int16_t还原后仍会触发低弹量告警。
  const int16_t bullet_remaining_signed = static_cast<int16_t>(bullet_remaining);
  UI_Set_Warning_Text((bullet_remaining_signed < LOW_AMMO_THRESHOLD) ? 1U : 0U,
                      _ui_init_normal_DynamicTextGroup1_0,
                      _ui_remove_normal_DynamicTextGroup1_0,
                      &warning_visible);
}

/**
 * @brief 根据detect模块掉线状态更新设备掉线告警。
 * @note 多个设备同时掉线时只显示优先级最高的一条，优先级由warnings表顺序决定。
 */
static void Device_Offline_Warnings_Update(void)
{
  // 这些动态文字在UI里共用相近位置；同一时刻只显示优先级最高的一条，避免重叠。
  static const UI_Offline_Warning_t warnings[] = {
      {VT_TOE, _ui_init_normal_DynamicTextGroup1_1, _ui_remove_normal_DynamicTextGroup1_1},
      {CHASSIS_JOINT1_TOE, _ui_init_normal_DynamicTextGroup1_2, _ui_remove_normal_DynamicTextGroup1_2},
      {CHASSIS_JOINT2_TOE, _ui_init_normal_DynamicTextGroup1_3, _ui_remove_normal_DynamicTextGroup1_3},
      {CHASSIS_JOINT3_TOE, _ui_init_normal_DynamicTextGroup1_4, _ui_remove_normal_DynamicTextGroup1_4},
      {CHASSIS_JOINT4_TOE, _ui_init_normal_DynamicTextGroup1_5, _ui_remove_normal_DynamicTextGroup1_5},
      {CHASSIS_WHEEL1_TOE, _ui_init_normal_DynamicTextGroup1_6, _ui_remove_normal_DynamicTextGroup1_6},
      {CHASSIS_WHEEL2_TOE, _ui_init_normal_DynamicTextGroup1_7, _ui_remove_normal_DynamicTextGroup1_7},
  };
  static uint8_t warning_visible = 0U;
  static uint8_t active_index = 0xFFU;
  uint8_t selected_index = 0xFFU;
  const uint8_t warning_count = (uint8_t)(sizeof(warnings) / sizeof(warnings[0]));

  // 按表顺序选择第一个掉线设备，表顺序就是显示优先级。
  for (uint8_t i = 0U; i < warning_count; ++i)
  {
    if (toe_is_error(warnings[i].toe))
    {
      selected_index = i;
      break;
    }
  }

  if (active_index != selected_index)
  {
    // 切换告警对象前删除上一条可见文字，并重置显示状态。
    if (active_index < warning_count && warning_visible != 0U)
    {
      warnings[active_index].hide();
    }

    active_index = selected_index;
    warning_visible = 0U;
  }

  if (selected_index < warning_count)
  {
    UI_Set_Warning_Text(1U,
                        warnings[selected_index].show,
                        warnings[selected_index].hide,
                        &warning_visible);
  }
}

/**
 * @brief 根据腿长和腿角更新单侧腿部两段连杆的UI线段。
 * @param upper_line 上连杆线段对象。
 * @param lower_line 下连杆线段对象。
 * @param theta 腿部相对竖直方向角度，单位为弧度。
 * @param leg_length_m 腿长，单位为米。
 * @param knee_side_sign 膝关节弯折方向符号。
 * @note 使用两圆交点几何关系计算膝关节位置，并限制到UI坐标范围。
 */
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

/**
 * @brief 根据底盘pitch角更新机体姿态线。
 * @param pitch_line 表示底盘姿态的线段对象。
 * @param pitch 底盘pitch角，单位为弧度。
 * @note 第一次调用时记录线段中心和半长，后续只旋转线段端点。
 */
static void UI_Update_Chassis_Pitch_Line(ui_interface_line_t *pitch_line, fp32 pitch)
{
  pitch = -pitch;

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

/**
 * @brief 更新左右腿连杆和底盘pitch线的UI显示。
 * @note 数据来自底盘控制对象中的WBR解算结果和底盘pitch估计值。
 */
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
