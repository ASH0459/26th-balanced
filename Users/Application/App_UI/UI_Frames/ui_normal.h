//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_normal_H
#define UI_normal_H

#include "ui_interface.h"

extern ui_interface_rect_t *ui_normal_DynamicGroup1_FricOrNot;
extern ui_interface_rect_t *ui_normal_DynamicGroup1_FricStateRect;
extern ui_interface_arc_t *ui_normal_DynamicGroup1_SpinArc;
extern ui_interface_line_t *ui_normal_DynamicGroup1_SuperCapPercentage;
extern ui_interface_rect_t *ui_normal_DynamicGroup1_ChassisStateRect;

void ui_init_normal_DynamicGroup1();
void ui_update_normal_DynamicGroup1();
void ui_remove_normal_DynamicGroup1();

extern ui_interface_string_t *ui_normal_DynamicTextGroup1_LowAmmoText;

void ui_init_normal_DynamicTextGroup1();
void ui_update_normal_DynamicTextGroup1();
void ui_remove_normal_DynamicTextGroup1();

extern ui_interface_line_t *ui_normal_LegDynamicGroup_L1U_left;
extern ui_interface_line_t *ui_normal_LegDynamicGroup_L1D_left;
extern ui_interface_line_t *ui_normal_LegDynamicGroup_L2U_right;
extern ui_interface_line_t *ui_normal_LegDynamicGroup_L2D_right;
extern ui_interface_number_t *ui_normal_LegDynamicGroup_step_up_number;

void ui_init_normal_LegDynamicGroup();
void ui_update_normal_LegDynamicGroup();
void ui_remove_normal_LegDynamicGroup();

extern ui_interface_rect_t *ui_normal_StaticGroup1_SuperCapRect;
extern ui_interface_line_t *ui_normal_StaticGroup1_SuperCapMidLine;
extern ui_interface_line_t *ui_normal_StaticGroup1_5mLine;
extern ui_interface_line_t *ui_normal_StaticGroup1_aimLine;
extern ui_interface_line_t *ui_normal_StaticGroup1_LaneLinerLeft;
extern ui_interface_line_t *ui_normal_StaticGroup1_LaneLineRight;
extern ui_interface_line_t *ui_normal_StaticGroup1_3mLine;

void ui_init_normal_StaticGroup1();
void ui_update_normal_StaticGroup1();
void ui_remove_normal_StaticGroup1();

extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricOnText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricOffText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricErrorText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisNormalText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisStopText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisStep1;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisStep2;

void ui_init_normal_StaticTextGroup1();
void ui_update_normal_StaticTextGroup1();
void ui_remove_normal_StaticTextGroup1();


#endif // UI_normal_H
