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

extern ui_interface_rect_t *ui_normal_StaticGroup1_SuperCapRect;
extern ui_interface_line_t *ui_normal_StaticGroup1_SuperCapMidLine;
extern ui_interface_round_t *ui_normal_StaticGroup1_aiming;
extern ui_interface_round_t *ui_normal_StaticGroup1_SpinRound;
extern ui_interface_line_t *ui_normal_StaticGroup1_LaneLinerLeft;
extern ui_interface_line_t *ui_normal_StaticGroup1_LaneLineRight;
extern ui_interface_line_t *ui_normal_StaticGroup1_RemainLine;

void ui_init_normal_StaticGroup1();
void ui_update_normal_StaticGroup1();
void ui_remove_normal_StaticGroup1();

extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricOnText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricOffText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_FricErrorText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisNormalText;
extern ui_interface_string_t *ui_normal_StaticTextGroup1_ChassisStopText;

void ui_init_normal_StaticTextGroup1();
void ui_update_normal_StaticTextGroup1();
void ui_remove_normal_StaticTextGroup1();


#endif // UI_normal_H
