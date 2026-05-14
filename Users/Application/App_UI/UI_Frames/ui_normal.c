//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_normal_DynamicGroup1_0;

ui_interface_rect_t *ui_normal_DynamicGroup1_FricOrNot = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[0]);
ui_interface_rect_t *ui_normal_DynamicGroup1_FricStateRect = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[1]);
ui_interface_arc_t *ui_normal_DynamicGroup1_SpinArc = (ui_interface_arc_t*)&(ui_normal_DynamicGroup1_0.data[2]);
ui_interface_line_t *ui_normal_DynamicGroup1_SuperCapPercentage = (ui_interface_line_t*)&(ui_normal_DynamicGroup1_0.data[3]);
ui_interface_rect_t *ui_normal_DynamicGroup1_ChassisStateRect = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[4]);

void _ui_init_normal_DynamicGroup1_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_DynamicGroup1_0.data[i].figure_name[0] = 1;
        ui_normal_DynamicGroup1_0.data[i].figure_name[1] = 0;
        ui_normal_DynamicGroup1_0.data[i].figure_name[2] = i + 0;
        ui_normal_DynamicGroup1_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_normal_DynamicGroup1_0.data[i].operate_type = 0;
    }

    ui_normal_DynamicGroup1_FricOrNot->figure_type = 1;
    ui_normal_DynamicGroup1_FricOrNot->operate_type = 1;
    ui_normal_DynamicGroup1_FricOrNot->layer = 0;
    ui_normal_DynamicGroup1_FricOrNot->color = 8;
    ui_normal_DynamicGroup1_FricOrNot->start_x = 617;
    ui_normal_DynamicGroup1_FricOrNot->start_y = 248;
    ui_normal_DynamicGroup1_FricOrNot->width = 1;
    ui_normal_DynamicGroup1_FricOrNot->end_x = 1307;
    ui_normal_DynamicGroup1_FricOrNot->end_y = 833;

    ui_normal_DynamicGroup1_FricStateRect->figure_type = 1;
    ui_normal_DynamicGroup1_FricStateRect->operate_type = 1;
    ui_normal_DynamicGroup1_FricStateRect->layer = 0;
    ui_normal_DynamicGroup1_FricStateRect->color = 5;
    ui_normal_DynamicGroup1_FricStateRect->start_x = 1718;
    ui_normal_DynamicGroup1_FricStateRect->start_y = 825;
    ui_normal_DynamicGroup1_FricStateRect->width = 3;
    ui_normal_DynamicGroup1_FricStateRect->end_x = 1844;
    ui_normal_DynamicGroup1_FricStateRect->end_y = 871;

    ui_normal_DynamicGroup1_SpinArc->figure_type = 4;
    ui_normal_DynamicGroup1_SpinArc->operate_type = 1;
    ui_normal_DynamicGroup1_SpinArc->layer = 0;
    ui_normal_DynamicGroup1_SpinArc->color = 2;
    ui_normal_DynamicGroup1_SpinArc->start_x = 960;
    ui_normal_DynamicGroup1_SpinArc->start_y = 538;
    ui_normal_DynamicGroup1_SpinArc->width = 10;
    ui_normal_DynamicGroup1_SpinArc->start_angle = 345;
    ui_normal_DynamicGroup1_SpinArc->end_angle = 15;
    ui_normal_DynamicGroup1_SpinArc->rx = 90;
    ui_normal_DynamicGroup1_SpinArc->ry = 90;

    ui_normal_DynamicGroup1_SuperCapPercentage->figure_type = 0;
    ui_normal_DynamicGroup1_SuperCapPercentage->operate_type = 1;
    ui_normal_DynamicGroup1_SuperCapPercentage->layer = 0;
    ui_normal_DynamicGroup1_SuperCapPercentage->color = 2;
    ui_normal_DynamicGroup1_SuperCapPercentage->start_x = 734;
    ui_normal_DynamicGroup1_SuperCapPercentage->start_y = 79;
    ui_normal_DynamicGroup1_SuperCapPercentage->width = 10;
    ui_normal_DynamicGroup1_SuperCapPercentage->end_x = 1134;
    ui_normal_DynamicGroup1_SuperCapPercentage->end_y = 79;

    ui_normal_DynamicGroup1_ChassisStateRect->figure_type = 1;
    ui_normal_DynamicGroup1_ChassisStateRect->operate_type = 1;
    ui_normal_DynamicGroup1_ChassisStateRect->layer = 0;
    ui_normal_DynamicGroup1_ChassisStateRect->color = 5;
    ui_normal_DynamicGroup1_ChassisStateRect->start_x = 1556;
    ui_normal_DynamicGroup1_ChassisStateRect->start_y = 825;
    ui_normal_DynamicGroup1_ChassisStateRect->width = 3;
    ui_normal_DynamicGroup1_ChassisStateRect->end_x = 1670;
    ui_normal_DynamicGroup1_ChassisStateRect->end_y = 866;


    ui_proc_5_frame(&ui_normal_DynamicGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicGroup1_0, sizeof(ui_normal_DynamicGroup1_0));
}

void _ui_update_normal_DynamicGroup1_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_DynamicGroup1_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_normal_DynamicGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicGroup1_0, sizeof(ui_normal_DynamicGroup1_0));
}

void _ui_remove_normal_DynamicGroup1_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_DynamicGroup1_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_normal_DynamicGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicGroup1_0, sizeof(ui_normal_DynamicGroup1_0));
}


void ui_init_normal_DynamicGroup1() {
    _ui_init_normal_DynamicGroup1_0();
}

void ui_update_normal_DynamicGroup1() {
    _ui_update_normal_DynamicGroup1_0();
}

void ui_remove_normal_DynamicGroup1() {
    _ui_remove_normal_DynamicGroup1_0();
}


ui_string_frame_t ui_normal_DynamicTextGroup1_0;
ui_interface_string_t* ui_normal_DynamicTextGroup1_LowAmmoText = &(ui_normal_DynamicTextGroup1_0.option);

void _ui_init_normal_DynamicTextGroup1_0() {
    ui_normal_DynamicTextGroup1_0.option.figure_name[0] = 1;
    ui_normal_DynamicTextGroup1_0.option.figure_name[1] = 1;
    ui_normal_DynamicTextGroup1_0.option.figure_name[2] = 0;
    ui_normal_DynamicTextGroup1_0.option.operate_type = 1;

    ui_normal_DynamicTextGroup1_LowAmmoText->figure_type = 7;
    ui_normal_DynamicTextGroup1_LowAmmoText->operate_type = 1;
    ui_normal_DynamicTextGroup1_LowAmmoText->layer = 0;
    ui_normal_DynamicTextGroup1_LowAmmoText->color = 5;
    ui_normal_DynamicTextGroup1_LowAmmoText->start_x = 682;
    ui_normal_DynamicTextGroup1_LowAmmoText->start_y = 845;
    ui_normal_DynamicTextGroup1_LowAmmoText->width = 5;
    ui_normal_DynamicTextGroup1_LowAmmoText->font_size = 50;
    ui_normal_DynamicTextGroup1_LowAmmoText->str_length = 11;
    strcpy(ui_normal_DynamicTextGroup1_LowAmmoText->string, "LOW AMMO!!!");


    ui_proc_string_frame(&ui_normal_DynamicTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicTextGroup1_0, sizeof(ui_normal_DynamicTextGroup1_0));
}

void _ui_update_normal_DynamicTextGroup1_0() {
    ui_normal_DynamicTextGroup1_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_DynamicTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicTextGroup1_0, sizeof(ui_normal_DynamicTextGroup1_0));
}

void _ui_remove_normal_DynamicTextGroup1_0() {
    ui_normal_DynamicTextGroup1_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_DynamicTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_DynamicTextGroup1_0, sizeof(ui_normal_DynamicTextGroup1_0));
}

void ui_init_normal_DynamicTextGroup1() {
    _ui_init_normal_DynamicTextGroup1_0();
}

void ui_update_normal_DynamicTextGroup1() {
    _ui_update_normal_DynamicTextGroup1_0();
}

void ui_remove_normal_DynamicTextGroup1() {
    _ui_remove_normal_DynamicTextGroup1_0();
}

ui_7_frame_t ui_normal_LegDynamicGroup_0;

ui_interface_line_t *ui_normal_LegDynamicGroup_L1U_left = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[0]);
ui_interface_line_t *ui_normal_LegDynamicGroup_L1D_left = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[1]);
ui_interface_line_t *ui_normal_LegDynamicGroup_L2U_right = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[2]);
ui_interface_line_t *ui_normal_LegDynamicGroup_L2D_right = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[3]);
ui_interface_number_t *ui_normal_LegDynamicGroup_step_up_number = (ui_interface_number_t*)&(ui_normal_LegDynamicGroup_0.data[4]);
ui_interface_number_t *ui_normal_LegDynamicGroup_yaw_offset = (ui_interface_number_t*)&(ui_normal_LegDynamicGroup_0.data[5]);
ui_interface_number_t *ui_normal_LegDynamicGroup_pitch_offset = (ui_interface_number_t*)&(ui_normal_LegDynamicGroup_0.data[6]);

void _ui_init_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_LegDynamicGroup_0.data[i].figure_name[0] = 1;
        ui_normal_LegDynamicGroup_0.data[i].figure_name[1] = 2;
        ui_normal_LegDynamicGroup_0.data[i].figure_name[2] = i + 0;
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 0;
    }

    ui_normal_LegDynamicGroup_L1U_left->figure_type = 0;
    ui_normal_LegDynamicGroup_L1U_left->operate_type = 1;
    ui_normal_LegDynamicGroup_L1U_left->layer = 0;
    ui_normal_LegDynamicGroup_L1U_left->color = 0;
    ui_normal_LegDynamicGroup_L1U_left->start_x = 145;
    ui_normal_LegDynamicGroup_L1U_left->start_y = 833;
    ui_normal_LegDynamicGroup_L1U_left->width = 3;
    ui_normal_LegDynamicGroup_L1U_left->end_x = 222;
    ui_normal_LegDynamicGroup_L1U_left->end_y = 761;

    ui_normal_LegDynamicGroup_L1D_left->figure_type = 0;
    ui_normal_LegDynamicGroup_L1D_left->operate_type = 1;
    ui_normal_LegDynamicGroup_L1D_left->layer = 0;
    ui_normal_LegDynamicGroup_L1D_left->color = 0;
    ui_normal_LegDynamicGroup_L1D_left->start_x = 223;
    ui_normal_LegDynamicGroup_L1D_left->start_y = 766;
    ui_normal_LegDynamicGroup_L1D_left->width = 3;
    ui_normal_LegDynamicGroup_L1D_left->end_x = 132;
    ui_normal_LegDynamicGroup_L1D_left->end_y = 667;

    ui_normal_LegDynamicGroup_L2U_right->figure_type = 0;
    ui_normal_LegDynamicGroup_L2U_right->operate_type = 1;
    ui_normal_LegDynamicGroup_L2U_right->layer = 0;
    ui_normal_LegDynamicGroup_L2U_right->color = 0;
    ui_normal_LegDynamicGroup_L2U_right->start_x = 286;
    ui_normal_LegDynamicGroup_L2U_right->start_y = 829;
    ui_normal_LegDynamicGroup_L2U_right->width = 3;
    ui_normal_LegDynamicGroup_L2U_right->end_x = 363;
    ui_normal_LegDynamicGroup_L2U_right->end_y = 757;

    ui_normal_LegDynamicGroup_L2D_right->figure_type = 0;
    ui_normal_LegDynamicGroup_L2D_right->operate_type = 1;
    ui_normal_LegDynamicGroup_L2D_right->layer = 0;
    ui_normal_LegDynamicGroup_L2D_right->color = 0;
    ui_normal_LegDynamicGroup_L2D_right->start_x = 364;
    ui_normal_LegDynamicGroup_L2D_right->start_y = 762;
    ui_normal_LegDynamicGroup_L2D_right->width = 3;
    ui_normal_LegDynamicGroup_L2D_right->end_x = 273;
    ui_normal_LegDynamicGroup_L2D_right->end_y = 663;

    ui_normal_LegDynamicGroup_step_up_number->figure_type = 6;
    ui_normal_LegDynamicGroup_step_up_number->operate_type = 1;
    ui_normal_LegDynamicGroup_step_up_number->layer = 0;
    ui_normal_LegDynamicGroup_step_up_number->color = 6;
    ui_normal_LegDynamicGroup_step_up_number->start_x = 1273;
    ui_normal_LegDynamicGroup_step_up_number->start_y = 815;
    ui_normal_LegDynamicGroup_step_up_number->width = 2;
    ui_normal_LegDynamicGroup_step_up_number->font_size = 20;
    ui_normal_LegDynamicGroup_step_up_number->number = 1;

    ui_normal_LegDynamicGroup_yaw_offset->figure_type = 5;
    ui_normal_LegDynamicGroup_yaw_offset->operate_type = 1;
    ui_normal_LegDynamicGroup_yaw_offset->layer = 0;
    ui_normal_LegDynamicGroup_yaw_offset->color = 2;
    ui_normal_LegDynamicGroup_yaw_offset->start_x = 1596;
    ui_normal_LegDynamicGroup_yaw_offset->start_y = 683;
    ui_normal_LegDynamicGroup_yaw_offset->width = 2;
    ui_normal_LegDynamicGroup_yaw_offset->font_size = 20;
    ui_normal_LegDynamicGroup_yaw_offset->number = 100;

    ui_normal_LegDynamicGroup_pitch_offset->figure_type = 5;
    ui_normal_LegDynamicGroup_pitch_offset->operate_type = 1;
    ui_normal_LegDynamicGroup_pitch_offset->layer = 0;
    ui_normal_LegDynamicGroup_pitch_offset->color = 2;
    ui_normal_LegDynamicGroup_pitch_offset->start_x = 1781;
    ui_normal_LegDynamicGroup_pitch_offset->start_y = 683;
    ui_normal_LegDynamicGroup_pitch_offset->width = 2;
    ui_normal_LegDynamicGroup_pitch_offset->font_size = 20;
    ui_normal_LegDynamicGroup_pitch_offset->number = 100;


    ui_proc_7_frame(&ui_normal_LegDynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_LegDynamicGroup_0, sizeof(ui_normal_LegDynamicGroup_0));
}

void _ui_update_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_normal_LegDynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_LegDynamicGroup_0, sizeof(ui_normal_LegDynamicGroup_0));
}

void _ui_remove_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_normal_LegDynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_LegDynamicGroup_0, sizeof(ui_normal_LegDynamicGroup_0));
}


void ui_init_normal_LegDynamicGroup() {
    _ui_init_normal_LegDynamicGroup_0();
}

void ui_update_normal_LegDynamicGroup() {
    _ui_update_normal_LegDynamicGroup_0();
}

void ui_remove_normal_LegDynamicGroup() {
    _ui_remove_normal_LegDynamicGroup_0();
}

ui_7_frame_t ui_normal_StaticGroup1_0;

ui_interface_rect_t *ui_normal_StaticGroup1_SuperCapRect = (ui_interface_rect_t*)&(ui_normal_StaticGroup1_0.data[0]);
ui_interface_line_t *ui_normal_StaticGroup1_SuperCapMidLine = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[1]);
ui_interface_line_t *ui_normal_StaticGroup1_5mLine = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[2]);
ui_interface_line_t *ui_normal_StaticGroup1_aimLine = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[3]);
ui_interface_line_t *ui_normal_StaticGroup1_LaneLinerLeft = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[4]);
ui_interface_line_t *ui_normal_StaticGroup1_LaneLineRight = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[5]);
ui_interface_line_t *ui_normal_StaticGroup1_3mLine = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[6]);

void _ui_init_normal_StaticGroup1_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_StaticGroup1_0.data[i].figure_name[0] = 1;
        ui_normal_StaticGroup1_0.data[i].figure_name[1] = 3;
        ui_normal_StaticGroup1_0.data[i].figure_name[2] = i + 0;
        ui_normal_StaticGroup1_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_normal_StaticGroup1_0.data[i].operate_type = 0;
    }

    ui_normal_StaticGroup1_SuperCapRect->figure_type = 1;
    ui_normal_StaticGroup1_SuperCapRect->operate_type = 1;
    ui_normal_StaticGroup1_SuperCapRect->layer = 0;
    ui_normal_StaticGroup1_SuperCapRect->color = 8;
    ui_normal_StaticGroup1_SuperCapRect->start_x = 727;
    ui_normal_StaticGroup1_SuperCapRect->start_y = 72;
    ui_normal_StaticGroup1_SuperCapRect->width = 3;
    ui_normal_StaticGroup1_SuperCapRect->end_x = 1137;
    ui_normal_StaticGroup1_SuperCapRect->end_y = 92;

    ui_normal_StaticGroup1_SuperCapMidLine->figure_type = 0;
    ui_normal_StaticGroup1_SuperCapMidLine->operate_type = 1;
    ui_normal_StaticGroup1_SuperCapMidLine->layer = 0;
    ui_normal_StaticGroup1_SuperCapMidLine->color = 8;
    ui_normal_StaticGroup1_SuperCapMidLine->start_x = 936;
    ui_normal_StaticGroup1_SuperCapMidLine->start_y = 67;
    ui_normal_StaticGroup1_SuperCapMidLine->width = 3;
    ui_normal_StaticGroup1_SuperCapMidLine->end_x = 936;
    ui_normal_StaticGroup1_SuperCapMidLine->end_y = 100;

    ui_normal_StaticGroup1_5mLine->figure_type = 0;
    ui_normal_StaticGroup1_5mLine->operate_type = 1;
    ui_normal_StaticGroup1_5mLine->layer = 0;
    ui_normal_StaticGroup1_5mLine->color = 8;
    ui_normal_StaticGroup1_5mLine->start_x = 905;
    ui_normal_StaticGroup1_5mLine->start_y = 492;
    ui_normal_StaticGroup1_5mLine->width = 2;
    ui_normal_StaticGroup1_5mLine->end_x = 1021;
    ui_normal_StaticGroup1_5mLine->end_y = 493;

    ui_normal_StaticGroup1_aimLine->figure_type = 0;
    ui_normal_StaticGroup1_aimLine->operate_type = 1;
    ui_normal_StaticGroup1_aimLine->layer = 0;
    ui_normal_StaticGroup1_aimLine->color = 8;
    ui_normal_StaticGroup1_aimLine->start_x = 958;
    ui_normal_StaticGroup1_aimLine->start_y = 311;
    ui_normal_StaticGroup1_aimLine->width = 2;
    ui_normal_StaticGroup1_aimLine->end_x = 963;
    ui_normal_StaticGroup1_aimLine->end_y = 762;

    ui_normal_StaticGroup1_LaneLinerLeft->figure_type = 0;
    ui_normal_StaticGroup1_LaneLinerLeft->operate_type = 1;
    ui_normal_StaticGroup1_LaneLinerLeft->layer = 0;
    ui_normal_StaticGroup1_LaneLinerLeft->color = 8;
    ui_normal_StaticGroup1_LaneLinerLeft->start_x = 532;
    ui_normal_StaticGroup1_LaneLinerLeft->start_y = 0;
    ui_normal_StaticGroup1_LaneLinerLeft->width = 3;
    ui_normal_StaticGroup1_LaneLinerLeft->end_x = 864;
    ui_normal_StaticGroup1_LaneLinerLeft->end_y = 461;

    ui_normal_StaticGroup1_LaneLineRight->figure_type = 0;
    ui_normal_StaticGroup1_LaneLineRight->operate_type = 1;
    ui_normal_StaticGroup1_LaneLineRight->layer = 0;
    ui_normal_StaticGroup1_LaneLineRight->color = 8;
    ui_normal_StaticGroup1_LaneLineRight->start_x = 1380;
    ui_normal_StaticGroup1_LaneLineRight->start_y = 0;
    ui_normal_StaticGroup1_LaneLineRight->width = 3;
    ui_normal_StaticGroup1_LaneLineRight->end_x = 1049;
    ui_normal_StaticGroup1_LaneLineRight->end_y = 466;

    ui_normal_StaticGroup1_3mLine->figure_type = 0;
    ui_normal_StaticGroup1_3mLine->operate_type = 1;
    ui_normal_StaticGroup1_3mLine->layer = 0;
    ui_normal_StaticGroup1_3mLine->color = 8;
    ui_normal_StaticGroup1_3mLine->start_x = 887;
    ui_normal_StaticGroup1_3mLine->start_y = 512;
    ui_normal_StaticGroup1_3mLine->width = 2;
    ui_normal_StaticGroup1_3mLine->end_x = 1047;
    ui_normal_StaticGroup1_3mLine->end_y = 513;


    ui_proc_7_frame(&ui_normal_StaticGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticGroup1_0, sizeof(ui_normal_StaticGroup1_0));
}

void _ui_update_normal_StaticGroup1_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_StaticGroup1_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_normal_StaticGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticGroup1_0, sizeof(ui_normal_StaticGroup1_0));
}

void _ui_remove_normal_StaticGroup1_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_StaticGroup1_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_normal_StaticGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticGroup1_0, sizeof(ui_normal_StaticGroup1_0));
}


void ui_init_normal_StaticGroup1() {
    _ui_init_normal_StaticGroup1_0();
}

void ui_update_normal_StaticGroup1() {
    _ui_update_normal_StaticGroup1_0();
}

void ui_remove_normal_StaticGroup1() {
    _ui_remove_normal_StaticGroup1_0();
}


ui_string_frame_t ui_normal_StaticTextGroup1_0;
ui_interface_string_t* ui_normal_StaticTextGroup1_FricOnText = &(ui_normal_StaticTextGroup1_0.option);

void _ui_init_normal_StaticTextGroup1_0() {
    ui_normal_StaticTextGroup1_0.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_0.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_0.option.figure_name[2] = 0;
    ui_normal_StaticTextGroup1_0.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricOnText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricOnText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricOnText->layer = 0;
    ui_normal_StaticTextGroup1_FricOnText->color = 8;
    ui_normal_StaticTextGroup1_FricOnText->start_x = 1727;
    ui_normal_StaticTextGroup1_FricOnText->start_y = 854;
    ui_normal_StaticTextGroup1_FricOnText->width = 2;
    ui_normal_StaticTextGroup1_FricOnText->font_size = 15;
    ui_normal_StaticTextGroup1_FricOnText->str_length = 7;
    strcpy(ui_normal_StaticTextGroup1_FricOnText->string, "FRIC ON");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_0, sizeof(ui_normal_StaticTextGroup1_0));
}

void _ui_update_normal_StaticTextGroup1_0() {
    ui_normal_StaticTextGroup1_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_0, sizeof(ui_normal_StaticTextGroup1_0));
}

void _ui_remove_normal_StaticTextGroup1_0() {
    ui_normal_StaticTextGroup1_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_0, sizeof(ui_normal_StaticTextGroup1_0));
}
ui_string_frame_t ui_normal_StaticTextGroup1_1;
ui_interface_string_t* ui_normal_StaticTextGroup1_FricOffText = &(ui_normal_StaticTextGroup1_1.option);

void _ui_init_normal_StaticTextGroup1_1() {
    ui_normal_StaticTextGroup1_1.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_1.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_1.option.figure_name[2] = 1;
    ui_normal_StaticTextGroup1_1.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricOffText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricOffText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricOffText->layer = 0;
    ui_normal_StaticTextGroup1_FricOffText->color = 8;
    ui_normal_StaticTextGroup1_FricOffText->start_x = 1725;
    ui_normal_StaticTextGroup1_FricOffText->start_y = 815;
    ui_normal_StaticTextGroup1_FricOffText->width = 2;
    ui_normal_StaticTextGroup1_FricOffText->font_size = 15;
    ui_normal_StaticTextGroup1_FricOffText->str_length = 8;
    strcpy(ui_normal_StaticTextGroup1_FricOffText->string, "FRIC OFF");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_1, sizeof(ui_normal_StaticTextGroup1_1));
}

void _ui_update_normal_StaticTextGroup1_1() {
    ui_normal_StaticTextGroup1_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_1, sizeof(ui_normal_StaticTextGroup1_1));
}

void _ui_remove_normal_StaticTextGroup1_1() {
    ui_normal_StaticTextGroup1_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_1, sizeof(ui_normal_StaticTextGroup1_1));
}
ui_string_frame_t ui_normal_StaticTextGroup1_2;
ui_interface_string_t* ui_normal_StaticTextGroup1_FricErrorText = &(ui_normal_StaticTextGroup1_2.option);

void _ui_init_normal_StaticTextGroup1_2() {
    ui_normal_StaticTextGroup1_2.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_2.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_2.option.figure_name[2] = 2;
    ui_normal_StaticTextGroup1_2.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricErrorText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricErrorText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricErrorText->layer = 0;
    ui_normal_StaticTextGroup1_FricErrorText->color = 5;
    ui_normal_StaticTextGroup1_FricErrorText->start_x = 1716;
    ui_normal_StaticTextGroup1_FricErrorText->start_y = 774;
    ui_normal_StaticTextGroup1_FricErrorText->width = 2;
    ui_normal_StaticTextGroup1_FricErrorText->font_size = 15;
    ui_normal_StaticTextGroup1_FricErrorText->str_length = 11;
    strcpy(ui_normal_StaticTextGroup1_FricErrorText->string, "FRIC ERR !!");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_2, sizeof(ui_normal_StaticTextGroup1_2));
}

void _ui_update_normal_StaticTextGroup1_2() {
    ui_normal_StaticTextGroup1_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_2, sizeof(ui_normal_StaticTextGroup1_2));
}

void _ui_remove_normal_StaticTextGroup1_2() {
    ui_normal_StaticTextGroup1_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_2, sizeof(ui_normal_StaticTextGroup1_2));
}
ui_string_frame_t ui_normal_StaticTextGroup1_3;
ui_interface_string_t* ui_normal_StaticTextGroup1_ChassisNormalText = &(ui_normal_StaticTextGroup1_3.option);

void _ui_init_normal_StaticTextGroup1_3() {
    ui_normal_StaticTextGroup1_3.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_3.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_3.option.figure_name[2] = 3;
    ui_normal_StaticTextGroup1_3.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisNormalText->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisNormalText->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisNormalText->layer = 0;
    ui_normal_StaticTextGroup1_ChassisNormalText->color = 8;
    ui_normal_StaticTextGroup1_ChassisNormalText->start_x = 1570;
    ui_normal_StaticTextGroup1_ChassisNormalText->start_y = 853;
    ui_normal_StaticTextGroup1_ChassisNormalText->width = 2;
    ui_normal_StaticTextGroup1_ChassisNormalText->font_size = 15;
    ui_normal_StaticTextGroup1_ChassisNormalText->str_length = 6;
    strcpy(ui_normal_StaticTextGroup1_ChassisNormalText->string, "NORMAL");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_3);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_3, sizeof(ui_normal_StaticTextGroup1_3));
}

void _ui_update_normal_StaticTextGroup1_3() {
    ui_normal_StaticTextGroup1_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_3);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_3, sizeof(ui_normal_StaticTextGroup1_3));
}

void _ui_remove_normal_StaticTextGroup1_3() {
    ui_normal_StaticTextGroup1_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_3);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_3, sizeof(ui_normal_StaticTextGroup1_3));
}
ui_string_frame_t ui_normal_StaticTextGroup1_4;
ui_interface_string_t* ui_normal_StaticTextGroup1_ChassisStopText = &(ui_normal_StaticTextGroup1_4.option);

void _ui_init_normal_StaticTextGroup1_4() {
    ui_normal_StaticTextGroup1_4.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_4.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_4.option.figure_name[2] = 4;
    ui_normal_StaticTextGroup1_4.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisStopText->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisStopText->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisStopText->layer = 0;
    ui_normal_StaticTextGroup1_ChassisStopText->color = 8;
    ui_normal_StaticTextGroup1_ChassisStopText->start_x = 1586;
    ui_normal_StaticTextGroup1_ChassisStopText->start_y = 815;
    ui_normal_StaticTextGroup1_ChassisStopText->width = 2;
    ui_normal_StaticTextGroup1_ChassisStopText->font_size = 15;
    ui_normal_StaticTextGroup1_ChassisStopText->str_length = 4;
    strcpy(ui_normal_StaticTextGroup1_ChassisStopText->string, "STOP");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_4);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_4, sizeof(ui_normal_StaticTextGroup1_4));
}

void _ui_update_normal_StaticTextGroup1_4() {
    ui_normal_StaticTextGroup1_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_4);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_4, sizeof(ui_normal_StaticTextGroup1_4));
}

void _ui_remove_normal_StaticTextGroup1_4() {
    ui_normal_StaticTextGroup1_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_4);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_4, sizeof(ui_normal_StaticTextGroup1_4));
}
ui_string_frame_t ui_normal_StaticTextGroup1_5;
ui_interface_string_t* ui_normal_StaticTextGroup1_ChassisStep1 = &(ui_normal_StaticTextGroup1_5.option);

void _ui_init_normal_StaticTextGroup1_5() {
    ui_normal_StaticTextGroup1_5.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_5.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_5.option.figure_name[2] = 5;
    ui_normal_StaticTextGroup1_5.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisStep1->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisStep1->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisStep1->layer = 0;
    ui_normal_StaticTextGroup1_ChassisStep1->color = 8;
    ui_normal_StaticTextGroup1_ChassisStep1->start_x = 1570;
    ui_normal_StaticTextGroup1_ChassisStep1->start_y = 778;
    ui_normal_StaticTextGroup1_ChassisStep1->width = 2;
    ui_normal_StaticTextGroup1_ChassisStep1->font_size = 15;
    ui_normal_StaticTextGroup1_ChassisStep1->str_length = 6;
    strcpy(ui_normal_StaticTextGroup1_ChassisStep1->string, "STEP 1");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_5);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_5, sizeof(ui_normal_StaticTextGroup1_5));
}

void _ui_update_normal_StaticTextGroup1_5() {
    ui_normal_StaticTextGroup1_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_5);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_5, sizeof(ui_normal_StaticTextGroup1_5));
}

void _ui_remove_normal_StaticTextGroup1_5() {
    ui_normal_StaticTextGroup1_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_5);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_5, sizeof(ui_normal_StaticTextGroup1_5));
}
ui_string_frame_t ui_normal_StaticTextGroup1_6;
ui_interface_string_t* ui_normal_StaticTextGroup1_ChassisStep2 = &(ui_normal_StaticTextGroup1_6.option);

void _ui_init_normal_StaticTextGroup1_6() {
    ui_normal_StaticTextGroup1_6.option.figure_name[0] = 1;
    ui_normal_StaticTextGroup1_6.option.figure_name[1] = 4;
    ui_normal_StaticTextGroup1_6.option.figure_name[2] = 6;
    ui_normal_StaticTextGroup1_6.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisStep2->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisStep2->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisStep2->layer = 0;
    ui_normal_StaticTextGroup1_ChassisStep2->color = 8;
    ui_normal_StaticTextGroup1_ChassisStep2->start_x = 1571;
    ui_normal_StaticTextGroup1_ChassisStep2->start_y = 738;
    ui_normal_StaticTextGroup1_ChassisStep2->width = 2;
    ui_normal_StaticTextGroup1_ChassisStep2->font_size = 15;
    ui_normal_StaticTextGroup1_ChassisStep2->str_length = 6;
    strcpy(ui_normal_StaticTextGroup1_ChassisStep2->string, "STEP 2");


    ui_proc_string_frame(&ui_normal_StaticTextGroup1_6);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_6, sizeof(ui_normal_StaticTextGroup1_6));
}

void _ui_update_normal_StaticTextGroup1_6() {
    ui_normal_StaticTextGroup1_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_6);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_6, sizeof(ui_normal_StaticTextGroup1_6));
}

void _ui_remove_normal_StaticTextGroup1_6() {
    ui_normal_StaticTextGroup1_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_StaticTextGroup1_6);
    SEND_MESSAGE((uint8_t *) &ui_normal_StaticTextGroup1_6, sizeof(ui_normal_StaticTextGroup1_6));
}

void ui_init_normal_StaticTextGroup1() {
    _ui_init_normal_StaticTextGroup1_0();
    _ui_init_normal_StaticTextGroup1_1();
    _ui_init_normal_StaticTextGroup1_2();
    _ui_init_normal_StaticTextGroup1_3();
    _ui_init_normal_StaticTextGroup1_4();
    _ui_init_normal_StaticTextGroup1_5();
    _ui_init_normal_StaticTextGroup1_6();
}

void ui_update_normal_StaticTextGroup1() {
    _ui_update_normal_StaticTextGroup1_0();
    _ui_update_normal_StaticTextGroup1_1();
    _ui_update_normal_StaticTextGroup1_2();
    _ui_update_normal_StaticTextGroup1_3();
    _ui_update_normal_StaticTextGroup1_4();
    _ui_update_normal_StaticTextGroup1_5();
    _ui_update_normal_StaticTextGroup1_6();
}

void ui_remove_normal_StaticTextGroup1() {
    _ui_remove_normal_StaticTextGroup1_0();
    _ui_remove_normal_StaticTextGroup1_1();
    _ui_remove_normal_StaticTextGroup1_2();
    _ui_remove_normal_StaticTextGroup1_3();
    _ui_remove_normal_StaticTextGroup1_4();
    _ui_remove_normal_StaticTextGroup1_5();
    _ui_remove_normal_StaticTextGroup1_6();
}

