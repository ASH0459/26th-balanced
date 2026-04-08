//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"


ui_string_frame_t ui_normal_ChangeTextGroup_0;
ui_interface_string_t* ui_normal_ChangeTextGroup_DbusErr = &(ui_normal_ChangeTextGroup_0.option);

void _ui_init_normal_ChangeTextGroup_0() {
    ui_normal_ChangeTextGroup_0.option.figure_name[0] = 1;
    ui_normal_ChangeTextGroup_0.option.figure_name[1] = 0;
    ui_normal_ChangeTextGroup_0.option.figure_name[2] = 0;
    ui_normal_ChangeTextGroup_0.option.operate_type = 1;

    ui_normal_ChangeTextGroup_DbusErr->figure_type = 7;
    ui_normal_ChangeTextGroup_DbusErr->operate_type = 1;
    ui_normal_ChangeTextGroup_DbusErr->layer = 0;
    ui_normal_ChangeTextGroup_DbusErr->color = 4;
    ui_normal_ChangeTextGroup_DbusErr->start_x = 1583;
    ui_normal_ChangeTextGroup_DbusErr->start_y = 414;
    ui_normal_ChangeTextGroup_DbusErr->width = 5;
    ui_normal_ChangeTextGroup_DbusErr->font_size = 50;
    ui_normal_ChangeTextGroup_DbusErr->str_length = 4;
    strcpy(ui_normal_ChangeTextGroup_DbusErr->string, "DUBS");


    ui_proc_string_frame(&ui_normal_ChangeTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_0, sizeof(ui_normal_ChangeTextGroup_0));
}

void _ui_update_normal_ChangeTextGroup_0() {
    ui_normal_ChangeTextGroup_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_0, sizeof(ui_normal_ChangeTextGroup_0));
}

void _ui_remove_normal_ChangeTextGroup_0() {
    ui_normal_ChangeTextGroup_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_0, sizeof(ui_normal_ChangeTextGroup_0));
}
ui_string_frame_t ui_normal_ChangeTextGroup_1;
ui_interface_string_t* ui_normal_ChangeTextGroup_VTErr = &(ui_normal_ChangeTextGroup_1.option);

void _ui_init_normal_ChangeTextGroup_1() {
    ui_normal_ChangeTextGroup_1.option.figure_name[0] = 1;
    ui_normal_ChangeTextGroup_1.option.figure_name[1] = 0;
    ui_normal_ChangeTextGroup_1.option.figure_name[2] = 1;
    ui_normal_ChangeTextGroup_1.option.operate_type = 1;

    ui_normal_ChangeTextGroup_VTErr->figure_type = 7;
    ui_normal_ChangeTextGroup_VTErr->operate_type = 1;
    ui_normal_ChangeTextGroup_VTErr->layer = 0;
    ui_normal_ChangeTextGroup_VTErr->color = 4;
    ui_normal_ChangeTextGroup_VTErr->start_x = 1633;
    ui_normal_ChangeTextGroup_VTErr->start_y = 410;
    ui_normal_ChangeTextGroup_VTErr->width = 5;
    ui_normal_ChangeTextGroup_VTErr->font_size = 50;
    ui_normal_ChangeTextGroup_VTErr->str_length = 2;
    strcpy(ui_normal_ChangeTextGroup_VTErr->string, "VT");


    ui_proc_string_frame(&ui_normal_ChangeTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_1, sizeof(ui_normal_ChangeTextGroup_1));
}

void _ui_update_normal_ChangeTextGroup_1() {
    ui_normal_ChangeTextGroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_1, sizeof(ui_normal_ChangeTextGroup_1));
}

void _ui_remove_normal_ChangeTextGroup_1() {
    ui_normal_ChangeTextGroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_1, sizeof(ui_normal_ChangeTextGroup_1));
}
ui_string_frame_t ui_normal_ChangeTextGroup_2;
ui_interface_string_t* ui_normal_ChangeTextGroup_Jont1Err = &(ui_normal_ChangeTextGroup_2.option);

void _ui_init_normal_ChangeTextGroup_2() {
    ui_normal_ChangeTextGroup_2.option.figure_name[0] = 1;
    ui_normal_ChangeTextGroup_2.option.figure_name[1] = 0;
    ui_normal_ChangeTextGroup_2.option.figure_name[2] = 2;
    ui_normal_ChangeTextGroup_2.option.operate_type = 1;

    ui_normal_ChangeTextGroup_Jont1Err->figure_type = 7;
    ui_normal_ChangeTextGroup_Jont1Err->operate_type = 1;
    ui_normal_ChangeTextGroup_Jont1Err->layer = 0;
    ui_normal_ChangeTextGroup_Jont1Err->color = 4;
    ui_normal_ChangeTextGroup_Jont1Err->start_x = 1500;
    ui_normal_ChangeTextGroup_Jont1Err->start_y = 412;
    ui_normal_ChangeTextGroup_Jont1Err->width = 5;
    ui_normal_ChangeTextGroup_Jont1Err->font_size = 50;
    ui_normal_ChangeTextGroup_Jont1Err->str_length = 7;
    strcpy(ui_normal_ChangeTextGroup_Jont1Err->string, "JOINT 1");


    ui_proc_string_frame(&ui_normal_ChangeTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_2, sizeof(ui_normal_ChangeTextGroup_2));
}

void _ui_update_normal_ChangeTextGroup_2() {
    ui_normal_ChangeTextGroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_2, sizeof(ui_normal_ChangeTextGroup_2));
}

void _ui_remove_normal_ChangeTextGroup_2() {
    ui_normal_ChangeTextGroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_normal_ChangeTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_normal_ChangeTextGroup_2, sizeof(ui_normal_ChangeTextGroup_2));
}

void ui_init_normal_ChangeTextGroup() {
    _ui_init_normal_ChangeTextGroup_0();
    _ui_init_normal_ChangeTextGroup_1();
    _ui_init_normal_ChangeTextGroup_2();
}

void ui_update_normal_ChangeTextGroup() {
    _ui_update_normal_ChangeTextGroup_0();
    _ui_update_normal_ChangeTextGroup_1();
    _ui_update_normal_ChangeTextGroup_2();
}

void ui_remove_normal_ChangeTextGroup() {
    _ui_remove_normal_ChangeTextGroup_0();
    _ui_remove_normal_ChangeTextGroup_1();
    _ui_remove_normal_ChangeTextGroup_2();
}

ui_5_frame_t ui_normal_DynamicGroup1_0;

ui_interface_rect_t *ui_normal_DynamicGroup1_FricOrNot = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[0]);
ui_interface_rect_t *ui_normal_DynamicGroup1_FricStateRect = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[1]);
ui_interface_arc_t *ui_normal_DynamicGroup1_SpinArc = (ui_interface_arc_t*)&(ui_normal_DynamicGroup1_0.data[2]);
ui_interface_line_t *ui_normal_DynamicGroup1_SuperCapPercentage = (ui_interface_line_t*)&(ui_normal_DynamicGroup1_0.data[3]);
ui_interface_rect_t *ui_normal_DynamicGroup1_ChassisStateRect = (ui_interface_rect_t*)&(ui_normal_DynamicGroup1_0.data[4]);

void _ui_init_normal_DynamicGroup1_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_DynamicGroup1_0.data[i].figure_name[0] = 1;
        ui_normal_DynamicGroup1_0.data[i].figure_name[1] = 1;
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
    ui_normal_DynamicGroup1_FricOrNot->start_x = 590;
    ui_normal_DynamicGroup1_FricOrNot->start_y = 233;
    ui_normal_DynamicGroup1_FricOrNot->width = 5;
    ui_normal_DynamicGroup1_FricOrNot->end_x = 1336;
    ui_normal_DynamicGroup1_FricOrNot->end_y = 866;

    ui_normal_DynamicGroup1_FricStateRect->figure_type = 1;
    ui_normal_DynamicGroup1_FricStateRect->operate_type = 1;
    ui_normal_DynamicGroup1_FricStateRect->layer = 0;
    ui_normal_DynamicGroup1_FricStateRect->color = 8;
    ui_normal_DynamicGroup1_FricStateRect->start_x = 1629;
    ui_normal_DynamicGroup1_FricStateRect->start_y = 823;
    ui_normal_DynamicGroup1_FricStateRect->width = 5;
    ui_normal_DynamicGroup1_FricStateRect->end_x = 1787;
    ui_normal_DynamicGroup1_FricStateRect->end_y = 880;

    ui_normal_DynamicGroup1_SpinArc->figure_type = 4;
    ui_normal_DynamicGroup1_SpinArc->operate_type = 1;
    ui_normal_DynamicGroup1_SpinArc->layer = 0;
    ui_normal_DynamicGroup1_SpinArc->color = 6;
    ui_normal_DynamicGroup1_SpinArc->start_x = 1693;
    ui_normal_DynamicGroup1_SpinArc->start_y = 545;
    ui_normal_DynamicGroup1_SpinArc->width = 10;
    ui_normal_DynamicGroup1_SpinArc->start_angle = 315;
    ui_normal_DynamicGroup1_SpinArc->end_angle = 45;
    ui_normal_DynamicGroup1_SpinArc->rx = 90;
    ui_normal_DynamicGroup1_SpinArc->ry = 90;

    ui_normal_DynamicGroup1_SuperCapPercentage->figure_type = 0;
    ui_normal_DynamicGroup1_SuperCapPercentage->operate_type = 1;
    ui_normal_DynamicGroup1_SuperCapPercentage->layer = 0;
    ui_normal_DynamicGroup1_SuperCapPercentage->color = 2;
    ui_normal_DynamicGroup1_SuperCapPercentage->start_x = 710;
    ui_normal_DynamicGroup1_SuperCapPercentage->start_y = 95;
    ui_normal_DynamicGroup1_SuperCapPercentage->width = 40;
    ui_normal_DynamicGroup1_SuperCapPercentage->end_x = 1210;
    ui_normal_DynamicGroup1_SuperCapPercentage->end_y = 95;

    ui_normal_DynamicGroup1_ChassisStateRect->figure_type = 1;
    ui_normal_DynamicGroup1_ChassisStateRect->operate_type = 1;
    ui_normal_DynamicGroup1_ChassisStateRect->layer = 0;
    ui_normal_DynamicGroup1_ChassisStateRect->color = 8;
    ui_normal_DynamicGroup1_ChassisStateRect->start_x = 1436;
    ui_normal_DynamicGroup1_ChassisStateRect->start_y = 822;
    ui_normal_DynamicGroup1_ChassisStateRect->width = 5;
    ui_normal_DynamicGroup1_ChassisStateRect->end_x = 1563;
    ui_normal_DynamicGroup1_ChassisStateRect->end_y = 882;


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
    ui_normal_DynamicTextGroup1_0.option.figure_name[1] = 2;
    ui_normal_DynamicTextGroup1_0.option.figure_name[2] = 0;
    ui_normal_DynamicTextGroup1_0.option.operate_type = 1;

    ui_normal_DynamicTextGroup1_LowAmmoText->figure_type = 7;
    ui_normal_DynamicTextGroup1_LowAmmoText->operate_type = 1;
    ui_normal_DynamicTextGroup1_LowAmmoText->layer = 0;
    ui_normal_DynamicTextGroup1_LowAmmoText->color = 5;
    ui_normal_DynamicTextGroup1_LowAmmoText->start_x = 683;
    ui_normal_DynamicTextGroup1_LowAmmoText->start_y = 844;
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

ui_5_frame_t ui_normal_LegDynamicGroup_0;

ui_interface_line_t *ui_normal_LegDynamicGroup_Leg1 = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[0]);
ui_interface_line_t *ui_normal_LegDynamicGroup_Leg2 = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[1]);
ui_interface_line_t *ui_normal_LegDynamicGroup_Leg3 = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[2]);
ui_interface_line_t *ui_normal_LegDynamicGroup_Leg4 = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[3]);
ui_interface_line_t *ui_normal_LegDynamicGroup_Leg5 = (ui_interface_line_t*)&(ui_normal_LegDynamicGroup_0.data[4]);

void _ui_init_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_LegDynamicGroup_0.data[i].figure_name[0] = 1;
        ui_normal_LegDynamicGroup_0.data[i].figure_name[1] = 3;
        ui_normal_LegDynamicGroup_0.data[i].figure_name[2] = i + 0;
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 0;
    }

    ui_normal_LegDynamicGroup_Leg1->figure_type = 0;
    ui_normal_LegDynamicGroup_Leg1->operate_type = 1;
    ui_normal_LegDynamicGroup_Leg1->layer = 0;
    ui_normal_LegDynamicGroup_Leg1->color = 0;
    ui_normal_LegDynamicGroup_Leg1->start_x = 159;
    ui_normal_LegDynamicGroup_Leg1->start_y = 783;
    ui_normal_LegDynamicGroup_Leg1->width = 5;
    ui_normal_LegDynamicGroup_Leg1->end_x = 236;
    ui_normal_LegDynamicGroup_Leg1->end_y = 711;

    ui_normal_LegDynamicGroup_Leg2->figure_type = 0;
    ui_normal_LegDynamicGroup_Leg2->operate_type = 1;
    ui_normal_LegDynamicGroup_Leg2->layer = 0;
    ui_normal_LegDynamicGroup_Leg2->color = 0;
    ui_normal_LegDynamicGroup_Leg2->start_x = 237;
    ui_normal_LegDynamicGroup_Leg2->start_y = 716;
    ui_normal_LegDynamicGroup_Leg2->width = 5;
    ui_normal_LegDynamicGroup_Leg2->end_x = 146;
    ui_normal_LegDynamicGroup_Leg2->end_y = 617;

    ui_normal_LegDynamicGroup_Leg3->figure_type = 0;
    ui_normal_LegDynamicGroup_Leg3->operate_type = 1;
    ui_normal_LegDynamicGroup_Leg3->layer = 0;
    ui_normal_LegDynamicGroup_Leg3->color = 0;
    ui_normal_LegDynamicGroup_Leg3->start_x = 332;
    ui_normal_LegDynamicGroup_Leg3->start_y = 776;
    ui_normal_LegDynamicGroup_Leg3->width = 5;
    ui_normal_LegDynamicGroup_Leg3->end_x = 409;
    ui_normal_LegDynamicGroup_Leg3->end_y = 704;

    ui_normal_LegDynamicGroup_Leg4->figure_type = 0;
    ui_normal_LegDynamicGroup_Leg4->operate_type = 1;
    ui_normal_LegDynamicGroup_Leg4->layer = 0;
    ui_normal_LegDynamicGroup_Leg4->color = 0;
    ui_normal_LegDynamicGroup_Leg4->start_x = 410;
    ui_normal_LegDynamicGroup_Leg4->start_y = 709;
    ui_normal_LegDynamicGroup_Leg4->width = 5;
    ui_normal_LegDynamicGroup_Leg4->end_x = 319;
    ui_normal_LegDynamicGroup_Leg4->end_y = 610;

    ui_normal_LegDynamicGroup_Leg5->figure_type = 0;
    ui_normal_LegDynamicGroup_Leg5->operate_type = 1;
    ui_normal_LegDynamicGroup_Leg5->layer = 0;
    ui_normal_LegDynamicGroup_Leg5->color = 0;
    ui_normal_LegDynamicGroup_Leg5->start_x = 111;
    ui_normal_LegDynamicGroup_Leg5->start_y = 792;
    ui_normal_LegDynamicGroup_Leg5->width = 5;
    ui_normal_LegDynamicGroup_Leg5->end_x = 411;
    ui_normal_LegDynamicGroup_Leg5->end_y = 791;


    ui_proc_5_frame(&ui_normal_LegDynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_LegDynamicGroup_0, sizeof(ui_normal_LegDynamicGroup_0));
}

void _ui_update_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_normal_LegDynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_normal_LegDynamicGroup_0, sizeof(ui_normal_LegDynamicGroup_0));
}

void _ui_remove_normal_LegDynamicGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_normal_LegDynamicGroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_normal_LegDynamicGroup_0);
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
ui_interface_round_t *ui_normal_StaticGroup1_aiming = (ui_interface_round_t*)&(ui_normal_StaticGroup1_0.data[2]);
ui_interface_round_t *ui_normal_StaticGroup1_SpinRound = (ui_interface_round_t*)&(ui_normal_StaticGroup1_0.data[3]);
ui_interface_line_t *ui_normal_StaticGroup1_LaneLinerLeft = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[4]);
ui_interface_line_t *ui_normal_StaticGroup1_LaneLineRight = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[5]);
ui_interface_line_t *ui_normal_StaticGroup1_RemainLine = (ui_interface_line_t*)&(ui_normal_StaticGroup1_0.data[6]);

void _ui_init_normal_StaticGroup1_0() {
    for (int i = 0; i < 7; i++) {
        ui_normal_StaticGroup1_0.data[i].figure_name[0] = 1;
        ui_normal_StaticGroup1_0.data[i].figure_name[1] = 4;
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
    ui_normal_StaticGroup1_SuperCapRect->start_x = 710;
    ui_normal_StaticGroup1_SuperCapRect->start_y = 90;
    ui_normal_StaticGroup1_SuperCapRect->width = 5;
    ui_normal_StaticGroup1_SuperCapRect->end_x = 1210;
    ui_normal_StaticGroup1_SuperCapRect->end_y = 140;

    ui_normal_StaticGroup1_SuperCapMidLine->figure_type = 0;
    ui_normal_StaticGroup1_SuperCapMidLine->operate_type = 1;
    ui_normal_StaticGroup1_SuperCapMidLine->layer = 0;
    ui_normal_StaticGroup1_SuperCapMidLine->color = 8;
    ui_normal_StaticGroup1_SuperCapMidLine->start_x = 955;
    ui_normal_StaticGroup1_SuperCapMidLine->start_y = 70;
    ui_normal_StaticGroup1_SuperCapMidLine->width = 10;
    ui_normal_StaticGroup1_SuperCapMidLine->end_x = 955;
    ui_normal_StaticGroup1_SuperCapMidLine->end_y = 160;

    ui_normal_StaticGroup1_aiming->figure_type = 2;
    ui_normal_StaticGroup1_aiming->operate_type = 1;
    ui_normal_StaticGroup1_aiming->layer = 0;
    ui_normal_StaticGroup1_aiming->color = 8;
    ui_normal_StaticGroup1_aiming->start_x = 943;
    ui_normal_StaticGroup1_aiming->start_y = 537;
    ui_normal_StaticGroup1_aiming->width = 10;
    ui_normal_StaticGroup1_aiming->r = 14;

    ui_normal_StaticGroup1_SpinRound->figure_type = 2;
    ui_normal_StaticGroup1_SpinRound->operate_type = 1;
    ui_normal_StaticGroup1_SpinRound->layer = 0;
    ui_normal_StaticGroup1_SpinRound->color = 8;
    ui_normal_StaticGroup1_SpinRound->start_x = 1691;
    ui_normal_StaticGroup1_SpinRound->start_y = 537;
    ui_normal_StaticGroup1_SpinRound->width = 15;
    ui_normal_StaticGroup1_SpinRound->r = 80;

    ui_normal_StaticGroup1_LaneLinerLeft->figure_type = 0;
    ui_normal_StaticGroup1_LaneLinerLeft->operate_type = 1;
    ui_normal_StaticGroup1_LaneLinerLeft->layer = 0;
    ui_normal_StaticGroup1_LaneLinerLeft->color = 8;
    ui_normal_StaticGroup1_LaneLinerLeft->start_x = 561;
    ui_normal_StaticGroup1_LaneLinerLeft->start_y = 66;
    ui_normal_StaticGroup1_LaneLinerLeft->width = 5;
    ui_normal_StaticGroup1_LaneLinerLeft->end_x = 757;
    ui_normal_StaticGroup1_LaneLinerLeft->end_y = 469;

    ui_normal_StaticGroup1_LaneLineRight->figure_type = 0;
    ui_normal_StaticGroup1_LaneLineRight->operate_type = 1;
    ui_normal_StaticGroup1_LaneLineRight->layer = 0;
    ui_normal_StaticGroup1_LaneLineRight->color = 8;
    ui_normal_StaticGroup1_LaneLineRight->start_x = 1326;
    ui_normal_StaticGroup1_LaneLineRight->start_y = 64;
    ui_normal_StaticGroup1_LaneLineRight->width = 5;
    ui_normal_StaticGroup1_LaneLineRight->end_x = 1145;
    ui_normal_StaticGroup1_LaneLineRight->end_y = 475;

    ui_normal_StaticGroup1_RemainLine->figure_type = 0;
    ui_normal_StaticGroup1_RemainLine->operate_type = 1;
    ui_normal_StaticGroup1_RemainLine->layer = 0;
    ui_normal_StaticGroup1_RemainLine->color = 8;
    ui_normal_StaticGroup1_RemainLine->start_x = 1023;
    ui_normal_StaticGroup1_RemainLine->start_y = 377;
    ui_normal_StaticGroup1_RemainLine->width = 5;
    ui_normal_StaticGroup1_RemainLine->end_x = 1026;
    ui_normal_StaticGroup1_RemainLine->end_y = 737;


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
    ui_normal_StaticTextGroup1_0.option.figure_name[1] = 5;
    ui_normal_StaticTextGroup1_0.option.figure_name[2] = 0;
    ui_normal_StaticTextGroup1_0.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricOnText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricOnText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricOnText->layer = 0;
    ui_normal_StaticTextGroup1_FricOnText->color = 8;
    ui_normal_StaticTextGroup1_FricOnText->start_x = 1633;
    ui_normal_StaticTextGroup1_FricOnText->start_y = 860;
    ui_normal_StaticTextGroup1_FricOnText->width = 2;
    ui_normal_StaticTextGroup1_FricOnText->font_size = 20;
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
    ui_normal_StaticTextGroup1_1.option.figure_name[1] = 5;
    ui_normal_StaticTextGroup1_1.option.figure_name[2] = 1;
    ui_normal_StaticTextGroup1_1.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricOffText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricOffText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricOffText->layer = 0;
    ui_normal_StaticTextGroup1_FricOffText->color = 8;
    ui_normal_StaticTextGroup1_FricOffText->start_x = 1634;
    ui_normal_StaticTextGroup1_FricOffText->start_y = 805;
    ui_normal_StaticTextGroup1_FricOffText->width = 2;
    ui_normal_StaticTextGroup1_FricOffText->font_size = 20;
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
    ui_normal_StaticTextGroup1_2.option.figure_name[1] = 5;
    ui_normal_StaticTextGroup1_2.option.figure_name[2] = 2;
    ui_normal_StaticTextGroup1_2.option.operate_type = 1;

    ui_normal_StaticTextGroup1_FricErrorText->figure_type = 7;
    ui_normal_StaticTextGroup1_FricErrorText->operate_type = 1;
    ui_normal_StaticTextGroup1_FricErrorText->layer = 0;
    ui_normal_StaticTextGroup1_FricErrorText->color = 5;
    ui_normal_StaticTextGroup1_FricErrorText->start_x = 1632;
    ui_normal_StaticTextGroup1_FricErrorText->start_y = 762;
    ui_normal_StaticTextGroup1_FricErrorText->width = 2;
    ui_normal_StaticTextGroup1_FricErrorText->font_size = 20;
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
    ui_normal_StaticTextGroup1_3.option.figure_name[1] = 5;
    ui_normal_StaticTextGroup1_3.option.figure_name[2] = 3;
    ui_normal_StaticTextGroup1_3.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisNormalText->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisNormalText->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisNormalText->layer = 0;
    ui_normal_StaticTextGroup1_ChassisNormalText->color = 8;
    ui_normal_StaticTextGroup1_ChassisNormalText->start_x = 1447;
    ui_normal_StaticTextGroup1_ChassisNormalText->start_y = 862;
    ui_normal_StaticTextGroup1_ChassisNormalText->width = 2;
    ui_normal_StaticTextGroup1_ChassisNormalText->font_size = 20;
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
    ui_normal_StaticTextGroup1_4.option.figure_name[1] = 5;
    ui_normal_StaticTextGroup1_4.option.figure_name[2] = 4;
    ui_normal_StaticTextGroup1_4.option.operate_type = 1;

    ui_normal_StaticTextGroup1_ChassisStopText->figure_type = 7;
    ui_normal_StaticTextGroup1_ChassisStopText->operate_type = 1;
    ui_normal_StaticTextGroup1_ChassisStopText->layer = 0;
    ui_normal_StaticTextGroup1_ChassisStopText->color = 8;
    ui_normal_StaticTextGroup1_ChassisStopText->start_x = 1461;
    ui_normal_StaticTextGroup1_ChassisStopText->start_y = 805;
    ui_normal_StaticTextGroup1_ChassisStopText->width = 2;
    ui_normal_StaticTextGroup1_ChassisStopText->font_size = 20;
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

void ui_init_normal_StaticTextGroup1() {
    _ui_init_normal_StaticTextGroup1_0();
    _ui_init_normal_StaticTextGroup1_1();
    _ui_init_normal_StaticTextGroup1_2();
    _ui_init_normal_StaticTextGroup1_3();
    _ui_init_normal_StaticTextGroup1_4();
}

void ui_update_normal_StaticTextGroup1() {
    _ui_update_normal_StaticTextGroup1_0();
    _ui_update_normal_StaticTextGroup1_1();
    _ui_update_normal_StaticTextGroup1_2();
    _ui_update_normal_StaticTextGroup1_3();
    _ui_update_normal_StaticTextGroup1_4();
}

void ui_remove_normal_StaticTextGroup1() {
    _ui_remove_normal_StaticTextGroup1_0();
    _ui_remove_normal_StaticTextGroup1_1();
    _ui_remove_normal_StaticTextGroup1_2();
    _ui_remove_normal_StaticTextGroup1_3();
    _ui_remove_normal_StaticTextGroup1_4();
}

