//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"


ui_string_frame_t ui_start_start_group_0;
ui_interface_string_t* ui_start_start_group_RobotZ = &(ui_start_start_group_0.option);

void _ui_init_start_start_group_0() {
    ui_start_start_group_0.option.figure_name[0] = 1;
    ui_start_start_group_0.option.figure_name[1] = 0;
    ui_start_start_group_0.option.figure_name[2] = 0;
    ui_start_start_group_0.option.operate_type = 1;

    ui_start_start_group_RobotZ->figure_type = 7;
    ui_start_start_group_RobotZ->operate_type = 1;
    ui_start_start_group_RobotZ->layer = 0;
    ui_start_start_group_RobotZ->color = 0;
    ui_start_start_group_RobotZ->start_x = 422;
    ui_start_start_group_RobotZ->start_y = 862;
    ui_start_start_group_RobotZ->width = 20;
    ui_start_start_group_RobotZ->font_size = 200;
    ui_start_start_group_RobotZ->str_length = 6;
    strcpy(ui_start_start_group_RobotZ->string, "ROBOTZ");


    ui_proc_string_frame(&ui_start_start_group_0);
    SEND_MESSAGE((uint8_t *) &ui_start_start_group_0, sizeof(ui_start_start_group_0));
}

void _ui_update_start_start_group_0() {
    ui_start_start_group_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_start_start_group_0);
    SEND_MESSAGE((uint8_t *) &ui_start_start_group_0, sizeof(ui_start_start_group_0));
}

void _ui_remove_start_start_group_0() {
    ui_start_start_group_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_start_start_group_0);
    SEND_MESSAGE((uint8_t *) &ui_start_start_group_0, sizeof(ui_start_start_group_0));
}

void ui_init_start_start_group() {
    _ui_init_start_start_group_0();
}

void ui_update_start_start_group() {
    _ui_update_start_start_group_0();
}

void ui_remove_start_start_group() {
    _ui_remove_start_start_group_0();
}

