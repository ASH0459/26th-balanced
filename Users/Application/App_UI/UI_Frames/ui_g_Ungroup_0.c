//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_0.h"

#define FRAME_ID 0
#define GROUP_ID 2
#define START_ID 0
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_Ungroup_0;
ui_interface_line_t *ui_g_Ungroup_five = (ui_interface_line_t *)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_three = (ui_interface_line_t *)&(ui_g_Ungroup_0.data[1]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = FRAME_ID;
        ui_g_Ungroup_0.data[i].figure_name[1] = GROUP_ID;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + START_ID;
        ui_g_Ungroup_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 0;
    }

    ui_g_Ungroup_five->figure_tpye = 0;
    ui_g_Ungroup_five->layer = 0;
    ui_g_Ungroup_five->start_x = 941;
    ui_g_Ungroup_five->start_y = 484;
    ui_g_Ungroup_five->end_x = 971;
    ui_g_Ungroup_five->end_y = 484;
    ui_g_Ungroup_five->color = 4;
    ui_g_Ungroup_five->width = 2;

    ui_g_Ungroup_three->figure_tpye = 0;
    ui_g_Ungroup_three->layer = 0;
    ui_g_Ungroup_three->start_x = 941;
    ui_g_Ungroup_three->start_y = 460;
    ui_g_Ungroup_three->end_x = 971;
    ui_g_Ungroup_three->end_y = 460;
    ui_g_Ungroup_three->color = 4;
    ui_g_Ungroup_three->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}
