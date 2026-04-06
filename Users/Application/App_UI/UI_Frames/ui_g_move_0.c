//
// Created by RM UI Designer
//

#include "ui_g_move_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_move_0;
ui_interface_round_t *ui_g_move_center_round = (ui_interface_round_t *)&(ui_g_move_0.data[0]);
ui_interface_number_t *ui_g_move_distance = (ui_interface_number_t *)&(ui_g_move_0.data[1]);
ui_interface_line_t *ui_g_move_cap = (ui_interface_line_t *)&(ui_g_move_0.data[2]);
ui_interface_number_t *ui_g_move_gyroton = (ui_interface_number_t *)&(ui_g_move_0.data[3]);
ui_interface_number_t *ui_g_move_fricton = (ui_interface_number_t *)&(ui_g_move_0.data[4]);

void _ui_init_g_move_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_move_0.data[i].figure_name[0] = FRAME_ID;
        ui_g_move_0.data[i].figure_name[1] = GROUP_ID;
        ui_g_move_0.data[i].figure_name[2] = i + START_ID;
        ui_g_move_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_move_0.data[i].operate_tpyel = 0;
    }

    ui_g_move_center_round->figure_tpye = 2;
    ui_g_move_center_round->layer = 0;
    ui_g_move_center_round->r = 354;
    ui_g_move_center_round->start_x = 960;
    ui_g_move_center_round->start_y = 540;
    ui_g_move_center_round->color = 3;
    ui_g_move_center_round->width = 1;

    ui_g_move_distance->figure_tpye = 5;
    ui_g_move_distance->layer = 0;
    ui_g_move_distance->font_size = 20;
    ui_g_move_distance->start_x = 1351;
    ui_g_move_distance->start_y = 638;
    ui_g_move_distance->color = 6;
    ui_g_move_distance->number = 12345;
    ui_g_move_distance->width = 2;

    ui_g_move_cap->figure_tpye = 0;
    ui_g_move_cap->layer = 0;
    ui_g_move_cap->start_x = 163;
    ui_g_move_cap->start_y = 183;
    ui_g_move_cap->end_x = 704;
    ui_g_move_cap->end_y = 183;
    ui_g_move_cap->color = 0;
    ui_g_move_cap->width = 10;

    ui_g_move_gyroton->figure_tpye = 6;
    ui_g_move_gyroton->layer = 0;
    ui_g_move_gyroton->font_size = 20;
    ui_g_move_gyroton->start_x = 107;
    ui_g_move_gyroton->start_y = 897;
    ui_g_move_gyroton->color = 0;
    ui_g_move_gyroton->number = 0;
    ui_g_move_gyroton->width = 2;

    ui_g_move_fricton->figure_tpye = 6;
    ui_g_move_fricton->layer = 0;
    ui_g_move_fricton->font_size = 20;
    ui_g_move_fricton->start_x = 107;
    ui_g_move_fricton->start_y = 852;
    ui_g_move_fricton->color = 0;
    ui_g_move_fricton->number = 0;
    ui_g_move_fricton->width = 2;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_move_0);
    SEND_MESSAGE((uint8_t *) &ui_g_move_0, sizeof(ui_g_move_0));
}

void _ui_update_g_move_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_move_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_move_0);
    SEND_MESSAGE((uint8_t *) &ui_g_move_0, sizeof(ui_g_move_0));
}

void _ui_remove_g_move_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_move_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_move_0);
    SEND_MESSAGE((uint8_t *) &ui_g_move_0, sizeof(ui_g_move_0));
}
