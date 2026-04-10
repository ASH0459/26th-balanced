#ifndef INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_
#define INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ui_normal.h"

#define RED_BLUE_COLOR 0
#define YELLOW_COLOR 1
#define GREEN_COLOR 2
#define ORANGE_COLOR 3
#define PURPLE_COLOR 4
#define PINK_COLOR 5
#define CYAN_COLOR 6
#define BLACK_COLOR 7
#define WHITE_COLOR 8

#define CAP_ENERGY_LOW_THRESHOLD 20.0f
#define CAP_ENERGY_MEDIUM_THRESHOLD 60.0f

#ifdef __cplusplus
}
#endif

static inline uint32_t UI_SuperCap_Bar_Start_X(void)
{
    return ui_normal_DynamicGroup1_SuperCapPercentage->start_x;
}

static inline uint32_t UI_SuperCap_Bar_Start_Y(void)
{
    return ui_normal_DynamicGroup1_SuperCapPercentage->start_y;
}

static inline uint32_t UI_SuperCap_Bar_End_X(void)
{
    return ui_normal_StaticGroup1_SuperCapRect->end_x;
}

#ifdef __cplusplus
extern "C" {
#endif

void UI_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
