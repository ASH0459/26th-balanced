#ifndef INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_
#define INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RED_BLUE_COLOR 0 /* 红蓝主色: 实际显示跟随己方阵营自动切换为红色或蓝色 */
#define YELLOW_COLOR 1   /* 黄色 */
#define GREEN_COLOR 2    /* 绿色 */
#define ORANGE_COLOR 3   /* 橙色 */
#define PURPLE_COLOR 4   /* 紫色 */
#define PINK_COLOR 5     /* 粉色 */
#define CYAN_COLOR 6     /* 青色 */
#define BLACK_COLOR 7    /* 黑色 */
#define WHITE_COLOR 8    /* 白色 */

void UI_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
