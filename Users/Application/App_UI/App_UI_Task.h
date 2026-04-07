#ifndef INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_
#define INFANTRY_ROBOT_USERS_APPLICATION_APP_UI_APP_UI_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

void UI_Task(void *argument);

// 重新初始化UI任务的函数
void UI_Task_Reset(void);

#ifdef __cplusplus
}
#endif

#endif
