#ifndef INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H
#define INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H

#include "App_Chassis_Task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void Chassis_Behaviour_Mode_Set(Chassis_Move *chassis_move_mode);
    void chassis_behaviour_control_set(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                       Chassis_Move *chassis_move_rc_to_vector);

#ifdef __cplusplus
}
#endif

#endif // INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H
