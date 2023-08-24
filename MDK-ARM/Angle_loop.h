#ifndef __ANGLE_LOOP_H
#define __ANGLE_LOOP_H


#include "pid.h"

#include "bsp_can.h"

extern PID_TypeDef    spd_drive_motor_pid[4];
extern PID_TypeDef    ang_drive_motor_pid[4];


void ang_init(float p , float i , float d );
void spd_init(float p , float i , float d );

void ang_control_loop(float wheel_ang[4],float wheel_ang_spd[4],float k) ;
void spd_control_loop(float wheel_speed[4]);

void motor_state_update();



#endif