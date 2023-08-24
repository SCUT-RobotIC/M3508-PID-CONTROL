#include "pid.h"
#include "bsp_can.h"
#include "stm32f4xx_hal.h"


PID_TypeDef    spd_drive_motor_pid[4];
PID_TypeDef    ang_drive_motor_pid[4];

extern  motor_measure_t   *motor_data[4];


void spd_init(float p , float i , float d ){
	for(int di=0; di<4; di++)	
  {
      pid_init(&spd_drive_motor_pid[di]);
      spd_drive_motor_pid[di].f_param_init(&spd_drive_motor_pid[di], PID_Speed, 16384, 2000, 20, 0, 8000, 0,  p ,i, d);//10,0.02,0	
		
		//  �ٶȻ� DB 20
		//  �ǶȻ� DB 500
		
		// ��ֻ��P
  }
}


void ang_init(float p , float i , float d ){
	for(int di=0; di<4; di++)	
  {
      pid_init(&ang_drive_motor_pid[di]);
      ang_drive_motor_pid[di].f_param_init(&ang_drive_motor_pid[di], PID_Speed, 16384,2000, 500, 0, 8000, 2000,  p ,i, d);//10,0.02,0	
			
			motor_data[di]->circle = 0;
		// ��ֻ��P
  }
}

// ignore chassis_vector_to_mecanum_wheel_speed


void spd_control_loop(float wheel_speed[4])  
{
	
		spd_drive_motor_pid[0].target = wheel_speed[0];     
		spd_drive_motor_pid[1].target = wheel_speed[1]; // 
		spd_drive_motor_pid[2].target = wheel_speed[2];  
		spd_drive_motor_pid[3].target = wheel_speed[3];
	
		spd_drive_motor_pid[0].f_cal_pid(&spd_drive_motor_pid[0],motor_data[0]->speed_rpm,0);   //1�ŵ����pid��������ֵ
		spd_drive_motor_pid[1].f_cal_pid(&spd_drive_motor_pid[1],motor_data[1]->speed_rpm,0);   //2�ŵ����pid��������ֵ
		spd_drive_motor_pid[2].f_cal_pid(&spd_drive_motor_pid[2],motor_data[2]->speed_rpm,0);   //3�ŵ����pid��������ֵ
		spd_drive_motor_pid[3].f_cal_pid(&spd_drive_motor_pid[3],motor_data[3]->speed_rpm,0);   //4�ŵ����pid��������ֵ
		
		/// ������ֵ���͸����
		CAN_cmd_chassis(spd_drive_motor_pid[0].output,spd_drive_motor_pid[1].output,spd_drive_motor_pid[2].output,spd_drive_motor_pid[3].output);					
}	

void motor_state_update(){
	
	motor_data[0] = get_chassis_motor_measure_point(0);//��ȡIDΪ1�ŵĵ������ָ��
	motor_data[1] = get_chassis_motor_measure_point(1);//��ȡIDΪ2�ŵĵ������ָ��
	motor_data[2] = get_chassis_motor_measure_point(2);//��ȡIDΪ3�ŵĵ������ָ��
	motor_data[3] = get_chassis_motor_measure_point(3);//��ȡIDΪ4�ŵĵ������ָ��
}

void ang_control_loop(float wheel_ang[4],float wheel_ang_spd[4],float k)  
{
	
		ang_drive_motor_pid[0].target = wheel_ang[0];     
		ang_drive_motor_pid[1].target = wheel_ang[1]; // 
		ang_drive_motor_pid[2].target = wheel_ang[2];  
		ang_drive_motor_pid[3].target = wheel_ang[3];
		
	
		
		ang_drive_motor_pid[0].f_cal_pid(&ang_drive_motor_pid[0],motor_data[0]->ecd+ motor_data[0]->circle * 8191,1);   //1�ŵ����pid��������ֵ
		ang_drive_motor_pid[1].f_cal_pid(&ang_drive_motor_pid[1],motor_data[1]->ecd+motor_data[1]->circle * 8191,1);   //2�ŵ����pid��������ֵ
		ang_drive_motor_pid[2].f_cal_pid(&ang_drive_motor_pid[2],motor_data[2]->ecd+motor_data[2]->circle * 8191,1);   //3�ŵ����pid��������ֵ
		ang_drive_motor_pid[3].f_cal_pid(&ang_drive_motor_pid[3],motor_data[3]->ecd+motor_data[3]->circle * 8191,1);   //4�ŵ����pid��������ֵ
		
		
		wheel_ang_spd[0] = ang_drive_motor_pid[0].output * k;
		wheel_ang_spd[1] = ang_drive_motor_pid[1].output * k;
		wheel_ang_spd[2] = ang_drive_motor_pid[2].output * k;
		wheel_ang_spd[3] = ang_drive_motor_pid[3].output * k;	
}	
	
