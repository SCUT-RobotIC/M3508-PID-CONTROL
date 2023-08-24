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
		
		//  速度环 DB 20
		//  角度环 DB 500
		
		// 先只管P
  }
}


void ang_init(float p , float i , float d ){
	for(int di=0; di<4; di++)	
  {
      pid_init(&ang_drive_motor_pid[di]);
      ang_drive_motor_pid[di].f_param_init(&ang_drive_motor_pid[di], PID_Speed, 16384,2000, 500, 0, 8000, 2000,  p ,i, d);//10,0.02,0	
			
			motor_data[di]->circle = 0;
		// 先只管P
  }
}

// ignore chassis_vector_to_mecanum_wheel_speed


void spd_control_loop(float wheel_speed[4])  
{
	
		spd_drive_motor_pid[0].target = wheel_speed[0];     
		spd_drive_motor_pid[1].target = wheel_speed[1]; // 
		spd_drive_motor_pid[2].target = wheel_speed[2];  
		spd_drive_motor_pid[3].target = wheel_speed[3];
	
		spd_drive_motor_pid[0].f_cal_pid(&spd_drive_motor_pid[0],motor_data[0]->speed_rpm,0);   //1号电机的pid电流计算值
		spd_drive_motor_pid[1].f_cal_pid(&spd_drive_motor_pid[1],motor_data[1]->speed_rpm,0);   //2号电机的pid电流计算值
		spd_drive_motor_pid[2].f_cal_pid(&spd_drive_motor_pid[2],motor_data[2]->speed_rpm,0);   //3号电机的pid电流计算值
		spd_drive_motor_pid[3].f_cal_pid(&spd_drive_motor_pid[3],motor_data[3]->speed_rpm,0);   //4号电机的pid电流计算值
		
		/// 将电流值发送给电机
		CAN_cmd_chassis(spd_drive_motor_pid[0].output,spd_drive_motor_pid[1].output,spd_drive_motor_pid[2].output,spd_drive_motor_pid[3].output);					
}	

void motor_state_update(){
	
	motor_data[0] = get_chassis_motor_measure_point(0);//获取ID为1号的电机数据指针
	motor_data[1] = get_chassis_motor_measure_point(1);//获取ID为2号的电机数据指针
	motor_data[2] = get_chassis_motor_measure_point(2);//获取ID为3号的电机数据指针
	motor_data[3] = get_chassis_motor_measure_point(3);//获取ID为4号的电机数据指针
}

void ang_control_loop(float wheel_ang[4],float wheel_ang_spd[4],float k)  
{
	
		ang_drive_motor_pid[0].target = wheel_ang[0];     
		ang_drive_motor_pid[1].target = wheel_ang[1]; // 
		ang_drive_motor_pid[2].target = wheel_ang[2];  
		ang_drive_motor_pid[3].target = wheel_ang[3];
		
	
		
		ang_drive_motor_pid[0].f_cal_pid(&ang_drive_motor_pid[0],motor_data[0]->ecd+ motor_data[0]->circle * 8191,1);   //1号电机的pid电流计算值
		ang_drive_motor_pid[1].f_cal_pid(&ang_drive_motor_pid[1],motor_data[1]->ecd+motor_data[1]->circle * 8191,1);   //2号电机的pid电流计算值
		ang_drive_motor_pid[2].f_cal_pid(&ang_drive_motor_pid[2],motor_data[2]->ecd+motor_data[2]->circle * 8191,1);   //3号电机的pid电流计算值
		ang_drive_motor_pid[3].f_cal_pid(&ang_drive_motor_pid[3],motor_data[3]->ecd+motor_data[3]->circle * 8191,1);   //4号电机的pid电流计算值
		
		
		wheel_ang_spd[0] = ang_drive_motor_pid[0].output * k;
		wheel_ang_spd[1] = ang_drive_motor_pid[1].output * k;
		wheel_ang_spd[2] = ang_drive_motor_pid[2].output * k;
		wheel_ang_spd[3] = ang_drive_motor_pid[3].output * k;	
}	
	
