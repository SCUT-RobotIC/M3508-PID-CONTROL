
#ifndef PID_H
#define PID_H
#include <stdint.h> 

#define NULL 0
///////////////////////////////////////////////
typedef enum
{

	PID_Position,
	PID_Speed     // ����ʽ
	
}PID_ID;



typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target ;
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;

	float   measure;					//����ֵ
	float   err;							//���
	float   last_err;      		//�ϴ����
	
	float pout;
	float iout;
	float dout;
	
	float output;						//�������
	float last_output;			//�ϴ����
	
	float MaxOutput;				//����޷�
	float IntegralLimit;		//�����޷�
	float DeadBand;			    //����������ֵ��
	float ControlPeriod;		//��������
	float  Max_Err;					//������
	
	uint32_t thistime;
	uint32_t lasttime;
  uint8_t  dtime;	
	

	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID������ʼ��
				   PID_ID id,
				   uint32_t  maxOutput,
				   uint32_t integralLimit,
				   float    deadband,
				   uint16_t controlPeriod,
					 int16_t  max_err,     
					 int16_t  target,
				   float kp,
				   float ki,
				   float kd );
				   
void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid���������޸�
					 
float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure,int a);   //�����ٶ�ʵ��ֵ��������Ƶ���ֵ


}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
static float pid_calculate(PID_TypeDef* pid, float measure,int a);		

///////////////////////////////////////////////
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    
	//PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
		
		

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
