#ifndef __CONTROL_H
#define __CONTROL_H

#include "user.h"

#define Frequency	100.0f			//每10ms读取一次编码器的值
#define Perimeter	0.2104867	    //轮子周长(单位:m)=0.67*3.1415926
#define Wheelspacing 0.1610f		//主动轮轴距(单位:m)
#define PI 3.1415926

// 速度低通滤波系数 (0~1, 越小滤波越强, 0.3~0.5推荐)
#define SPEED_FILTER_ALPHA  0.4f
// PI死区阈值 (m/s), 偏差小于此值不调整PWM, 避免量化噪声振荡
#define PI_DEADBAND         0.005f
// PWM输出限幅 (周期8000, 7800=97.5%占空比, 留余量避免完全饱和)
#define PWM_MAX             7800

//更换电机需修改此处参数
#define ENCODER_LINES  	13       // 编码器线数 (每转13个脉冲)
#define MULTIPLY_FACTOR 2      // 2倍频系数 (只检测上升沿)
#define GEAR_RATIO   	28         // 减速比 28:1 MG513X 20:1 MG310
	
//编码器结构体
typedef struct  
{
  int A;      
  int B;  
}Encoder;


//电机速度控制相关参数结构体
typedef struct  
{
	float Current_Encoder;     	//编码器数值，读取电机实时速度
	float Motor_Pwm;     		//电机PWM数值，控制电机实时速度
	float Target_Encoder;  		//电机目标编码器速度值，控制电机目标速度
	float Velocity; 	 		//电机速度值
	
}Motor_parameter;

typedef struct
{
	float Kp;
	float ki;
//	float Kd;
	
}PID_Parameter;


extern Encoder OriginalEncoder; 					//编码器原始数据   
extern Motor_parameter MotorA,MotorB;				//左右电机相关变量
extern PID_Parameter V_PID;					//PID 参数
extern float Velocity_KP,Velocity_KI;	

void Get_Velocity_From_Encoder(int Encoder1,int Encoder2);

int myabs(int a);

int Incremental_PI_Left (float Encoder,float Target);
int Incremental_PI_Right (float Encoder,float Target);

#endif
