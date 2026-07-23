#include "control.h"


Encoder OriginalEncoder; 					//编码器原始数据 
Motor_parameter MotorA,MotorB;				//左右电机相关变量

float Velocity_KP=400,Velocity_KI=300;	

PID_Parameter V_PID={400.0f,300.0f};			//PID 参数

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	static float Filtered_SpeedA = 0.0f, Filtered_SpeedB = 0.0f;
	float Encoder_A_pr,Encoder_B_pr;
	float raw_speedA, raw_speedB;
	
	OriginalEncoder.A=Encoder1;	
	OriginalEncoder.B=Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; 
	Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	raw_speedA = Encoder_A_pr*Frequency*Perimeter/728.0f;
	raw_speedB = Encoder_B_pr*Frequency*Perimeter/728.0f;
	
	//一阶低通滤波: 平滑编码器量化噪声
	Filtered_SpeedA = SPEED_FILTER_ALPHA * raw_speedA + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedA;
	Filtered_SpeedB = SPEED_FILTER_ALPHA * raw_speedB + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedB;
	
	MotorA.Current_Encoder = Filtered_SpeedA;
	MotorB.Current_Encoder = Filtered_SpeedB;
}

/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 float abs_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 
	 //死区: 偏差很小时跳过PI更新, 避免编码器量化噪声引起PWM振荡
	 abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	 if(abs_bias < PI_DEADBAND)
	 {
		 Last_bias = Bias;  //仍然跟踪偏差, 退出死区时平滑恢复
		 return (int)Pwm;
	 }
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 Pwm = PWM_Limit(Pwm,PWM_MAX,-PWM_MAX);
	 return (int)Pwm;                          				//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 float abs_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 
	 //死区: 偏差很小时跳过PI更新, 避免编码器量化噪声引起PWM振荡
	 abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	 if(abs_bias < PI_DEADBAND)
	 {
		 Last_bias = Bias;  //仍然跟踪偏差, 退出死区时平滑恢复
		 return (int)Pwm;
	 }
	 
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 Pwm = PWM_Limit(Pwm,PWM_MAX,-PWM_MAX);
	 return (int)Pwm;                          				//增量输出
}
