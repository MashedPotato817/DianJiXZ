/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
uint16_t Gray_Data[8];
uint16_t Gray_Raw[8];
float Gray_Line_Pos_mm;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车启动标志位

#define GRAY_BLACK_LEVEL              1
#define GRAY_BASE_SPEED_MM_S          300.0f
#define GRAY_MID_SPEED_MM_S           240.0f
#define GRAY_CURVE_SPEED_MM_S         185.0f
#define GRAY_CROSS_SPEED_MM_S         310.0f
#define GRAY_LOST_SEARCH_SPEED_MM_S   0.0f
#define GRAY_SENSOR_PITCH_MM          10.0f
#define GRAY_SENSOR_FORWARD_MM        150.0f
#define GRAY_CAR_WHEELBASE_MM         105.0f
#define GRAY_STEER_GAIN               1.70f
#define GRAY_MAX_ANGULAR_SPEED        3.60f
#define GRAY_LOST_TURN_SPEED          1.80f
#define GRAY_LOST_POS_DEADBAND_MM     3.0f
#define GRAY_MID_POS_MM               10.0f
#define GRAY_CURVE_POS_MM             22.0f
#define GRAY_CROSS_BLACK_COUNT        7
#define GRAY_CROSS_DETECT_COUNT       2
#define GRAY_CROSS_RUN_TICKS          32
#define GRAY_CROSS_COOLDOWN_TICKS     60

static const float Gray_Pos_mm[8] = {
    -3.5f * GRAY_SENSOR_PITCH_MM,
    -2.5f * GRAY_SENSOR_PITCH_MM,
    -1.5f * GRAY_SENSOR_PITCH_MM,
    -0.5f * GRAY_SENSOR_PITCH_MM,
     0.5f * GRAY_SENSOR_PITCH_MM,
     1.5f * GRAY_SENSOR_PITCH_MM,
     2.5f * GRAY_SENSOR_PITCH_MM,
     3.5f * GRAY_SENSOR_PITCH_MM
};

static void Gray_Select_Channel(uint8_t channel)
{
    if (channel & 0x01) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    else                DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);

    if (channel & 0x02) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);

    if (channel & 0x04) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
}

static int Gray_ToBlack(uint32_t pin_state)
{
    int level = pin_state ? 1 : 0;
    return (level == GRAY_BLACK_LEVEL) ? 1 : 0;
}

void Gray_Read_All(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        Gray_Select_Channel(i);
        delay_us(50);
        Gray_Raw[i] = DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) ? 1 : 0;
        Gray_Data[i] = Gray_ToBlack(Gray_Raw[i]);
    }
}

void Gray_Mode(void)
{
    static float last_line_pos_mm = 0;
    static int8_t last_search_dir = 1;
    static uint8_t cross_detect_count = 0;
    static uint8_t cross_run_ticks = 0;
    static uint8_t cross_cooldown_ticks = 0;
    float pos_sum = 0;
    float abs_line_pos_mm;
    int black_count = 0;
    float y_m;
    float lookahead_m;
    float curvature;
    uint8_t i;

    Gray_Read_All();
    for (i = 0; i < 8; i++) {
        if (Gray_Data[i]) {
            pos_sum += Gray_Pos_mm[i];
            black_count++;
        }
    }

    if (cross_cooldown_ticks > 0) {
        cross_cooldown_ticks--;
    }

    if (cross_run_ticks > 0) {
        cross_run_ticks--;
        Move_X = GRAY_CROSS_SPEED_MM_S / 1000.0f;
        Move_Z = 0;
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    if (black_count >= GRAY_CROSS_BLACK_COUNT) {
        if (cross_detect_count < GRAY_CROSS_DETECT_COUNT) {
            cross_detect_count++;
        }
        if ((cross_detect_count >= GRAY_CROSS_DETECT_COUNT) && (cross_cooldown_ticks == 0)) {
            cross_detect_count = 0;
            cross_run_ticks = GRAY_CROSS_RUN_TICKS;
            cross_cooldown_ticks = GRAY_CROSS_COOLDOWN_TICKS;
            Move_X = GRAY_CROSS_SPEED_MM_S / 1000.0f;
            Move_Z = 0;
            Get_Target_Encoder(Move_X, Move_Z);
            return;
        }
    } else {
        cross_detect_count = 0;
    }

    Move_X = GRAY_BASE_SPEED_MM_S / 1000.0f;
    if (black_count == 0) {
        Gray_Line_Pos_mm = last_line_pos_mm;
        Move_X = GRAY_LOST_SEARCH_SPEED_MM_S / 1000.0f;
        Move_Z = last_search_dir * GRAY_LOST_TURN_SPEED;
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    Gray_Line_Pos_mm = pos_sum / black_count;
    last_line_pos_mm = Gray_Line_Pos_mm;

    if (Gray_Line_Pos_mm > GRAY_LOST_POS_DEADBAND_MM) {
        last_search_dir = -1;
    } else if (Gray_Line_Pos_mm < -GRAY_LOST_POS_DEADBAND_MM) {
        last_search_dir = 1;
    }

    abs_line_pos_mm = (Gray_Line_Pos_mm >= 0) ? Gray_Line_Pos_mm : -Gray_Line_Pos_mm;
    if ((black_count >= 5) || (abs_line_pos_mm >= GRAY_CURVE_POS_MM)) {
        Move_X = GRAY_CURVE_SPEED_MM_S / 1000.0f;
    } else if (abs_line_pos_mm >= GRAY_MID_POS_MM) {
        Move_X = GRAY_MID_SPEED_MM_S / 1000.0f;
    }

    y_m = Gray_Line_Pos_mm / 1000.0f;
    lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;
    curvature = (2.0f * y_m) / (lookahead_m * lookahead_m + y_m * y_m);
    Move_Z = -GRAY_STEER_GAIN * Move_X * curvature;

    if (Move_Z > GRAY_MAX_ANGULAR_SPEED) Move_Z = GRAY_MAX_ANGULAR_SPEED;
    if (Move_Z < -GRAY_MAX_ANGULAR_SPEED) Move_Z = -GRAY_MAX_ANGULAR_SPEED;

    Get_Target_Encoder(Move_X, Move_Z);
}
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			
			Key();
			LED_Flash(100);
			Get_Velocity_From_Encoder(-Get_Encoder_countA,-Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(Run_Mode==0)
			{
				Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			}else if(Run_Mode==1){
				Gray_Mode();//8路灰度巡线
			}
//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			{
				Set_PWM(MotorA.Motor_Pwm,-MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
		}
    }
}

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
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	MotorA.Current_Encoder= -Encoder_A_pr*Frequency*Perimeter/780.0f;  
	MotorB.Current_Encoder= -Encoder_B_pr*Frequency*Perimeter/780.0f;   //1560=2*13*30=2（两路脉冲）*1（上升沿计数）*霍尔编码器13线*电机的减速比
	
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
	//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//	 MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	 MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
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

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
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
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
//	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
//	{
//	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
//	 { 
//			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
//			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
//			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
//			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
//			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
//			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
//			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
//			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
//			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
//	 }
//	 if(Flag_Move==0)		
//	 {	
//		 //If no direction control instruction is available, check the steering control status
//		 //如果无方向控制指令，检查转向控制状态
//		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
//		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
//		 else 		               Move_Z=0;                       //stop           //停止
//	 }
//	}	
//	else //Non-omnidirectional moving trolley //非全向移动小车
//	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
//	}
	
//	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=key_scan(200);//click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//单击控制小车的启停
	else if(tmp==2)
	{
		Run_Mode++;
		Run_Mode%=2;
	}
}
