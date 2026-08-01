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
#include "k230_link.h"
#include "ball_control.h"
#include "ball_calibrate.h"
#include "ball_task.h"
#include "debug_telemetry.h"
#include "calib_store.h"
#include <string.h>

u8 ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
uint16_t Gray_Data[8];
uint16_t Gray_Raw[8];
float Gray_Line_Pos_mm;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车启动标志位

/* 本题 UART1 专用于 K230，未编译蓝牙回调模块时保持底盘遥控量为静止。 */
int Flag_Left = 0;
int Flag_Right = 0;
int Flag_Direction = 0;

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
    if (channel & 0x01) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_AD0_PIN);
    else                DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_AD0_PIN);

    if (channel & 0x02) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_AD1_PIN);
    else                DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_AD1_PIN);

    if (channel & 0x04) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_AD2_PIN);
    else                DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_AD2_PIN);
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
        Gray_Raw[i] = DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_OUT_PIN) ? 1 : 0;
        Gray_Data[i] = Gray_ToBlack(Gray_Raw[i]);
    }
}

void Gray_Mode(void)
{
    static float lost_search_angle;
    static float last_search_move_z;
    static uint8_t line_seen;
    float pos_sum = 0;
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

    if (black_count == 0) {
        Gray_Line_Pos_mm = 0;
        /* 首次上电未识别到黑线、无有效搜线方向或已搜满一圈时停车 */
        if ((!line_seen) || (last_search_move_z == 0.0f) ||
            (lost_search_angle >= GRAY_LOST_SEARCH_MAX_ANGLE_RAD)) {
            Move_X = 0;
            Move_Z = 0;
        } else {
            /* 丢线后停止前进，沿最后一次有效偏线方向低速原地搜线 */
            Move_X = 0;
            Move_Z = last_search_move_z;
            lost_search_angle += GRAY_LOST_SEARCH_ANGULAR_SPEED / Frequency;
        }
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    line_seen = 1;
    lost_search_angle = 0;
    Gray_Line_Pos_mm = pos_sum / black_count;
    Move_X = GRAY_BASE_SPEED_MM_S / 1000.0f;

    y_m = Gray_Line_Pos_mm / 1000.0f;
    lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;
    curvature = (2.0f * y_m) / (lookahead_m * lookahead_m + y_m * y_m);
    Move_Z = -GRAY_STEER_GAIN * Move_X * curvature;

    if (Move_Z > GRAY_MAX_ANGULAR_SPEED) Move_Z = GRAY_MAX_ANGULAR_SPEED;
    if (Move_Z < -GRAY_MAX_ANGULAR_SPEED) Move_Z = -GRAY_MAX_ANGULAR_SPEED;

    /* 仅在存在偏线时更新搜线方向，居中直线不覆盖最近一次转向方向 */
    if (Move_Z > 0.0f) last_search_move_z = GRAY_LOST_SEARCH_ANGULAR_SPEED;
    else if (Move_Z < 0.0f) last_search_move_z = -GRAY_LOST_SEARCH_ANGULAR_SPEED;

    Get_Target_Encoder(Move_X, Move_Z);
}
void TIMER_0_INST_IRQHandler(void)
{
    K230_BallPosition ball_position;

    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMERG_IIDX_ZERO)
    {
			K230_Link_Tick5ms();
			Ball_Calibrate_Tick5ms();
			/* 标定状态变化输出事件：长按后应看到 START→DONE→主循环 SAVED。 */
			{
				static Ball_CalibrateState last_cal_state = BALL_CAL_STATE_IDLE;
				Ball_CalibrateState now_cal_state = Ball_Calibrate_GetState();
				if (now_cal_state != last_cal_state) {
					if (now_cal_state == BALL_CAL_STATE_WAIT_BALL) {
						Debug_Telemetry_LogEvent("CAL,START");
					} else if (now_cal_state == BALL_CAL_STATE_DONE) {
						Debug_Telemetry_LogEvent("CAL,DONE");
					} else if ((now_cal_state == BALL_CAL_STATE_IDLE) &&
						   (last_cal_state != BALL_CAL_STATE_IDLE)) {
						Debug_Telemetry_LogEvent("CAL,ABORT");
					}
					last_cal_state = now_cal_state;
				}
			}
			/* 任务状态变化输出事件行，便于日志定位任务推进时刻。 */
			{
				static Ball_TaskState last_task_state = BALL_TASK_IDLE;
				Ball_TaskState now_task_state = Ball_Task_GetState();
				if (now_task_state != last_task_state) {
					switch (now_task_state) {
					case BALL_TASK_POINT_PLUS:
						Debug_Telemetry_LogEvent("TASK,PLUS");
						break;
					case BALL_TASK_POINT_MINUS:
						Debug_Telemetry_LogEvent("TASK,MINUS");
						break;
					case BALL_TASK_POINT_DONE:
						Debug_Telemetry_LogEvent("TASK,DONE");
						break;
					case BALL_TASK_FAULT:
						Debug_Telemetry_LogEvent("TASK,FAULT");
						break;
					default:
						break;
					}
					last_task_state = now_task_state;
				}
			}
			K230_Link_GetPosition(&ball_position);
			/*
			 * 始终推进控制时钟供 OLED 刷新；自动标定激活时 Step 内部会因
			 * servo_hold 立即返回，不会覆盖标定状态机直接写入的舵机角度。
			 */
			Ball_Control_Step(&ball_position, 0.005f);
			/* 任务验收使用本周期刚更新的位置速度，避免读取上一周期状态。 */
			Ball_Task_Tick5ms();

			Key();
			/* 联调：慢闪=未发，2 Hz=已发无字节，0.5 Hz=收到原始字节，常亮=已解帧。 */
			if (K230_Link_HasValidFrame() != 0U) LED_ON();
			else if (K230_Link_IsLocalLoopbackDetected() != 0U) LED_OFF();
			else if (K230_Link_HasRxBytes() != 0U) LED_Flash(200);
			else if (K230_Link_HasTxAttempt() != 0U) LED_Flash(50);
			else                                LED_Flash(100);
			Debug_Telemetry_Tick5ms(Get_Encoder_countA, Get_Encoder_countB);
			Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
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
				Set_PWM(-MotorA.Motor_Pwm,-MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
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
	static float Filtered_SpeedA = 0.0f, Filtered_SpeedB = 0.0f;
	float Encoder_A_pr, Encoder_B_pr, raw_speedA, raw_speedB;
	OriginalEncoder.A = Encoder1;
	OriginalEncoder.B = Encoder2;
	Encoder_A_pr = OriginalEncoder.A;
	Encoder_B_pr = -OriginalEncoder.B;
	raw_speedA =  Encoder_A_pr * Frequency * Perimeter / CPR;
	raw_speedB =  Encoder_B_pr * Frequency * Perimeter / CPR;

	Filtered_SpeedA = SPEED_FILTER_ALPHA * raw_speedA + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedA;
	Filtered_SpeedB = SPEED_FILTER_ALPHA * raw_speedB + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedB;

	MotorA.Current_Encoder = Filtered_SpeedA;
	MotorB.Current_Encoder = Filtered_SpeedB;
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
	 float abs_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	 if(abs_bias < PI_DEADBAND) { Last_bias = Bias; return (int)Pwm; }
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
	 Last_bias=Bias;	                   					//保存上一次偏差
	 return (int)Pwm;                         				//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{
	 static float Bias,Pwm,Last_bias;
	 float abs_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 abs_bias = (Bias > 0.0f) ? Bias : -Bias;
	 if(abs_bias < PI_DEADBAND) { Last_bias = Bias; return (int)Pwm; }
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
	 Last_bias=Bias;	                   					//保存上一次偏差
	 return (int)Pwm;                         				//增量输出
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
		if (Ball_Calibrate_IsActive() != 0U) {
			Debug_Telemetry_LogEvent("KEY,SINGLE_IGN");
		} else {
			Ball_Task_StartPoint();	// START(PA18)单击：定点运动任务（0→+5→-5）
			Debug_Telemetry_LogEvent("KEY,single");
		}
	}
	else if(tmp==2)
	{
		Run_Mode++;
		Run_Mode%=2;
		Debug_Telemetry_LogEvent("KEY,double");
	}
	else if(tmp==3)
	{
		Ball_TaskState task_state = Ball_Task_GetState();
		if ((task_state == BALL_TASK_POINT_PLUS) ||
		    (task_state == BALL_TASK_POINT_MINUS)) {
			Debug_Telemetry_LogEvent("KEY,LONG_IGN");
		} else {
			Ball_Calibrate_Start();	// START(PA18)长按：自动扫掠标定并保存
			Debug_Telemetry_LogEvent("KEY,long");
		}
	}
}

/*
 * UART0 命令接口（主循环轮询）：接收 $SET,<name>,<value># 运行时调参。
 * 支持 KP/KD/TRIM 设置、$CAL 触发标定。修改在关中断下执行，避免与 5ms ISR 竞争。
 * 例：$SET,KP,0.30#  $SET,KD,0.05#  $SET,TRIM,137.5#  $CAL#
 */
#define UART0_CMD_BUF_LEN 32U
static char uart0_cmd_buf[UART0_CMD_BUF_LEN];
static uint8_t uart0_cmd_len = 0U;

static float UART0_Command_ParseFloat(const char *s)
{
    float sign = 1.0f;
    float val = 0.0f;
    float frac = 0.1f;
    uint8_t in_frac = 0U;

    if (*s == '-') {
        sign = -1.0f;
        s++;
    }
    while (*s != '\0') {
        if ((*s >= '0') && (*s <= '9')) {
            if (in_frac != 0U) {
                val += frac * (float)(*s - '0');
                frac *= 0.1f;
            } else {
                val = val * 10.0f + (float)(*s - '0');
            }
        } else if (*s == '.') {
            in_frac = 1U;
        }
        s++;
    }
    return sign * val;
}

void UART0_Command_Poll(void)
{
    uint32_t primask;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        char ch = (char)DL_UART_Main_receiveData(UART_0_INST);
        if (ch == '#') {
            if (uart0_cmd_len > 0U) {
                uart0_cmd_buf[uart0_cmd_len] = '\0';
                if (strncmp(uart0_cmd_buf, "$SET,", 5U) == 0U) {
                    char *name = &uart0_cmd_buf[5];
                    char *comma = strchr(name, ',');
                    if (comma != 0) {
                        float value;
                        char confirm[DEBUG_TELEMETRY_EVENT_MAX_LEN];
                        *comma = '\0';
                        value = UART0_Command_ParseFloat(comma + 1);
                        primask = __get_PRIMASK();
                        __disable_irq();
                        if (strcmp(name, "KP") == 0U) {
                            Ball_Control_SetGains(value, -1.0f);
                        } else if (strcmp(name, "KD") == 0U) {
                            Ball_Control_SetGains(-1.0f, value);
                        } else if (strcmp(name, "TRIM") == 0U) {
                            Ball_Control_SetTrimAngle(value);
                        }
                        if (primask == 0U) {
                            __enable_irq();
                        }
                        /* 回显确认：SET,<name>,<value>，确保命令确实生效 */
                        (void)strcpy(confirm, "SET,");
                        (void)strncat(confirm, name,
                                      DEBUG_TELEMETRY_EVENT_MAX_LEN - 1U);
                        (void)strncat(confirm, ",",
                                      DEBUG_TELEMETRY_EVENT_MAX_LEN - 1U);
                        (void)strncat(confirm, comma + 1,
                                      DEBUG_TELEMETRY_EVENT_MAX_LEN - 1U);
                        Debug_Telemetry_LogEvent(confirm);
                    }
#if BALL_CAL_SAVE_ENABLE
                } else if (strcmp(uart0_cmd_buf, "$SAVETEST") == 0U) {
                    /* 写 150.0 测试值并读回，验证 Flash 写入链路是否可用 */
                    float readback = 0.0f;
                    if ((CalibStore_Save(150.0f) != 0U) &&
                        (CalibStore_Load(&readback) != 0U) &&
                        (readback == 150.0f)) {
                        Debug_Telemetry_LogEvent("SAVETEST,OK");
                    } else {
                        Debug_Telemetry_LogEvent("SAVETEST,FAIL");
                    }
                } else if (strcmp(uart0_cmd_buf, "$CAL") == 0U) {
#else
                } else if (strcmp(uart0_cmd_buf, "$CAL") == 0U) {
#endif
                    Ball_Calibrate_Start();
                }
            }
            uart0_cmd_len = 0U;
        } else if (uart0_cmd_len < (UART0_CMD_BUF_LEN - 1U)) {
            uart0_cmd_buf[uart0_cmd_len++] = ch;
        } else {
            uart0_cmd_len = 0U;  /* 溢出重置 */
        }
    }
}
