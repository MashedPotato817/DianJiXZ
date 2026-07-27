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

typedef enum {
    GRAY_TRACK_FOLLOW = 0,
    GRAY_TRACK_SHARP_TURN,
    GRAY_TRACK_LOST_SEARCH
} Gray_Track_State;

/* 将 PI 状态移至文件作用域，便于急弯/搜线切换时清零，避免历史 PWM 冲击 */
static float PI_Left_Bias, PI_Left_Pwm, PI_Left_Last_Bias;
static float PI_Right_Bias, PI_Right_Pwm, PI_Right_Last_Bias;

static void Speed_PI_Reset(void)
{
    PI_Left_Bias = PI_Left_Pwm = PI_Left_Last_Bias = 0.0f;
    PI_Right_Bias = PI_Right_Pwm = PI_Right_Last_Bias = 0.0f;
}

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
    static Gray_Track_State track_state = GRAY_TRACK_FOLLOW;
    static float lost_search_angle;
    static float last_search_move_z;
    static float sharp_turn_move_z;
    static float filtered_pos_mm;
    static uint8_t line_seen;
    static uint8_t filtered_pos_valid;
    static uint8_t curve_mode;
    static uint8_t curve_enter_count;
    static uint8_t curve_exit_count;
    static uint8_t sharp_detect_count;
    static uint8_t center_detect_count;
    float pos_sum = 0;
    int black_count = 0;
    float raw_pos_mm;
    float y_m;
    float lookahead_m;
    float curvature;
    float abs_pos_mm;
    uint8_t outer_left;
    uint8_t outer_right;
    uint8_t i;

    Gray_Read_All();
    for (i = 0; i < 8; i++) {
        if (Gray_Data[i]) {
            pos_sum += Gray_Pos_mm[i];
            black_count++;
        }
    }

    if (black_count > 0) {
        raw_pos_mm = pos_sum / black_count;
        Gray_Line_Pos_mm = raw_pos_mm;  /* OLED 保留显示未经滤波的实际质心 */
    } else {
        raw_pos_mm = 0.0f;
        Gray_Line_Pos_mm = 0.0f;
    }

    /* 急弯或丢线期间只等待黑线回到中间，避免边缘黑线重新触发普通前进 */
    if ((track_state == GRAY_TRACK_SHARP_TURN) ||
        (track_state == GRAY_TRACK_LOST_SEARCH)) {
        if ((track_state == GRAY_TRACK_SHARP_TURN) && (black_count == 0)) {
            track_state = GRAY_TRACK_LOST_SEARCH;
        }
        if ((black_count > 0) &&
            (((raw_pos_mm >= 0.0f) ? raw_pos_mm : -raw_pos_mm) <=
             GRAY_SHARP_TURN_CENTER_THRESHOLD_MM)) {
            if (center_detect_count < GRAY_SHARP_TURN_CENTER_CONFIRM_TICKS) {
                center_detect_count++;
            }
        } else {
            center_detect_count = 0;
        }

        if (center_detect_count >= GRAY_SHARP_TURN_CENTER_CONFIRM_TICKS) {
            track_state = GRAY_TRACK_FOLLOW;
            filtered_pos_mm = raw_pos_mm;
            filtered_pos_valid = 1;
            curve_mode = 1;  /* 重新见线后先以弯道速度恢复前进 */
            curve_enter_count = 0;
            curve_exit_count = 0;
            center_detect_count = 0;
            lost_search_angle = 0.0f;
            Speed_PI_Reset();
        } else {
            /* 首次上电全白时停车；其他情况保持锁存方向原地找线 */
            if ((!line_seen) || (sharp_turn_move_z == 0.0f) ||
                (lost_search_angle >= GRAY_LOST_SEARCH_MAX_ANGLE_RAD)) {
                Move_X = 0.0f;
                Move_Z = 0.0f;
            } else {
                Move_X = 0.0f;
                Move_Z = sharp_turn_move_z;
                lost_search_angle += GRAY_SHARP_TURN_ANGULAR_SPEED / Frequency;
            }
            Get_Target_Encoder(Move_X, Move_Z);
            return;
        }
    }

    if (black_count == 0) {
        filtered_pos_valid = 0;
        curve_enter_count = 0;
        curve_exit_count = 0;
        center_detect_count = 0;

        /* 首次上电未识别到黑线时保持停车；已有方向时进入兜底搜线 */
        if (line_seen && (last_search_move_z != 0.0f)) {
            if (track_state != GRAY_TRACK_LOST_SEARCH) {
                track_state = GRAY_TRACK_LOST_SEARCH;
                sharp_turn_move_z = last_search_move_z;
                lost_search_angle = 0.0f;
                Speed_PI_Reset();
            }
            Move_X = 0.0f;
            Move_Z = sharp_turn_move_z;
            lost_search_angle += GRAY_LOST_SEARCH_ANGULAR_SPEED / Frequency;
        } else {
            Move_X = 0.0f;
            Move_Z = 0.0f;
        }
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    line_seen = 1;
    if (!filtered_pos_valid) {
        filtered_pos_mm = raw_pos_mm;
        filtered_pos_valid = 1;
    } else {
        filtered_pos_mm = GRAY_POS_FILTER_ALPHA * raw_pos_mm +
                          (1.0f - GRAY_POS_FILTER_ALPHA) * filtered_pos_mm;
    }

    abs_pos_mm = (raw_pos_mm >= 0.0f) ? raw_pos_mm : -raw_pos_mm;
    outer_left = (Gray_Data[0] || Gray_Data[1]) ? 1 : 0;
    outer_right = (Gray_Data[6] || Gray_Data[7]) ? 1 : 0;

    /* 黑线连续位于一侧最外端时，提前原地转向，不等全白后再补救 */
    if ((abs_pos_mm >= GRAY_SHARP_TURN_POS_THRESHOLD_MM) &&
        (((raw_pos_mm < 0.0f) && outer_left) ||
         ((raw_pos_mm > 0.0f) && outer_right))) {
        if (sharp_detect_count < GRAY_SHARP_TURN_CONFIRM_TICKS) {
            sharp_detect_count++;
        }
    } else {
        sharp_detect_count = 0;
    }

    if (sharp_detect_count >= GRAY_SHARP_TURN_CONFIRM_TICKS) {
        /* 当前实车约定：黑线在右侧时 Move_Z 为负，左侧时为正 */
        sharp_turn_move_z = (raw_pos_mm > 0.0f) ?
                            -GRAY_SHARP_TURN_ANGULAR_SPEED :
                             GRAY_SHARP_TURN_ANGULAR_SPEED;
        track_state = GRAY_TRACK_SHARP_TURN;
        lost_search_angle = 0.0f;
        center_detect_count = 0;
        sharp_detect_count = 0;
        Speed_PI_Reset();
        Move_X = 0.0f;
        Move_Z = sharp_turn_move_z;
        lost_search_angle += GRAY_SHARP_TURN_ANGULAR_SPEED / Frequency;
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    /* 速度档位采用滞回与连续帧确认，避免 18 mm 附近反复切换 */
    abs_pos_mm = (filtered_pos_mm >= 0.0f) ? filtered_pos_mm : -filtered_pos_mm;
    if (!curve_mode) {
        if ((abs_pos_mm >= GRAY_CURVE_POS_THRESHOLD_MM) ||
            (black_count >= GRAY_CURVE_BLACK_COUNT)) {
            if (curve_enter_count < GRAY_CURVE_ENTER_CONFIRM_TICKS) {
                curve_enter_count++;
            }
        } else {
            curve_enter_count = 0;
        }
        if (curve_enter_count >= GRAY_CURVE_ENTER_CONFIRM_TICKS) {
            curve_mode = 1;
            curve_enter_count = 0;
            curve_exit_count = 0;
        }
    } else {
        if ((abs_pos_mm <= GRAY_CURVE_POS_EXIT_THRESHOLD_MM) &&
            (black_count <= GRAY_CURVE_BLACK_COUNT_EXIT)) {
            if (curve_exit_count < GRAY_CURVE_EXIT_CONFIRM_TICKS) {
                curve_exit_count++;
            }
        } else {
            curve_exit_count = 0;
        }
        if (curve_exit_count >= GRAY_CURVE_EXIT_CONFIRM_TICKS) {
            curve_mode = 0;
            curve_enter_count = 0;
            curve_exit_count = 0;
        }
    }

    Move_X = curve_mode ? (GRAY_CURVE_SPEED_MM_S / 1000.0f) :
                          (GRAY_STRAIGHT_SPEED_MM_S / 1000.0f);
    y_m = filtered_pos_mm / 1000.0f;
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
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			
			Key();
			LED_Flash(100);
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
	/* 5 ms 内只有约 1 个脉冲时，单周期测速会频繁读到 0。
	 * 采用 4 周期滑动累计，每 5 ms 仍更新一次速度，兼顾连续性与响应速度。 */
	static int Encoder_HistoryA[SPEED_MEASURE_WINDOW_TICKS];
	static int Encoder_HistoryB[SPEED_MEASURE_WINDOW_TICKS];
	static int Encoder_SumA, Encoder_SumB;
	static uint8_t History_Index, History_Count;
	static float Filtered_SpeedA = 0.0f, Filtered_SpeedB = 0.0f;
	float raw_speedA, raw_speedB;
	OriginalEncoder.A = Encoder1;
	OriginalEncoder.B = Encoder2;

	Encoder_SumA -= Encoder_HistoryA[History_Index];
	Encoder_SumB -= Encoder_HistoryB[History_Index];
	Encoder_HistoryA[History_Index] = OriginalEncoder.A;
	Encoder_HistoryB[History_Index] = -OriginalEncoder.B;
	Encoder_SumA += Encoder_HistoryA[History_Index];
	Encoder_SumB += Encoder_HistoryB[History_Index];

	if (History_Count < SPEED_MEASURE_WINDOW_TICKS) History_Count++;
	History_Index++;
	if (History_Index >= SPEED_MEASURE_WINDOW_TICKS) History_Index = 0;

	raw_speedA = (float)Encoder_SumA * Frequency * Perimeter /
	             ((float)CPR * History_Count);
	raw_speedB = (float)Encoder_SumB * Frequency * Perimeter /
	             ((float)CPR * History_Count);

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
	 float abs_bias;
	 PI_Left_Bias=Target-Encoder;                					//计算偏差
	 abs_bias = (PI_Left_Bias > 0.0f) ? PI_Left_Bias : -PI_Left_Bias;
	 if(abs_bias < PI_DEADBAND) { PI_Left_Last_Bias = PI_Left_Bias; return (int)PI_Left_Pwm; }
	 PI_Left_Pwm+=Velocity_KP*(PI_Left_Bias-PI_Left_Last_Bias)+Velocity_KI*PI_Left_Bias;   	//增量式PI控制器
	if(Flag_Stop) PI_Left_Pwm=0;
	 PI_Left_Pwm = PWM_Limit(PI_Left_Pwm, PWM_MAX, -PWM_MAX);
	 PI_Left_Last_Bias=PI_Left_Bias;	                   					//保存上一次偏差
	 return (int)PI_Left_Pwm;                         				//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{
	 float abs_bias;
	 PI_Right_Bias=Target-Encoder;                					//计算偏差
	 abs_bias = (PI_Right_Bias > 0.0f) ? PI_Right_Bias : -PI_Right_Bias;
	 if(abs_bias < PI_DEADBAND) { PI_Right_Last_Bias = PI_Right_Bias; return (int)PI_Right_Pwm; }
	 PI_Right_Pwm+=Velocity_KP*(PI_Right_Bias-PI_Right_Last_Bias)+Velocity_KI*PI_Right_Bias;   	//增量式PI控制器
	if(Flag_Stop) PI_Right_Pwm=0;
	 PI_Right_Pwm = PWM_Limit(PI_Right_Pwm, PWM_MAX, -PWM_MAX);
	 PI_Right_Last_Bias=PI_Right_Bias;	                   					//保存上一次偏差
	 return (int)PI_Right_Pwm;                         				//增量输出
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
