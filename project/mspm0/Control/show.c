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
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "show.h"
#include "control.h"
#include "k230_link.h"
#include "servo.h"
#include "ball_control.h"
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
static void oled_show_legacy(void)
{
     u8 i;
     int gray_pos_show;
     memset(OLED_GRAM,0, 128*8*sizeof(u8)); //GRAM清零但不立即刷新，防止花屏
        //=============第一行显示小车模式=======================//
	
             if(Car_Mode==0)   OLED_ShowString(0,0,"Mec ");
        else if(Car_Mode==1)   OLED_ShowString(0,0,"Omni");
        else if(Car_Mode==2)   OLED_ShowString(0,0,"AKM ");
        else if(Car_Mode==3)   OLED_ShowString(0,0,"Diff");
        else if(Car_Mode==4)   OLED_ShowString(0,0,"4WD ");
		else if(Car_Mode==5)   OLED_ShowString(0,0,"Tank");
	    if(Run_Mode==0)   OLED_ShowString(90,0,"APP");
        else if(Run_Mode==1)   OLED_ShowString(90,0,"GRY");
	
		OLED_ShowString(00,10,"G");
        for(i=0;i<8;i++)
        {
            OLED_ShowString(12+i*10,10,Gray_Raw[i] ? "1" : "0");
        }
		gray_pos_show = (int)Gray_Line_Pos_mm;
		OLED_ShowString(00,20,"P");
		if(gray_pos_show<0)    OLED_ShowString(16,20,"-");
		if(gray_pos_show>=0)   OLED_ShowString(16,20,"+");
		OLED_ShowNumber(26,20,myabs(gray_pos_show),3,12);
		OLED_ShowString(60,20,"Z");
		if(Move_Z<0)    OLED_ShowString(76,20,"-");
		if(Move_Z>=0)   OLED_ShowString(76,20,"+");
		OLED_ShowNumber(86,20,myabs((int)(Move_Z*1000)),4,12);
        //=============第四行显示左编码器PWM与读数=======================//
                              OLED_ShowString(00,30,"L");
        if((MotorA.Target_Encoder*1000)<0)          OLED_ShowString(16,30,"-"),
                                                  OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);
        if((MotorA.Target_Encoder*1000)>=0)       OLED_ShowString(16,30,"+"),
                              OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);

        if(MotorA.Current_Encoder<0)   OLED_ShowString(60,30,"-");
        if(MotorA.Current_Encoder>=0)    OLED_ShowString(60,30,"+");
                              OLED_ShowNumber(68,30,myabs((int)(MotorA.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,30,"mm/s");

        //=============第五行显示右编码器PWM与读数=======================//
                              OLED_ShowString(00,40,"R");
        if((MotorB.Target_Encoder*1000)<0)         OLED_ShowString(16,40,"-"),
                                                    OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);
        if((MotorB.Target_Encoder*1000)>=0)    		OLED_ShowString(16,40,"+"),
													OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);

        if(MotorB.Current_Encoder<0)    OLED_ShowString(60,40,"-");
        if(MotorB.Current_Encoder>=0)   OLED_ShowString(60,40,"+");
                              OLED_ShowNumber(68,40,myabs((int)(MotorB.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,40,"mm/s");

        //=============第六行显示电压与电机开关=======================//
                              OLED_ShowString(0,50,"V");
                                                    OLED_ShowString(30,50,".");
                                                    OLED_ShowString(64,50,"V");
                                                    OLED_ShowNumber(19,50,(int)Voltage,2,12);
                                                    OLED_ShowNumber(39,50,(u16)(Voltage*10)%10,2,12);
        if(Flag_Stop)         OLED_ShowString(95,50,"OFF");
        if(!Flag_Stop)        OLED_ShowString(95,50,"ON ");

        //=============刷新=======================//
        OLED_Refresh_Gram();
		
}

/*
 * 当前联调页：UART1 已分配给 K230，因此 OLED 专门显示闭环关键量。
 * K230 无有效帧（包括超时）时，不显示上一帧位置，避免误判为有效数据。
 */
#define OLED_REFRESH_PERIOD_MS 200U

void oled_show(void)
{
    K230_BallPosition position;
    uint16_t pulse_us;
    uint16_t duty_x100;
    int x_tenths;
    const Ball_Control *ball_control;
    static uint32_t last_refresh_ms = 0U;

    K230_Link_GetPosition(&position);
    ball_control = Ball_Control_Get();
    /* OLED 软件模拟 SPI 是主循环瓶颈，200ms 节流避免拖慢遥测/控制。 */
    if ((uint32_t)(ball_control->control_now_ms - last_refresh_ms) <
        OLED_REFRESH_PERIOD_MS) {
        return;
    }
    last_refresh_ms = ball_control->control_now_ms;
    pulse_us = Servo_GetPulseUs();
    /* 20 ms 周期：775 表示 7.75%。 */
    duty_x100 = (uint16_t)(((uint32_t)pulse_us * 10000U + 10000U) / 20000U);

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    OLED_ShowString(0, 0, "PWM:");
    OLED_ShowNumber(28, 0, pulse_us, 4, 12);
    OLED_ShowString(54, 0, "us");

    OLED_ShowString(0, 12, "DUTY:");
    OLED_ShowNumber(34, 12, duty_x100 / 100U, 2, 12);
    OLED_ShowString(46, 12, ".");
    OLED_ShowNumber(52, 12, duty_x100 % 100U, 2, 12);
    OLED_ShowString(64, 12, "%");

    OLED_ShowString(0, 28, "K230:");
    OLED_ShowString(36, 28, position.valid ? "OK" : "LS");

    OLED_ShowString(0, 44, "X:");
    if (position.valid == 0U) {
        OLED_ShowString(18, 44, "--.-mm");
    } else {
        x_tenths = (int)(position.x_mm * 10.0f);
        if (x_tenths < 0) {
            OLED_ShowString(18, 44, "-");
            x_tenths = -x_tenths;
        } else {
            OLED_ShowString(18, 44, "+");
        }
        OLED_ShowNumber(24, 44, (uint32_t)x_tenths / 10U, 3, 12);
        OLED_ShowString(42, 44, ".");
        OLED_ShowNumber(48, 44, (uint32_t)x_tenths % 10U, 1, 12);
        OLED_ShowString(54, 44, "mm");
    }

    OLED_ShowString(0, 56, "CTRL:");
    switch (ball_control->phase) {
        case BALL_CONTROL_PHASE_CAPTURE:
            OLED_ShowString(36, 56, "CAP ");
            break;
        case BALL_CONTROL_PHASE_ACCEL:
            OLED_ShowString(36, 56, "ACC ");
            break;
        case BALL_CONTROL_PHASE_RUN:
            OLED_ShowString(36, 56, "RUN ");
            break;
        case BALL_CONTROL_PHASE_BRAKE:
            OLED_ShowString(36, 56, "BRK ");
            break;
        case BALL_CONTROL_PHASE_EDGE:
            OLED_ShowString(36, 56, "EDGE");
            break;
        case BALL_CONTROL_PHASE_FAULT:
            OLED_ShowString(36, 56, "FLT ");
            break;
        case BALL_CONTROL_PHASE_HOLD:
            OLED_ShowString(36, 56, "HOLD");
            break;
        case BALL_CONTROL_PHASE_LOST:
            OLED_ShowString(36, 56, "LOST");
            break;
        case BALL_CONTROL_PHASE_PASS:
            OLED_ShowString(36, 56, "PASS");
            break;
        case BALL_CONTROL_PHASE_OFF:
        default:
            OLED_ShowString(36, 56, "OFF ");
            break;
    }

    OLED_Refresh_Gram();
}

/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{
  static u8 flag;
    int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
    Voltage_Show=(Voltage-1000)*2/3;        if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
    Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;           //对编码器数据就行数据处理便于图形化
    Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
    flag=!flag;
    if(PID_Send==1)         //发送PID参数,在APP调参界面显示
    {
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Velocity_KP,(int)Velocity_KI,(int)0,(int)0,(int)0,(int)0,0,0,0);//打印到APP上面
        PID_Send=0;
    }
   else if(flag==0)     // 发送电池电压，速度，角度等参数，在APP首页显示
        printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)0); //打印到APP上面
     else                               //发送小车姿态角，在波形界面显示
      printf("{B%d:%d:%d}$",(int)Gray_Line_Pos_mm,(int)0,(int)0); //发送灰度线位置，单位 mm
                                                                                                                    //可按格式自行增加显示波形，最多可显示五个
}
/**************************************************************************
Function: Virtual oscilloscope sends data to upper computer
Input   : none
Output  : none
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
    u8 i;//计数变量
    float Vol;                              //电压变量
    unsigned char Send_Count; //串口需要发送的数据个数
 //   Vol=(float)Voltage/100;
    DataScope_Get_Channel_Data( 0, 1 );       //显示角度 单位：度（°）
    DataScope_Get_Channel_Data( 0, 2 );         //显示超声波测量的距离 单位：CM
    DataScope_Get_Channel_Data( 0, 3 );                 //显示电池电压 单位：V
//      DataScope_Get_Channel_Data( 0 , 4 );
//      DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//      DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//      DataScope_Get_Channel_Data(0, 7 );
//      DataScope_Get_Channel_Data( 0, 8 );
//      DataScope_Get_Channel_Data(0, 9 );
//      DataScope_Get_Channel_Data( 0 , 10);
    Send_Count = DataScope_Data_Generate(3);
    for(i = 0 ; i < Send_Count; i++)
    {
//        uart0_send_char(DataScope_OutPut_Buffer[i]);
    }
}

