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
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
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
/**************************************************************************
Function: Send telemetry to ESP32-C3
Input   : none
Output  : none
函数功能：主循环节拍向 ESP32-C3 发送巡线遥测帧（经 UART1）
入口参数：无
返回  值：无
帧格式：$D,<G>,<P>,<Z>,<LT>,<RT>,<LS>,<RS>\n
  G  =8路灰度掩码(bit0=通道0)  P=软化后线位置(mm)  Z=角速度(mrad/s)
  LT/RT=左右目标速度(mm/s)     LS/RS=左右实际速度(mm/s)
**************************************************************************/
void APP_Show(void)
{
    uint8_t g = 0, i;
    for (i = 0; i < 8; i++) if (Gray_Raw[i]) g |= (uint8_t)(1U << i);  /* 打包8路灰度掩码 */
    printf("$D,%d,%d,%d,%d,%d,%d,%d\n",
           g,
           (int)Gray_Line_Pos_mm,
           (int)(Move_Z * 1000.0f),
           (int)(MotorA.Target_Encoder * 1000.0f),
           (int)(MotorB.Target_Encoder * 1000.0f),
           (int)(MotorA.Current_Encoder * 1000.0f),
           (int)(MotorB.Current_Encoder * 1000.0f));
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

