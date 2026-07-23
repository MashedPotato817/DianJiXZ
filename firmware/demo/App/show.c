#include "show.h"
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
	memset(OLED_GRAM,0, 128*8*sizeof(u8)); //GRAM清零但不立即刷新，防止花屏
	//=============第一行显示小车模式=======================//
	OLED_ShowString(0,0,(const u8 *)"Diff");
	OLED_ShowString(0,10,(const u8 *)"Key_Control");

	//=============第二行显示PWM输出值=======================//
	OLED_ShowString(0,20,(const u8 *)"PWM");
	OLED_ShowString(30,20,(const u8 *)"L:");
	OLED_ShowNumber(46,20,myabs((int)(MotorA.Motor_Pwm)),4,12);
	OLED_ShowString(82,20,(const u8 *)"R:");
	OLED_ShowNumber(98,20,myabs((int)(MotorB.Motor_Pwm)),4,12);

	//=============第四行显示左编码器PWM与读数=======================//
	OLED_ShowString(00,30,(const u8 *)"L");
	if((MotorA.Target_Encoder*1000)<0)          
		OLED_ShowString(16,30,(const u8 *)"-"),
		OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);
	if((MotorA.Target_Encoder*1000)>=0)       
		OLED_ShowString(16,30,(const u8 *)"+"),			  
		OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);

	if(MotorA.Current_Encoder<0)   
		OLED_ShowString(60,30,(const u8 *)"-");
	if(MotorA.Current_Encoder>=0)    
		OLED_ShowString(60,30,(const u8 *)"+");
						  
	OLED_ShowNumber(68,30,myabs((int)(MotorA.Current_Encoder*1000)),4,12);
	OLED_ShowString(96,30,(const u8 *)"mm/s");

	//=============第五行显示右编码器PWM与读数=======================//
	OLED_ShowString(00,40,(const u8 *)"R");
	if((MotorB.Target_Encoder*1000)<0)         
		OLED_ShowString(16,40,(const u8 *)"-"),											
		OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);
	if((MotorB.Target_Encoder*1000)>=0)    		
		OLED_ShowString(16,40,(const u8 *)"+"),
		OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);

	if(MotorB.Current_Encoder<0)    
		OLED_ShowString(60,40,(const u8 *)"-");
	if(MotorB.Current_Encoder>=0)   
		OLED_ShowString(60,40,(const u8 *)"+");
	OLED_ShowNumber(68,40,myabs((int)(MotorB.Current_Encoder*1000)),4,12);
	OLED_ShowString(96,40,(const u8 *)"mm/s");

	//=============第六行显示状态=======================//
	if(Flag_Stop)
		OLED_ShowString(0,50,(const u8 *)"STOP ");
	else
		OLED_ShowString(0,50,(const u8 *)"RUN  ");
	
	//=============刷新=======================//
	OLED_Refresh_Gram();
		
}