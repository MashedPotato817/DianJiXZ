#include "user.h"

int Flag_Stop = 1;


void user_init(void)
{
	Velocity_KP = V_PID.Kp;
	Velocity_KI = V_PID.ki;
	
	OLED_Init();
	
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(ENCODERB_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

	Flag_Stop = 1;
	MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;
	

}

void user_main(void)
{
	static uint32_t last_oled_tick = 0;
	static uint32_t last_printf_tick = 0;
	uint32_t now;
	
	while (1)
	{
		now = Systick_getTick();
		
		// OLED每100ms刷新一次 (10Hz), 避免闪屏浪费CPU
		if (((now - last_oled_tick) & SysTickMAX_COUNT) >= SysTick_MS(100))
		{
			oled_show();
			last_oled_tick = now;
		}
		
		// 串口每500ms打印一次 (2Hz)
		if (((now - last_printf_tick) & SysTickMAX_COUNT) >= SysTick_MS(500))
		{
			printf("MA_V:%.2lf m/s \tMB_V:%.2lf m/s\r\n",MotorA.Current_Encoder,MotorB.Current_Encoder);
			last_printf_tick = now;
		}
	}
}

void Key(void)
{
	u8 tmp;
	tmp=click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
		if(!Flag_Stop)
		{
			MotorA.Target_Encoder = MotorB.Target_Encoder = 1.0f;
		}
		else
		{
			MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;

		}
	}		//单击控制小车的启停
}

//10ms
void TIMER_0_INST_IRQHandler(void)
{
	if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
		if(DL_TIMER_IIDX_ZERO)	
		{
			LED_Flash(100);//led闪烁
			Key();//获取当前按键状态
			Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);//获取编码器数据并计算线速度
			Get_Encoder_countA = Get_Encoder_countB = 0;
			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	//左轮PI控制
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);//右轮PI控制
			Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
		}			
	}	
}

/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
	Get_Encoder();
}

