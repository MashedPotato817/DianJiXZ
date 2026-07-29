#ifndef H_SERVO
#define H_SERVO

/*
 * 摆杆舵机抽象层。
 *
 * 此文件只定义角度命令与限幅，不直接写定时器寄存器。待确认舵机信号线
 * 与空闲 TimerA 通道后，在 Servo_ApplyHardware() 中接入 SysConfig 生成宏。
 */
#define SERVO_ANGLE_MIN_DEG      (-30.0f)
#define SERVO_ANGLE_MAX_DEG      (30.0f)
#define SERVO_ANGLE_NEUTRAL_DEG  (0.0f)

void Servo_Init(void);
void Servo_SetTargetAngle(float angle_deg);
float Servo_GetTargetAngle(void);
void Servo_ApplyHardware(void);

#endif
