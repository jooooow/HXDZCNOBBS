#include "stm32f4xx.h"
#include "delay.h"
#include "timer7.h"
#include "timer6.h"
#include "timer4.h"
#include "iic.h"
#include "usart.h"
#include "stdlib.h"

#define C2_HIGH()	GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define C2_LOW()  GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define ZHENG()	  GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define FAN() 		GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define MOTOR_5V_PIN1_HIGH	GPIO_SetBits(GPIOE, GPIO_Pin_5)
#define MOTOR_5V_PIN1_LOW	  GPIO_ResetBits(GPIOE, GPIO_Pin_5)
#define MOTOR_5V_PIN2_HIGH	GPIO_SetBits(GPIOE, GPIO_Pin_6)
#define MOTOR_5V_PIN2_LOW	  GPIO_ResetBits(GPIOE, GPIO_Pin_6)
#define MOTOR_5V_PIN3_HIGH	GPIO_SetBits(GPIOE, GPIO_Pin_4)
#define MOTOR_5V_PIN3_LOW	  GPIO_ResetBits(GPIOE, GPIO_Pin_4)
#define MOTOR_5V_PIN4_HIGH	GPIO_SetBits(GPIOE, GPIO_Pin_12)
#define MOTOR_5V_PIN4_LOW	  GPIO_ResetBits(GPIOE, GPIO_Pin_12)

#define POSITIVE	0
#define NEGATIVE  1

//init
void InitJy61Task(void);
void InitMotor42Task(void);
void InitMotor28Task(void);
void InitMotor42Pin(void);
void InitMotor28Pin(void);

//task
void Jy61Task(void);
void ADIS16470Task(void);
void Motor42Task(void);
void Motor28Task(void);

//move
void Motor42SpeedConvert(short sp);
void Motor28SpeedConvert(short sp);
void Motor42Move(u8 dir);
void Motor28Move(u8 dir);

u8 direction = 0;
u8 motor_5v_cnt = 0;

extern u8 jy61_read_done_flag;
unsigned char jy61_angle_raw_data[6];
extern unsigned char usart3_tx_buffer[40];
float jy61_roll,jy61_pitch,jy61_yaw;
extern u8 motor_28_dirction;
extern u8 motor_42_dirction;
u8 motor_42_stop;
u8 motor_28_stop;

u8 read_jy61_flag;
u8 motor_42_flag;
u8 motor_28_flag;

short motor_42_speed = 0;
short motor_28_speed = 0;

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float set;
	float real;
	
	float error;
	float old_error;
	
	float kp_output;
	float ki_output;
	float kd_output;
	float output;
}LocationPidStruct;

LocationPidStruct roll_angel_pid = {5,0,0,0,0,0,0,0,0,0,0};
LocationPidStruct pitch_angel_pid = {0,0,0,0,0,0,0,0,0,0,0};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart3_init(115200);
	InitJy61Task();
	InitMotor42Task();
	InitMotor28Task();

	TIM_Cmd(TIM4,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
	TIM_Cmd(TIM7,ENABLE);
	
	static unsigned short loop_cnt;

	while(1)
	{
		if(read_jy61_flag == 1)
		{
			Jy61Task();
			read_jy61_flag = 0;
		}
		Motor42Task();
		Motor28Task();
		
		loop_cnt++;
		if(loop_cnt == 100)
		{
			sprintf((char*)usart3_tx_buffer,"%.1f,\t%.1f\t%.1f\r\n",jy61_roll,roll_angel_pid.error,roll_angel_pid.output);
			DMA_Cmd(DMA1_Stream3, ENABLE);
			loop_cnt = 0;
		}
	}
	
	return 0;
}

//init
void InitJy61Task(void)
{
	IIC_Init();
	Timer4Init();
}

void InitMotor42Task(void)
{
	InitMotor42Pin();
	init_timer7(900,200);
}

void InitMotor28Task(void)
{
	InitMotor28Pin();
	init_timer6(900,200);
}

void InitMotor42Pin(void)
{
	GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
}

void InitMotor28Pin(void)
{
	GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);
}

//task
void Jy61Task(void)
{
	IIC_Read_Len(0x50,0x3d,6,&jy61_angle_raw_data[0]);

	jy61_roll = (float)(((short)jy61_angle_raw_data[1]<<8)|jy61_angle_raw_data[0])/32768*180;
	jy61_pitch = (float)(((short)jy61_angle_raw_data[3]<<8)|jy61_angle_raw_data[2])/32768*180;

	if(jy61_roll >= 180 && jy61_roll <= 360)
		jy61_roll = 360 - jy61_roll;
	else 
		jy61_roll = -jy61_roll;
	
	if(jy61_pitch >= 180 && jy61_pitch <= 360)
		jy61_pitch = 360 - jy61_pitch;
	else 
		jy61_pitch = -jy61_pitch;
}

void Motor42Task(void)
{
	roll_angel_pid.set = 0;
	roll_angel_pid.real = jy61_roll;
	roll_angel_pid.error = roll_angel_pid.set - roll_angel_pid.real;
	
	if(abs(roll_angel_pid.error) < 1)
	{
		roll_angel_pid.error = 0;
	}
	
	roll_angel_pid.kp_output = roll_angel_pid.kp*roll_angel_pid.error;
	roll_angel_pid.output = roll_angel_pid.kp_output;
	
	Motor42SpeedConvert(roll_angel_pid.output);
}

void Motor28Task(void)
{	
	pitch_angel_pid.set = 20;
	pitch_angel_pid.real = jy61_pitch;
	pitch_angel_pid.error = pitch_angel_pid.set - pitch_angel_pid.real;
	pitch_angel_pid.kp_output = pitch_angel_pid.kp*pitch_angel_pid.error;
	pitch_angel_pid.output = pitch_angel_pid.kp_output;
	Motor28SpeedConvert(pitch_angel_pid.output);
}

//move
void Motor42SpeedConvert(short sp)
{
	static short speed = 0;
	static short old_speed = 0;
	
	if(sp != 0)
		speed = (sp > 0 ? 1 : -1)*(4000 - abs(sp));
	else
		speed = 0;
	
	if(speed != old_speed && sp != 0)
	{
		init_timer6(900,abs(speed));
	}
	
	if(speed > 0)
	{
		motor_42_stop = 0;
		motor_42_dirction = POSITIVE;
	}
	else if(speed < 0)
	{
		motor_42_stop = 0;
		motor_42_dirction = NEGATIVE;
	}
	
	if(sp == 0)
	{
		motor_42_stop = 1;
	}
	
	old_speed = speed;
}
void Motor28SpeedConvert(short sp)
{
	static short speed = 0;
	static short old_speed = 0;
	
	if(sp != 0)
		speed = (sp > 0 ? 1 : -1)*(4000 - abs(sp));
	else 
		speed = 0;
	
	if(speed != old_speed && speed != 0)
	{
		init_timer7(900,abs(speed));
	}
	
	if(speed > 0)
	{
		motor_28_stop = 0;
		motor_28_dirction = POSITIVE;
	}
	else if(speed < 0)
	{
		motor_28_stop = 0;
		motor_28_dirction = NEGATIVE;
	}
	
	if(sp == 0)
	{
		motor_28_stop = 1;
	}
	
	old_speed = speed;
}
void Motor42Move(u8 dir)
{
	static u8 is_high = 1;
	if(dir == POSITIVE)
		ZHENG();
	else
		FAN();
	if(is_high)
		C2_HIGH();
	else
		C2_LOW();
	is_high = !is_high;
}

void Motor28Move(u8 dir)
{
	static signed char index = 3;
	switch(index)
	{
		case 0:
			MOTOR_5V_PIN1_HIGH;
			MOTOR_5V_PIN2_LOW;
			MOTOR_5V_PIN3_LOW;
			MOTOR_5V_PIN4_LOW;
			break;
		case 1:
			MOTOR_5V_PIN1_LOW;
			MOTOR_5V_PIN2_HIGH;
			MOTOR_5V_PIN3_LOW;
			MOTOR_5V_PIN4_LOW;
			break;
		case 2:
			MOTOR_5V_PIN1_LOW;
			MOTOR_5V_PIN2_LOW;
			MOTOR_5V_PIN3_HIGH;
			MOTOR_5V_PIN4_LOW;
			break;
		case 3:
			MOTOR_5V_PIN1_LOW;
			MOTOR_5V_PIN2_LOW;
			MOTOR_5V_PIN3_LOW;
			MOTOR_5V_PIN4_HIGH;
			break;
	}
	if(dir == POSITIVE)
		index++;
	else
		index--;
	if(index > 3)
		index = 0;
	else if(index < 0)
		index = 3;
}
