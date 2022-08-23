#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"                                  
#include "SOLGUI_Include.h"
#include "led.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "control.h"
#include "encoder.h"
//#include "usart.c"
#include "wifi.h"
#include "string.h"
#include "mpuexti.h"
#include "timer.h"


extern int Vertical_Kp;//直立环
extern float Vertical_Kd;//直立环
extern float Velocity_Kp;//速度环
extern float Velocity_Ki;//速度环
extern float pitch,roll,yaw; 		//欧拉角
extern short aacx,aacy,aacz;		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern int SPEED_R;
extern int SPEED_L;
extern int Vertical_out,Velocity_out,LastPWM;
extern int Target_Speed;
extern int meddle;


void INinit(void)   //tb6612逻辑初始化
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //使用PD6
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}

void TIM1_PWM_Init(unsigned int arr,unsigned int psc)  //PA8 PA11
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitTypeStrue;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;				 //使用pa7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.7
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwm模式2，大有效，小无效
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//有效极性为高
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC1Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwm模式2，大有效，小无效
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//有效极性为高
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC4Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//高级定时器专属--MOE主输出使能
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//使能自动重装载的预装载
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//使能自动重装载的预装载
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);   /*使能TIMx在ARR上的预装载寄存器*/
	
	TIM_Cmd(TIM1,ENABLE);
}

void right_back(unsigned int speed)
{
  TIM_SetCompare4(TIM1,speed);
	left_motor_back;
}

void left_back(unsigned int speed)
{
  TIM_SetCompare1(TIM1,speed);
	right_motor_back;
}

void right_forward(unsigned int speed)
{
  TIM_SetCompare4(TIM1,speed);
	left_motor_go;
}

void left_forward(unsigned int speed)
{
  TIM_SetCompare1(TIM1,speed);
	right_motor_go;
}



void LoadPWM(int pwm)
{
  if(pwm>=0)
	{
		if(pwm>7200) pwm=7200;
	  left_back(pwm);
		right_back(pwm+120);
	}
	if(pwm<0)
	{
	  pwm=-pwm;
		if(pwm>7200) pwm=7200;
		left_forward(pwm);
		right_forward(pwm+120);
	}
}
	

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);//【1】
	return PWM_out;
}


/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.计算速度偏差
	Encoder_Err=((encoder_left+encoder_right)-Target);//舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	//2.对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}




void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{
	  EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	  SPEED_L=-Read_Speed(2);
		SPEED_R=Read_Speed(4);
		Velocity_out=Velocity(Target_Speed,SPEED_L,SPEED_R);//速度环入口：左右实际速度
		Vertical_out=Vertical(Velocity_out+meddle,roll,gyrox);//直立环入口：期望角度、真实角度、真实角速度
		//LastPWM=Vertical_out-Vertical_Kp*Velocity_out;

		LoadPWM(Vertical_out);
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		
	}
}
