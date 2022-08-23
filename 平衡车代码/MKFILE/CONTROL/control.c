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


extern int Vertical_Kp;//ֱ����
extern float Vertical_Kd;//ֱ����
extern float Velocity_Kp;//�ٶȻ�
extern float Velocity_Ki;//�ٶȻ�
extern float pitch,roll,yaw; 		//ŷ����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;	//������ԭʼ����
extern int SPEED_R;
extern int SPEED_L;
extern int Vertical_out,Velocity_out,LastPWM;
extern int Target_Speed;
extern int meddle;


void INinit(void)   //tb6612�߼���ʼ��
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //ʹ��PD6
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;				 //ʹ��pa7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.7
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwmģʽ2������Ч��С��Ч
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//��Ч����Ϊ��
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC1Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwmģʽ2������Ч��С��Ч
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//��Ч����Ϊ��
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC4Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//�߼���ʱ��ר��--MOE�����ʹ��
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ���Զ���װ�ص�Ԥװ��
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ���Զ���װ�ص�Ԥװ��
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);   /*ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���*/
	
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
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);//��1��
	return PWM_out;
}


/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.�����ٶ�ƫ��
	Encoder_Err=((encoder_left+encoder_right)-Target);//��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
	//2.���ٶ�ƫ����е�ͨ�˲�
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S+=EnC_Err_Lowout;
	//4.�����޷�
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	
	//5.�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}




void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{
	  EXTI_ClearITPendingBit(EXTI_Line5);//����жϱ�־λ
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	  SPEED_L=-Read_Speed(2);
		SPEED_R=Read_Speed(4);
		Velocity_out=Velocity(Target_Speed,SPEED_L,SPEED_R);//�ٶȻ���ڣ�����ʵ���ٶ�
		Vertical_out=Vertical(Velocity_out+meddle,roll,gyrox);//ֱ������ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
		//LastPWM=Vertical_out-Vertical_Kp*Velocity_out;

		LoadPWM(Vertical_out);
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		
	}
}
