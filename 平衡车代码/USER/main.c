#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"
#include "key.h"
#include "zimo.h"                                    //ȡ��ģ�ļ�
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
/************************************************

************************************************/

float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
short temp;					//�¶�
int SPEED_R;
int SPEED_L;
int Encoder_A_EXTI,Encoder_B_EXTI;
int lspeed,rspeed;
int Vertical_Kp=690;//690ֱ����
float Vertical_Kd=0.939;//0.939ֱ����
float Velocity_Kp=-0.3;//0.3�ٶȻ�
float Velocity_Ki=-0.0015;//0.0015�ٶȻ�
int tempary;
int Vertical_out,Velocity_out,LastPWM;
int meddle=0;
int Target_Speed=0;


 void JTAGDisable()                                  //����JTAG	
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}


 int main(void)
 {	
  LED_Init();
	//LED=0;
	delay_init();
	TIM1_PWM_Init(7199,0);
	JTAGDisable();  	                                  //����JTAG	����Ҫ����ɾȥ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	MPU_Init();					//��ʼ��MPU6050 
  Encoder_TIM4_Init();	
  Encoder_TIM2_Init();
	 
	INinit();   //tb6612�߼���ʼ��
	uart1_init(115200);
	while(mpu_dmp_init());
	LED=1;
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//��������
	delay_ms(500);
	menu_int();
  OLED_Init();
	SOLGUI_Refresh();
	TIM3_Int_Init(1599,99);
	while(1)
	{
	  SOLGUI_Menu_PageStage();		
    if(USART_RX_STA1!=0)
		{					   	
			delay_ms(100);
			if (strcmp((const char *)USART_RX_BUF1, "p0")==0)        Vertical_Kp=Vertical_Kp-10;
			if (strcmp((const char *)USART_RX_BUF1, "p1")==0)        Vertical_Kp=Vertical_Kp+10;
			if (strcmp((const char *)USART_RX_BUF1, "d0")==0)        Vertical_Kd=Vertical_Kd-0.02;
			if (strcmp((const char *)USART_RX_BUF1, "d1")==0)        Vertical_Kd=Vertical_Kd+0.02;
			if (strcmp((const char *)USART_RX_BUF1, "i1")==0) 
			{
			  Velocity_Kp=Velocity_Kp+0.1;
				Velocity_Ki=Velocity_Kp/200;
			}
			if (strcmp((const char *)USART_RX_BUF1, "i0")==0) 
			{
			  Velocity_Kp=Velocity_Kp-0.1;
				Velocity_Ki=Velocity_Kp/200;
			}
			if (strcmp((const char *)USART_RX_BUF1, "h")==0)        meddle=meddle+1;
			if (strcmp((const char *)USART_RX_BUF1, "q")==0)        meddle=meddle-1;
			if (strcmp((const char *)USART_RX_BUF1, "w")==0)        Target_Speed=1;
			if (strcmp((const char *)USART_RX_BUF1, "s")==0)        Target_Speed=-1;
			USART_RX_STA1=0;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//��������
		}
		

	   SOLGUI_Refresh();       //OLEDˢ��		
	}
 }



