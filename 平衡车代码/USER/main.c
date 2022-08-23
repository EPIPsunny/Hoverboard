#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"
#include "key.h"
#include "zimo.h"                                    //取字模文件
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

float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度
int SPEED_R;
int SPEED_L;
int Encoder_A_EXTI,Encoder_B_EXTI;
int lspeed,rspeed;
int Vertical_Kp=690;//690直立环
float Vertical_Kd=0.939;//0.939直立环
float Velocity_Kp=-0.3;//0.3速度环
float Velocity_Ki=-0.0015;//0.0015速度环
int tempary;
int Vertical_out,Velocity_out,LastPWM;
int meddle=0;
int Target_Speed=0;


 void JTAGDisable()                                  //禁用JTAG	
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
	JTAGDisable();  	                                  //禁用JTAG	很重要不得删去
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	MPU_Init();					//初始化MPU6050 
  Encoder_TIM4_Init();	
  Encoder_TIM2_Init();
	 
	INinit();   //tb6612逻辑初始化
	uart1_init(115200);
	while(mpu_dmp_init());
	LED=1;
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
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
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
		}
		

	   SOLGUI_Refresh();       //OLED刷新		
	}
 }



