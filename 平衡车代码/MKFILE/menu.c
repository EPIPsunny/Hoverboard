#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "oled.h"
#include "delay.h"
#include "SOLGUI_Include.h"
#include "string.h"
#include "menu.h"
#include "key.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "encoder.h"
#include "wifi.h"


MENU_PAGE FirstPage;
MENU_PAGE MODE1;
MENU_PAGE MODE2;
extern int mode;
extern float pitch;
extern float roll;
extern float yaw;
extern short aacx,aacy,aacz;		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern short temp;					//温度
extern int SPEED_R;
extern int SPEED_L;
extern int Encoder_A_EXTI,Encoder_B_EXTI;
extern int lspeed,rspeed;
extern int tempary;
extern int Vertical_Kp;//直立环
extern float Vertical_Kd;//直立环
extern float Velocity_Kp;//速度环
extern float Velocity_Ki;//速度环
extern int meddle;
extern int Target_Speed;

void menu_int(void)
{
   SOLGUI_Init(&FirstPage);
}

__M_PAGE(FirstPage,"First Page",PAGE_NULL,
{
	SOLGUI_Cursor(6,0,4);                                //设置文本选项数
  //SOLGUI_Widget_GotoPage(0,&MODE1);                     //建立子页面
	//SOLGUI_Widget_GotoPage(1,&MODE2);
	SOLGUI_Widget_OptionText(3,"Velocity_Ki=%f",Velocity_Ki);	
  //SOLGUI_Widget_OptionText(0,"roll=%.4f" ,roll);
//	SOLGUI_Widget_OptionText(1,"yaw=%.4f" ,yaw);
	SOLGUI_Widget_OptionText(0,"Vertical_Kp=%d" ,Vertical_Kp);
	SOLGUI_Widget_OptionText(1,"Vertical_Kd=%f" ,Vertical_Kd);
	SOLGUI_Widget_OptionText(2,"Velocity_Kp=%f" ,Velocity_Kp);
	//SOLGUI_Widget_OptionText(0,"meddle:%d" ,meddle);
});

__M_PAGE(MODE1,"MODE1",&FirstPage,                       //建立二级菜单
{
//  mode=1;                                               //模式标志位
//  SOLGUI_Cursor(6,0,7);                                 //
//	SOLGUI_Widget_OptionText(0, "date1: %d", date1);      //建立文本选项
//	SOLGUI_Widget_Spin(1,"page",INT32,100,0,&page);       //旋钮调节函数
});
__M_PAGE( MODE2,"MODE2",&MODE1,                           //建立三级菜单 **&MODE1**重点返回 MODE1 页面
{
	
	
		
});



