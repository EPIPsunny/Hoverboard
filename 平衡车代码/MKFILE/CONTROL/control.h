#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "sys.h"



#define AIN1  PBout(12)
#define AIN2  PBout(13)
#define BIN1  PBout(14)
#define BIN2  PBout(15)

#define right_motor_stops AIN1 = 0, AIN2 = 0
#define left_motor_stops BIN1 = 0, BIN2 = 0

#define right_motor_go     AIN1 = 0, AIN2 = 1
#define right_motor_back   AIN1 = 1, AIN2 = 0
#define left_motor_go		   BIN1 = 0, BIN2 = 1
#define left_motor_back	   BIN1 = 1, BIN2 = 0
					


void INinit(void);   //tb6612逻辑初始化
void TIM1_PWM_Init(unsigned int arr,unsigned int psc);  //PA8 PA11
void left_forward(unsigned int speed);
void right_forward(unsigned int speed);
void right_back(unsigned int speed);
void left_back(unsigned int speed);

int Vertical(float Med,float Angle,float gyro_Y);//直立环
void LoadPWM(int pwm);
int Velocity(int Target,int encoder_left,int encoder_right);


		 				    
#endif
