#ifndef __ENCODER_H
#define __ENCODER_H	 
#include "sys.h"

void Encoder_TIM2_Init(void);
void Encoder_TIM4_Init(void);
int Read_Speed(int TIMx);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
int Read_Speed(int TIMx);
void Encoder_Init_TIM3(void);


void Encoder_Init_TIM_Exit0(void);//���ڼ�������A��ʹ��PA0����
void Encoder_Init_TIM_Exit1(void);//���ڼ�������B��ʹ��PA1����
void EXTI0_IRQHandler(void);  //�ⲿ�ж���0������	
void EXTI1_IRQHandler(void);//�ⲿ�ж���1������
void EXTI6_IRQHandler(void);
void EXTI7_IRQHandler(void);
int Read_Encoder(int TIMX);
void TIM4_Int_Init(u16 arr,u16 psc);
void Encoder_Init_TIM_Exit6(void);
void Encoder_Init_TIM_Exit7(void);
void TIM2_Int_Init(u16 arr,u16 psc);

#endif
