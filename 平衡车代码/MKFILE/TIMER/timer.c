#include "timer.h"
//��ʱ����ʼ��
//�ṩͨ�ö�ʱ��2�ĳ�ʼ����һ����ʱ�ж�
//ʹ��ʱ�����������ͷ�ļ�
//���뺯��Timer_TIM2_Init(,);
//�߼���ʱ������ͨ��������ͬ
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//�Զ�װ��ֵ
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//Ԥ��Ƶϵ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//��ʱ����
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//ʹ�ܶ�ʱ�������ж�
	
	NVIC_InitSture.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=3;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM3,ENABLE);
}





