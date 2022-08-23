#include "encoder.h"
#include "stm32f10x.h"

extern int Encoder_A_EXTI,Encoder_B_EXTI;
extern int lspeed,rspeed;
/**************************************************************************
�������ܣ��ⲿ�жϲɼ���������ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM_Exit0(void)//���ڼ�������A��ʹ��PA0����
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	EXTI_InitTypeDef EXTI_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
   
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PA�˿�ʱ��
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	            //�˿�����
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
   	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
 	  EXTI_InitStructure.EXTI_Line=EXTI_Line0;
 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//�����ش���
 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 	  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
     
  // �����ж����ȼ� 													
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void Encoder_Init_TIM_Exit1(void)//���ڼ�������B��ʹ��PA1����
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	EXTI_InitTypeDef EXTI_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PA�˿�ʱ��
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	            //�˿�����
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
   	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
 	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//�����ش���
 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
     
  //�����ж����ȼ�												
   NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;	  
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

//void Encoder_Init_TIM_Exit6(void)//���ڼ�������A��ʹ��PA0����
//{
//   	GPIO_InitTypeDef GPIO_InitStructure;
//   	EXTI_InitTypeDef EXTI_InitStructure;
//   	NVIC_InitTypeDef NVIC_InitStructure;
//   
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PA�˿�ʱ��
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	            //�˿�����
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
//   	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
// 	  EXTI_InitStructure.EXTI_Line=EXTI_Line6;
// 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
// 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//�����ش���
// 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
//     
//  // �����ж����ȼ� 													
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

//void Encoder_Init_TIM_Exit7(void)//���ڼ�������A��ʹ��PA0����
//{
//   	GPIO_InitTypeDef GPIO_InitStructure;
//   	EXTI_InitTypeDef EXTI_InitStructure;
//   	NVIC_InitTypeDef NVIC_InitStructure;
//   
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PA�˿�ʱ��
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	            //�˿�����
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
//   	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
// 	  EXTI_InitStructure.EXTI_Line=EXTI_Line7;
// 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
// 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//�����ش���
// 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
//     
//  // �����ж����ȼ� 													
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//�Զ�װ��ֵ
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//Ԥ��Ƶϵ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//��ʱ����
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//ʹ�ܶ�ʱ�������ж�
	
	NVIC_InitSture.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=2;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM4,ENABLE);
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//�Զ�װ��ֵ
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//Ԥ��Ƶϵ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//��ʱ����
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//ʹ�ܶ�ʱ�������ж�
	
	NVIC_InitSture.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=2;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM2,ENABLE);
}



int Read_Encoder(int TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM=(short)Encoder_A_EXTI;  Encoder_A_EXTI=0; break;	
		 case 4:  Encoder_TIM=(short)Encoder_B_EXTI;  Encoder_B_EXTI=0; break;
	   default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}

void EXTI1_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line1);
	Encoder_A_EXTI++;
}





//void EXTI0_IRQHandler(void)  //�ⲿ�ж���0������
//{
//		EXTI->PR=1<<0;  //���LINE�ϵ��жϱ�־λ
//		if(PAin(0)==0)   //�����жϼ�⵽���Ƿ����½���
//	{
//	if(PAin(1)==0)   Encoder_A_EXTI++;//��B��ĵ�ƽ����Ǹߵ��������ת���1��������Ƿ�ת��1
//	else             Encoder_A_EXTI--;
//	}
//	else                  //������
//	{ 
//	if(PAin(1)==0)  Encoder_A_EXTI--; //B��͵�ƽΪ��ת����1���ߵ�ƽ��ת��1
//	else             Encoder_A_EXTI++;
//	}		
//}

//void EXTI1_IRQHandler(void)//�ⲿ�ж���1������
//{			
//		EXTI->PR=1<<1;  //���LINE�ϵ��жϱ�־λ
//	if(PAin(1)==1) //�����жϼ�⵽���Ƿ���������
//	{
//	if(PAin(0)==0)  Encoder_A_EXTI++;  //��A��ĵ�ƽ����ǵ�,���������ת���1��������Ƿ�ת��1 
//	else            Encoder_A_EXTI--;
//	}
//	else
//	{
//	if(PAin(0)==0)  Encoder_A_EXTI--;
//	else            Encoder_A_EXTI++;
//	}		
//}

///////////////////////////////////////////////
void Encoder_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter=10;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	TIM_SetCounter(TIM2,0);

	TIM_Cmd(TIM2,ENABLE);
}


void Encoder_TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter=10;
	TIM_ICInit(TIM4,&TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	TIM_SetCounter(TIM4,0);

	TIM_Cmd(TIM4,ENABLE);
}

/**********************
������
�ٶȶ�ȡ����
��ڲ�������ʱ��
**********************/
int Read_Speed(int TIMx)
{
	int value_1;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2,0);break;//IF�Ƕ�ʱ��2��1.�ɼ��������ļ���ֵ�����档2.����ʱ���ļ���ֵ���㡣
		case 4:value_1=(short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4,0);break;
		default:value_1=0;
	}
	return value_1;
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		//lspeed=Encoder_A_EXTI;
	}
}
	
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		//lspeed=Encoder_A_EXTI;
	}
}
///////////////////////////////////////////////////////////////////////

