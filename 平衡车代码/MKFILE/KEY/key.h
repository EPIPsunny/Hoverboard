#ifndef __KEY_H

#define __KEY_H

#include "sys.h"

//----������������
#define KEY_NUM 6
//----������ʱ�������
#define Long_Press_Time 100


//###########################���ڲ���##############################
typedef struct __KEY_T
{
	u8 (*Is_Key_Down)(void);		//Ӳ����⺯��
	
	u8 Trg;							//���� 
	
	u8 Cont;						//��ס
	
	u16 fliter_cont;
	
	u8 S_Value;						//�̰���ֵ
	
	u8 L_Value;						//������ֵ

}KEY_T;


//----����ֵ���塿
#define KEY_NULL 	0x00 	//δ����

#define KEY_R_S 	0x10	//�Ҽ��̰�

#define KEY_R_L 	0x11	//�Ҽ�����

#define KEY_D_S 	0x20	//�¼��̰�

#define KEY_D_L 	0x21	//�¼�����

#define KEY_L_S 	0x30	//����̰�

#define KEY_L_L 	0x31	//�������

#define KEY_OK_S 	0x40	//OK���̰�

#define KEY_OK_L 	0x41	//OK������

#define KEY_U_S 	0x50	//�ϼ��̰�

#define KEY_U_L 	0x51	//�ϼ�����

#define KEY_BK_S 	0x60	//Back���̰�

#define KEY_BK_L 	0x61	//Back������




#define KEY_UP GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)				//��

#define KEY_DO GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)				//��

#define KEY_LE GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)				//��

#define KEY_BA GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)				//��

#define KEY_OK GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)				//OK

#define KEY_RI GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)				//����


void KEY_Init(void);//IO��ʼ��
void KeyRead(KEY_T *key);
	
u8 Key_GetValue(void);			//����������ܼ���ȡ��ֵ�������������뱻�����Եĵ���(��������ʱ��)��





#endif
