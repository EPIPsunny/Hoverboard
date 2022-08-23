#ifndef __WIFI_H
#define __WIFI_H	 
#include "stdio.h"	
#include "sys.h" 
#include "led.h"
#define USART1_REC_LEN  			200  	//�����������ֽ��� 200
#define USART1_SEND_LEN       200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF1[USART1_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA1;         		//����״̬���	


//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
void WIFI_GIVE_CMD(char *cmd);
void u1_printf(char* fmt,...);


#endif
