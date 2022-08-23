#include "KEY.h"
#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"

u8 sw_flag = 0;


//###########################�����塿##############################



static u8 Is_Key_U_Down(void)	{ 	return PBin(13); 	}			//��

static u8 Is_Key_D_Down(void)	{ 	return PBin(14); 	}			//��

static u8 Is_Key_L_Down(void)	{ 	return PAin(11); 	}			//��

static u8 Is_Key_R_Down(void)	{ 	return PAin(12); 	} 		//��

static u8 Is_Key_OK_Down(void)	{ 	return PBin(12); 	}		//OK

static u8 Is_Key_BK_Down(void)	{ 	return PBin(15); 	}		//����


///////////////////////////

KEY_T _Key_R={Is_Key_R_Down,0,0,0,KEY_R_S,KEY_R_L};

KEY_T _Key_D={Is_Key_D_Down,0,0,0,KEY_D_S,KEY_D_L};

KEY_T _Key_L={Is_Key_L_Down,0,0,0,KEY_L_S,KEY_L_L};

KEY_T _Key_OK={Is_Key_OK_Down,0,0,0,KEY_OK_S,KEY_OK_L};

KEY_T _Key_U={Is_Key_U_Down,0,0,0,KEY_U_S,KEY_U_L};

KEY_T _Key_BK={Is_Key_BK_Down,0,0,0,KEY_BK_S,KEY_BK_L};

///////////////////////////////

KEY_T *Key[KEY_NUM]=
{
	&_Key_R,

	&_Key_D,

	&_Key_L,

	&_Key_OK,

	&_Key_U,

	&_Key_BK,

};

//////////////////////////

////////////////////////
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;//����һ���ṹ��ָ��
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);//GPIOB��ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//GPIOA��ʱ��ʹ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
	/*   ��ʼ��GPIOA                      */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          //�趨IO�ڵ�ģʽΪ��������/	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;             //�趨IO�ڵ�����ΪA11|A12|A15���� 		
	GPIO_Init(GPIOB,&GPIO_InitStructure);                  //GPIO_Init����
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          //�趨IO�ڵ�ģʽΪ��������/	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11|GPIO_Pin_12;             //�趨IO�ڵ�����ΪA11|A12|A15���� 		
	GPIO_Init(GPIOA,&GPIO_InitStructure);                  //GPIO_Init����
}

u8 Key_GetValue(void)		
{
	static u8 key_value=0;
	u8 i=0;
	
	key_value=KEY_NULL;
	
	for(i=0;i<KEY_NUM;i++)
	{
		KeyRead(Key[i]); 						//��ȡ��ֵ
	
		if(Key[i]->Trg==1)
		{

			key_value=Key[i]->S_Value;			//���ض̰�
		}
		if (Key[i]->Cont==1)
	    { 
	         Key[i]->fliter_cont++;       		

			if(Key[i]->fliter_cont>Long_Press_Time) 
	         { 
	             Key[i]->fliter_cont=0;

				 key_value=Key[i]->L_Value;  	//���س���	  
	         }        
	    }
		if(Key[i]->Cont==0&&Key[i]->Trg==0) 
		{
		 	Key[i]->fliter_cont=0;
		} 
	}
	return(key_value);		
}

static void KeyRead(KEY_T *key) 			   //����ĳ��̰����㷨���������жϣ�
{ 
    unsigned char ReadData=key->Is_Key_Down()^1;		   
    
	key->Trg=ReadData&(ReadData^key->Cont);      
    
	key->Cont=ReadData;                    
}





