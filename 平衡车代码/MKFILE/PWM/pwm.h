#ifndef __PWM_H

#define __PWM_H

 #include "sys.h"

#define PWM1 TIM3->CCR1        //ģ���ϵ�PWM4    PA6
#define PWM2 TIM3->CCR2   //ģ���ϵ�PWM7        PA7
#define PWM3 TIM3->CCR3     ////ģ���ϵ�PWM3      PB0
#define PWM4 TIM3->CCR4     //ģ���ϵ�PWM0       PB1

void TIM3_PWM_Init(u16 arr,u16 psc);

#define Laser PAout(12)






#endif

