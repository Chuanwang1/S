
#ifndef __MOTOR_DUOJI_H__
#define __MOTOR_DUOJI_H__


/**********************************
����ͷ�ļ�
**********************************/
#include "sys.h"


/**********************************
PIN�ڶ���
**********************************/
#define MOTOR_DUOJI_TIM_CLK_ENABLE          RCC_APB2Periph_TIM1				//��ʱ��1��PWMͨ��1����PA8,ͨ��2����PA9
#define MOTOR_DUOJI_TIM             				TIM1
#define MOTOR_DUOJI_GPIO_CLK_ENABLE         RCC_APB2Periph_GPIOA
#define MOTOR_DUOJI_PORT                    GPIOA
#define MOTOR_DUOJI_PIN                     GPIO_Pin_8
#define MOTOR_DUOJI_PIN_2                   GPIO_Pin_11

#define MOTOR_DUOJI2_TIM_CLK_ENABLE         RCC_APB1Periph_TIM2				//��ʱ��1��PWMͨ��1����PA8,ͨ��2����PA9
#define MOTOR_DUOJI2_TIM             				TIM2
#define MOTOR_DUOJI2_GPIO_CLK_ENABLE        RCC_APB2Periph_GPIOA
#define MOTOR_DUOJI2_PORT                   GPIOA
#define MOTOR_DUOJI2_PIN                    GPIO_Pin_0
#define MOTOR_DUOJI2_PIN_2                  GPIO_Pin_1


/**********************************
��������
**********************************/
void Motor_Duoji_Init(void);								//�����ʼ������
void Motor_Duoji2_Init(void);								//�����ʼ������
	
#endif

