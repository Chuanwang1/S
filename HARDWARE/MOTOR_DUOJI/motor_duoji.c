
/**********************************
����ͷ�ļ�
**********************************/
#include "motor_duoji.h"
#include "delay.h"


/*
����countֵ���Ƶ��ת���Ƕȣ�1750-1950��Ӧ180�㣬ÿ10��Ӧ0.9�㣩
uint16_t motor_duoji_cout;			//���pwmֵ

motor_duoji_cout = 1850;				//���pwm��ʼֵ90��
TIM_SetCompare1(TIM1,cout); 		//���ö�ʱ��pwmֵ
*/


/****
*******�����ʼ������
*****/
void Motor_Duoji_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;					//ADC�����Ҷ���
	ADC_Init(ADC1, &ADC_InitStructure);															//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ��� 
	
	RCC_APB2PeriphClockCmd(MOTOR_DUOJI_TIM_CLK_ENABLE, ENABLE);			// ʹ�ܶ�ʱ��ʱ��
	RCC_APB2PeriphClockCmd(MOTOR_DUOJI_GPIO_CLK_ENABLE , ENABLE);  	//ʹ��GPIO����ʱ��

	//����IO����
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 						//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(MOTOR_DUOJI_PORT,MOTOR_DUOJI_PIN);						 			//��ʼ�����
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI_PIN_2;				 				//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 						//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(MOTOR_DUOJI_PORT,MOTOR_DUOJI_PIN_2);						 		//��ʼ�����

	//���ø�����Ϊ�����������,���PWM���岨��
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI_PIN; 									//TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI_PIN_2; 									//TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��

	//PWMƵ��===(2000*720)/72000000=0.02=20ms
	TIM_TimeBaseStructure.TIM_Period = 1999; 												//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =719; 											//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 										//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(MOTOR_DUOJI_TIM, &TIM_TimeBaseStructure); 			//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM1 Channe11 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 							//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 															//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(MOTOR_DUOJI_TIM, &TIM_OCInitStructure);  						//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 							//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 															//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//�������:TIM����Ƚϼ��Ը�
	TIM_OC4Init(MOTOR_DUOJI_TIM, &TIM_OCInitStructure);  						//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_CtrlPWMOutputs(MOTOR_DUOJI_TIM,ENABLE);											//MOE �����ʹ��

	TIM_OC1PreloadConfig(MOTOR_DUOJI_TIM, TIM_OCPreload_Enable); 	 	//Ԥװ��ʹ��
	TIM_OC4PreloadConfig(MOTOR_DUOJI_TIM, TIM_OCPreload_Enable); 	 	//Ԥװ��ʹ��	 

	TIM_ARRPreloadConfig(MOTOR_DUOJI_TIM, ENABLE); 									//ʹ�ܶ�ʱ����ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(MOTOR_DUOJI_TIM, ENABLE);  															//ʹ�ܶ�ʱ��
}

void Motor_Duoji2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(MOTOR_DUOJI2_TIM_CLK_ENABLE, ENABLE);			// ʹ�ܶ�ʱ��ʱ��
	RCC_APB2PeriphClockCmd(MOTOR_DUOJI2_GPIO_CLK_ENABLE , ENABLE);  	//ʹ��GPIO����ʱ��

	//����IO����
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI2_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 						//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI2_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(MOTOR_DUOJI2_PORT,MOTOR_DUOJI2_PIN);						 			//��ʼ�����
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI2_PIN_2;				 				//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 						//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI2_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(MOTOR_DUOJI2_PORT,MOTOR_DUOJI2_PIN_2);						 		//��ʼ�����

	//���ø�����Ϊ�����������,���PWM���岨��
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI2_PIN; 									//TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI2_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_DUOJI2_PIN_2; 									//TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//IO���ٶ�Ϊ50MHz
	GPIO_Init(MOTOR_DUOJI2_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��

	//PWMƵ��===(2000*720)/72000000=0.02=20ms
	TIM_TimeBaseStructure.TIM_Period = 1999; 												//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =719; 											//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 										//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(MOTOR_DUOJI2_TIM, &TIM_TimeBaseStructure); 			//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM1 Channe11 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 							//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 															//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(MOTOR_DUOJI2_TIM, &TIM_OCInitStructure);  						//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 							//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 															//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(MOTOR_DUOJI2_TIM, &TIM_OCInitStructure);  						//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_CtrlPWMOutputs(MOTOR_DUOJI2_TIM,ENABLE);											//MOE �����ʹ��

	TIM_OC1PreloadConfig(MOTOR_DUOJI2_TIM, TIM_OCPreload_Enable); 	 	//Ԥװ��ʹ��
	TIM_OC2PreloadConfig(MOTOR_DUOJI2_TIM, TIM_OCPreload_Enable); 	 	//Ԥװ��ʹ��	 

	TIM_ARRPreloadConfig(MOTOR_DUOJI2_TIM, ENABLE); 									//ʹ�ܶ�ʱ����ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(MOTOR_DUOJI2_TIM, ENABLE);  															//ʹ�ܶ�ʱ��
}
