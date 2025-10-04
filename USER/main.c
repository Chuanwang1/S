#include "sys.h"
#include "stdio.h"
#include "string.h"
#include "delay.h"
#include "gpio.h"
#include "key.h"
#include "usart.h"
#include "motor_duoji.h"
#include "oled.h"


/**********************************
��������
**********************************/
uint8_t key_num = 0;									//����ɨ���־λ	
uint32_t time_num = 0;								//10ms��ʱ
uint8_t flag_waste_1 = 0;							//�ɻ�������Ͱ��־λ
uint8_t flag_waste_2 = 0;							//��������Ͱ��־λ
uint8_t flag_waste_3 = 0;							//�к�����Ͱ��־λ
uint8_t flag_waste_4 = 0;							//��������Ͱ��־λ
extern uint8_t usart2_buf[64];				//����2��������

/**********************************
��������
**********************************/
void Key_function(void);							//��������
void Monitor_function(void);					//��⺯��
void Manage_function(void);						//������


/****
*******	������ 
*****/
int main()
{
	Delay_Init();	    	 								//��ʱ��ʼ��	  
	Gpio_Init();		  									//IO��ʼ��
	Key_Init();		  										//������ʼ��
	Motor_Duoji_Init();    							//�����ʼ��
	Motor_Duoji2_Init();   							//�����ʼ��
	OLED_Init();
	Usart1_Init(9600);									//����1��ʼ��
	Usart2_Init(9600);									//����2��ʼ��
	
	TIM_SetCompare1(TIM1,1830); 				//PWM���
	Delay_ms(100);											//��ʱ100ms
	TIM_SetCompare4(TIM1,1830); 				//PWM���
	Delay_ms(100);											//��ʱ100ms
	TIM_SetCompare1(TIM2,1830); 				//PWM���
	Delay_ms(100);											//��ʱ100ms
	TIM_SetCompare2(TIM2,1830); 				//PWM���
	Delay_ms(100);											//��ʱ100ms
	OLED_Clear();
	OLED_ShowChinese(0,0,0,16,1);
	OLED_ShowChinese(16,0,1,16,1);
	OLED_ShowChinese(32,0,2,16,1);
	OLED_ShowChinese(48,0,15,16,1);
	OLED_ShowChinese(64,0,16,16,1);
	OLED_ShowChar(80,0,':',16,1);
	OLED_ShowChinese(96,0,11,16,1);
	OLED_ShowChinese(112,0,12,16,1);
	
	OLED_ShowChinese(0,16,3,16,1);
	OLED_ShowChinese(16,16,4,16,1);
	OLED_ShowChinese(32,16,15,16,1);
	OLED_ShowChinese(48,16,16,16,1);	
	OLED_ShowChar(80,16,':',16,1);
	OLED_ShowChinese(96,16,11,16,1);
	OLED_ShowChinese(112,16,12,16,1);
	
	OLED_ShowChinese(0,32,5,16,1);
	OLED_ShowChinese(16,32,6,16,1);
	OLED_ShowChinese(32,32,15,16,1);
	OLED_ShowChinese(48,32,16,16,1);
	OLED_ShowChar(80,32,':',16,1);
	OLED_ShowChinese(96,32,11,16,1);
	OLED_ShowChinese(112,32,12,16,1);
	
	OLED_ShowChinese(0,48,7,16,1);
	OLED_ShowChinese(16,48,8,16,1);
	OLED_ShowChinese(32,48,15,16,1);
	OLED_ShowChinese(48,48,16,16,1);	
	OLED_ShowChar(80,48,':',16,1);
	OLED_ShowChinese(96,48,11,16,1);
	OLED_ShowChinese(112,48,12,16,1);
	
	while(1)
	{
		Key_function();										//��������
		Monitor_function();								//��⺯��
		Manage_function();								//������


	}
}

/****
*******��������
*****/
void Key_function(void)
{
	key_num = Chiclet_Keyboard_Scan(0);		//����ɨ��
	if(key_num != 0)											//�а�������
	{
		switch(key_num)
		{
			case 1:														//����1�����ƿɻ�������Ͱ��
				OLED_ShowChinese(96,0,9,16,1);
				OLED_ShowChinese(112,0,10,16,1);
				TIM_SetCompare1(TIM2,1920); 		//��
				Delay_ms(1000);									//��ʱ2s
				Delay_ms(1000);
				TIM_SetCompare1(TIM2,1830); 		//�ر�
				OLED_ShowChinese(96,0,11,16,1);
				OLED_ShowChinese(112,0,12,16,1);
			break;

			case 2:														//����2�����Ƴ�������Ͱ��
				OLED_ShowChinese(96,16,9,16,1);
				OLED_ShowChinese(112,16,10,16,1);
				TIM_SetCompare2(TIM2,1920); 		//��
				Delay_ms(1000);									//��ʱ2s
				Delay_ms(1000);
				TIM_SetCompare2(TIM2,1830); 		//�ر�
				OLED_ShowChinese(96,16,11,16,1);
				OLED_ShowChinese(112,16,12,16,1);
			break;

			case 3:														//����3�������к�����Ͱ��
				OLED_ShowChinese(96,32,9,16,1);
				OLED_ShowChinese(112,32,10,16,1);
				TIM_SetCompare1(TIM1,1920); 		//��
				Delay_ms(1000);									//��ʱ2s
				Delay_ms(1000);
				TIM_SetCompare1(TIM1,1830); 		//�ر�
				OLED_ShowChinese(96,32,11,16,1);
				OLED_ShowChinese(112,32,12,16,1);
			break;
				
			case 4:														//����4��������������Ͱ��
				OLED_ShowChinese(96,48,9,16,1);
				OLED_ShowChinese(112,48,10,16,1);
				TIM_SetCompare4(TIM1,1920); 		//��
				Delay_ms(1000);									//��ʱ2s
				Delay_ms(1000);
				TIM_SetCompare4(TIM1,1830); 		//�ر�
				OLED_ShowChinese(96,48,11,16,1);
				OLED_ShowChinese(112,48,12,16,1);
			break;

			default:
				break;
		}
	}
}

/****
*******��⺯��
*****/
void Monitor_function(void)
{
	if(USART2_WaitRecive() == 0)					//����յ�����
	{
		if(usart2_buf[0] == 0x01)						//�յ�����ָ���ǡ���ֽ|����ƿ|�ɻ��������������ƿɻ�������Ͱ��
		{
			OLED_ShowChinese(96,0,9,16,1);
			OLED_ShowChinese(112,0,10,16,1);
			TIM_SetCompare1(TIM2,1920); 			//��
			Delay_ms(1000);										//��ʱ2s
			Delay_ms(1000);
			TIM_SetCompare1(TIM2,1830); 			//�ر�
			USART2_Clear();										//��ջ���
			OLED_ShowChinese(96,0,11,16,1);
			OLED_ShowChinese(112,0,12,16,1);
		}
		else if(usart2_buf[0] == 0x02)			//�յ�����ָ���ǡ�ʣ��|��ͷ|���������������Ƴ�������Ͱ��
		{
			OLED_ShowChinese(96,16,9,16,1);
			OLED_ShowChinese(112,16,10,16,1);
			TIM_SetCompare2(TIM2,1930); 			//��
			Delay_ms(1000);										//��ʱ2s
			Delay_ms(1000);
			TIM_SetCompare2(TIM2,1820); 			//�ر�
			USART2_Clear();										//��ջ���
			OLED_ShowChinese(96,16,11,16,1);
			OLED_ShowChinese(112,16,12,16,1);
		}
		else if(usart2_buf[0] == 0x03)			//�յ�����ָ���ǡ�����|�к��������������к�������Ͱ��
		{
			OLED_ShowChinese(96,32,9,16,1);
			OLED_ShowChinese(112,32,10,16,1);
			TIM_SetCompare1(TIM1,1920); 			//��
			Delay_ms(1000);										//��ʱ2s
			Delay_ms(1000);
			TIM_SetCompare1(TIM1,1830); 			//�ر�
			USART2_Clear();										//��ջ���
			OLED_ShowChinese(96,32,11,16,1);
			OLED_ShowChinese(112,32,12,16,1);
		}
		else if(usart2_buf[0] == 0x04)			//�յ�����ָ���ǡ�ľͷ|������������������������Ͱ��
		{
			OLED_ShowChinese(96,48,9,16,1);
			OLED_ShowChinese(112,48,10,16,1);
			TIM_SetCompare4(TIM1,1920); 			//��
			Delay_ms(1000);										//��ʱ2s
			Delay_ms(1000);
			TIM_SetCompare4(TIM1,1830); 			//�ر�
			USART2_Clear();										//��ջ���
			OLED_ShowChinese(96,48,11,16,1);
			OLED_ShowChinese(112,48,12,16,1);
		}
	}
}
/****
*******������
*****/
void Manage_function(void)
{
	if(IR_1 == 0)													//�ɻ�������Ͱװ�������������ͨ����������"�ɻ�������Ͱ����"
	{
		Delay_ms(100);
		if(IR_1 == 0)
		{
			LED1_G = 0;
			LED1_R = 1;
			if(flag_waste_1)
			{
				UsartPrintf(USART1,"�ɻ�������Ͱ����\r\n");
				flag_waste_1 = 0;
			}
		}
	}
	else																	//δװ�����̵���
	{
		LED1_G = 1;
		LED1_R = 0;
		flag_waste_1 = 1;
	}
	
	if(IR_2 == 0)													//��������Ͱװ�������������ͨ����������"��������Ͱ����"
	{
		Delay_ms(100);
		if(IR_2 == 0)
		{
			LED2_G = 0;
			LED2_R = 1;
			if(flag_waste_2)
			{
				UsartPrintf(USART1,"��������Ͱ����\r\n");
				flag_waste_2 = 0;
			}
		}
	}
	else																	//δװ�����̵���
	{
		LED2_G = 1;
		LED2_R = 0;
		flag_waste_2 = 1;
	}
	
	if(IR_3 == 0)													//�к�����Ͱװ�������������ͨ����������"�к�����Ͱ����"
	{
		Delay_ms(100);
		if(IR_3 == 0)
		{
			LED3_G = 0;
			LED3_R = 1;
			if(flag_waste_3)
			{
				UsartPrintf(USART1,"�к�����Ͱ����\r\n");
				flag_waste_3 = 0;
			}
		}
	}
	else																	//δװ�����̵���
	{
		LED3_G = 1;
		LED3_R = 0;
		flag_waste_3 = 1;
	}
	
	if(IR_4 == 0)													//��������Ͱװ�������������ͨ����������"��������Ͱ����"
	{
		Delay_ms(100);
		if(IR_4 == 0)
		{
			LED4_G = 0;
			LED4_R = 1;
			if(flag_waste_4)
			{
				UsartPrintf(USART1,"��������Ͱ����\r\n");
				flag_waste_4 = 0;
			}
		}
	}
	else																	//δװ�����̵���
	{
		LED4_G = 1;
		LED4_R = 0;
		flag_waste_4 = 1;
	}
}

