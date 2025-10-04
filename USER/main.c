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
变量定义
**********************************/
uint8_t key_num = 0;									//按键扫描标志位	
uint32_t time_num = 0;								//10ms计时
uint8_t flag_waste_1 = 0;							//可回收垃圾桶标志位
uint8_t flag_waste_2 = 0;							//厨余垃圾桶标志位
uint8_t flag_waste_3 = 0;							//有害垃圾桶标志位
uint8_t flag_waste_4 = 0;							//其他垃圾桶标志位
extern uint8_t usart2_buf[64];				//串口2接收数组

/**********************************
函数声明
**********************************/
void Key_function(void);							//按键函数
void Monitor_function(void);					//监测函数
void Manage_function(void);						//处理函数


/****
*******	主函数 
*****/
int main()
{
	Delay_Init();	    	 								//延时初始化	  
	Gpio_Init();		  									//IO初始化
	Key_Init();		  										//按键初始化
	Motor_Duoji_Init();    							//舵机初始化
	Motor_Duoji2_Init();   							//舵机初始化
	OLED_Init();
	Usart1_Init(9600);									//串口1初始化
	Usart2_Init(9600);									//串口2初始化
	
	TIM_SetCompare1(TIM1,1830); 				//PWM输出
	Delay_ms(100);											//延时100ms
	TIM_SetCompare4(TIM1,1830); 				//PWM输出
	Delay_ms(100);											//延时100ms
	TIM_SetCompare1(TIM2,1830); 				//PWM输出
	Delay_ms(100);											//延时100ms
	TIM_SetCompare2(TIM2,1830); 				//PWM输出
	Delay_ms(100);											//延时100ms
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
		Key_function();										//按键函数
		Monitor_function();								//监测函数
		Manage_function();								//处理函数


	}
}

/****
*******按键函数
*****/
void Key_function(void)
{
	key_num = Chiclet_Keyboard_Scan(0);		//按键扫描
	if(key_num != 0)											//有按键按下
	{
		switch(key_num)
		{
			case 1:														//按键1：控制可回收垃圾桶打开
				OLED_ShowChinese(96,0,9,16,1);
				OLED_ShowChinese(112,0,10,16,1);
				TIM_SetCompare1(TIM2,1920); 		//打开
				Delay_ms(1000);									//延时2s
				Delay_ms(1000);
				TIM_SetCompare1(TIM2,1830); 		//关闭
				OLED_ShowChinese(96,0,11,16,1);
				OLED_ShowChinese(112,0,12,16,1);
			break;

			case 2:														//按键2：控制厨余垃圾桶打开
				OLED_ShowChinese(96,16,9,16,1);
				OLED_ShowChinese(112,16,10,16,1);
				TIM_SetCompare2(TIM2,1920); 		//打开
				Delay_ms(1000);									//延时2s
				Delay_ms(1000);
				TIM_SetCompare2(TIM2,1830); 		//关闭
				OLED_ShowChinese(96,16,11,16,1);
				OLED_ShowChinese(112,16,12,16,1);
			break;

			case 3:														//按键3：控制有害垃圾桶打开
				OLED_ShowChinese(96,32,9,16,1);
				OLED_ShowChinese(112,32,10,16,1);
				TIM_SetCompare1(TIM1,1920); 		//打开
				Delay_ms(1000);									//延时2s
				Delay_ms(1000);
				TIM_SetCompare1(TIM1,1830); 		//关闭
				OLED_ShowChinese(96,32,11,16,1);
				OLED_ShowChinese(112,32,12,16,1);
			break;
				
			case 4:														//按键4：控制其他垃圾桶打开
				OLED_ShowChinese(96,48,9,16,1);
				OLED_ShowChinese(112,48,10,16,1);
				TIM_SetCompare4(TIM1,1920); 		//打开
				Delay_ms(1000);									//延时2s
				Delay_ms(1000);
				TIM_SetCompare4(TIM1,1830); 		//关闭
				OLED_ShowChinese(96,48,11,16,1);
				OLED_ShowChinese(112,48,12,16,1);
			break;

			default:
				break;
		}
	}
}

/****
*******监测函数
*****/
void Monitor_function(void)
{
	if(USART2_WaitRecive() == 0)					//如果收到数据
	{
		if(usart2_buf[0] == 0x01)						//收到语音指令是“废纸|塑料瓶|可回收垃圾”，控制可回收垃圾桶打开
		{
			OLED_ShowChinese(96,0,9,16,1);
			OLED_ShowChinese(112,0,10,16,1);
			TIM_SetCompare1(TIM2,1920); 			//打开
			Delay_ms(1000);										//延时2s
			Delay_ms(1000);
			TIM_SetCompare1(TIM2,1830); 			//关闭
			USART2_Clear();										//清空缓存
			OLED_ShowChinese(96,0,11,16,1);
			OLED_ShowChinese(112,0,12,16,1);
		}
		else if(usart2_buf[0] == 0x02)			//收到语音指令是“剩饭|骨头|厨余垃圾”，控制厨余垃圾桶打开
		{
			OLED_ShowChinese(96,16,9,16,1);
			OLED_ShowChinese(112,16,10,16,1);
			TIM_SetCompare2(TIM2,1930); 			//打开
			Delay_ms(1000);										//延时2s
			Delay_ms(1000);
			TIM_SetCompare2(TIM2,1820); 			//关闭
			USART2_Clear();										//清空缓存
			OLED_ShowChinese(96,16,11,16,1);
			OLED_ShowChinese(112,16,12,16,1);
		}
		else if(usart2_buf[0] == 0x03)			//收到语音指令是“灯泡|有害垃圾”，控制有害收垃圾桶打开
		{
			OLED_ShowChinese(96,32,9,16,1);
			OLED_ShowChinese(112,32,10,16,1);
			TIM_SetCompare1(TIM1,1920); 			//打开
			Delay_ms(1000);										//延时2s
			Delay_ms(1000);
			TIM_SetCompare1(TIM1,1830); 			//关闭
			USART2_Clear();										//清空缓存
			OLED_ShowChinese(96,32,11,16,1);
			OLED_ShowChinese(112,32,12,16,1);
		}
		else if(usart2_buf[0] == 0x04)			//收到语音指令是“木头|其他垃圾”，控制其他垃圾桶打开
		{
			OLED_ShowChinese(96,48,9,16,1);
			OLED_ShowChinese(112,48,10,16,1);
			TIM_SetCompare4(TIM1,1920); 			//打开
			Delay_ms(1000);										//延时2s
			Delay_ms(1000);
			TIM_SetCompare4(TIM1,1830); 			//关闭
			USART2_Clear();										//清空缓存
			OLED_ShowChinese(96,48,11,16,1);
			OLED_ShowChinese(112,48,12,16,1);
		}
	}
}
/****
*******处理函数
*****/
void Manage_function(void)
{
	if(IR_1 == 0)													//可回收垃圾桶装满，红灯亮，并通过蓝牙发送"可回收垃圾桶已满"
	{
		Delay_ms(100);
		if(IR_1 == 0)
		{
			LED1_G = 0;
			LED1_R = 1;
			if(flag_waste_1)
			{
				UsartPrintf(USART1,"可回收垃圾桶已满\r\n");
				flag_waste_1 = 0;
			}
		}
	}
	else																	//未装满，绿灯亮
	{
		LED1_G = 1;
		LED1_R = 0;
		flag_waste_1 = 1;
	}
	
	if(IR_2 == 0)													//厨余垃圾桶装满，红灯亮，并通过蓝牙发送"厨余垃圾桶已满"
	{
		Delay_ms(100);
		if(IR_2 == 0)
		{
			LED2_G = 0;
			LED2_R = 1;
			if(flag_waste_2)
			{
				UsartPrintf(USART1,"厨余垃圾桶已满\r\n");
				flag_waste_2 = 0;
			}
		}
	}
	else																	//未装满，绿灯亮
	{
		LED2_G = 1;
		LED2_R = 0;
		flag_waste_2 = 1;
	}
	
	if(IR_3 == 0)													//有害垃圾桶装满，红灯亮，并通过蓝牙发送"有害垃圾桶已满"
	{
		Delay_ms(100);
		if(IR_3 == 0)
		{
			LED3_G = 0;
			LED3_R = 1;
			if(flag_waste_3)
			{
				UsartPrintf(USART1,"有害垃圾桶已满\r\n");
				flag_waste_3 = 0;
			}
		}
	}
	else																	//未装满，绿灯亮
	{
		LED3_G = 1;
		LED3_R = 0;
		flag_waste_3 = 1;
	}
	
	if(IR_4 == 0)													//其他垃圾桶装满，红灯亮，并通过蓝牙发送"其他垃圾桶已满"
	{
		Delay_ms(100);
		if(IR_4 == 0)
		{
			LED4_G = 0;
			LED4_R = 1;
			if(flag_waste_4)
			{
				UsartPrintf(USART1,"其他垃圾桶已满\r\n");
				flag_waste_4 = 0;
			}
		}
	}
	else																	//未装满，绿灯亮
	{
		LED4_G = 1;
		LED4_R = 0;
		flag_waste_4 = 1;
	}
}

