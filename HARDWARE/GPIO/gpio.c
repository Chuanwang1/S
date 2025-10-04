
/**********************************
����ͷ�ļ�
**********************************/
#include "gpio.h"


/****
*******	LED IO��ʼ��
*****/    
void Gpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//��ʱ�Ӻ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);	//��GPIO��ʱ�ӣ��ȴ򿪸��ò����޸ĸ��ù���
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//Ҫ�ȿ�ʱ�ӣ�����ӳ�䣻����ʾ�ر�jtag��ʹ��swd��  


	RCC_APB2PeriphClockCmd(LED1_G_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED1_G_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED1_G_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED1_G_PORT,LED1_G_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED1_R_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED1_R_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED1_R_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED1_R_PORT,LED1_R_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED2_G_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED2_G_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED2_G_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED2_G_PORT,LED2_G_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED2_R_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED2_R_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED2_R_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED2_R_PORT,LED2_R_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED3_G_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED3_G_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED3_G_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED3_G_PORT,LED3_G_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED3_R_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED3_R_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED3_R_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED3_R_PORT,LED3_R_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED4_G_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED4_G_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED4_G_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED4_G_PORT,LED4_G_PIN);						 							//IO�������
	
	RCC_APB2PeriphClockCmd(LED4_R_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = LED4_R_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED4_R_PORT, &GPIO_InitStructure);					 			//�����趨������ʼ��
	GPIO_SetBits(LED4_R_PORT,LED4_R_PIN);						 							//IO�������

	RCC_APB2PeriphClockCmd(IR_1_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = IR_1_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(IR_1_PORT, &GPIO_InitStructure);					 		//�����趨������ʼ��
	GPIO_SetBits(IR_1_PORT,IR_1_PIN);						 						//IO�������
	
	RCC_APB2PeriphClockCmd(IR_2_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = IR_2_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(IR_2_PORT, &GPIO_InitStructure);					 		//�����趨������ʼ��
	GPIO_SetBits(IR_2_PORT,IR_2_PIN);						 						//IO�������
	
	RCC_APB2PeriphClockCmd(IR_3_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = IR_3_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(IR_3_PORT, &GPIO_InitStructure);					 		//�����趨������ʼ��
	GPIO_SetBits(IR_3_PORT,IR_3_PIN);						 						//IO�������
	
	RCC_APB2PeriphClockCmd(IR_4_GPIO_CLK_ENABLE, ENABLE);	 	//ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = IR_4_PIN;				 					//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(IR_4_PORT, &GPIO_InitStructure);					 		//�����趨������ʼ��
	GPIO_SetBits(IR_4_PORT,IR_4_PIN);						 						//IO�������
	
}
 
