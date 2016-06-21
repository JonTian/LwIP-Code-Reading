/*
********************************************************************************
*                                  uC/OS-III
*							
*                              ARM Cortex-M3 Port
*
* File      	: Config.C
* Version   	: V1.0
* By        	: ����ǿ
*
* For       	: Stm32f10x
* Mode      	: Thumb2
* Toolchain 	: 
*             		RealView Microcontroller Development Kit (MDK)
*             		Keil uVision
* Description   : STM32F10x �ڲ� ϵͳ������
*
*					1��ϵͳ�ж����ȼ�ģʽ����
*					2��ϵͳ��������ָ��
*					3��ϵͳʱ�Ӽ�ʱ������
*					4��оƬ���ų�ʼ��
*					
* Date          : 2012.05.22
*******************************************************************************/

#include "includes.h"
#include "stm32f10x_conf.h"
#include "bsp.h"




/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(volatile u32 nCount)
{
  for(; nCount != 0; nCount--);
}



/*******************************************************************************
* Function Name :void SysTickInit(void)
* Description   :ϵͳ��ʱ��ʱ������
* Input         :
* Output        :
* Other         :ʱ��Ϊ1ms
* Date          :2011.11.03  12:59:13
*******************************************************************************/
void SysTickInit(void)
{
	SysTick_Config(SystemCoreClock / 1000);			//uCOSʱ��1ms
}






/*******************************************************************************
* Function Name :void SystemConfig(void)
* Description   :ϵͳ��ʼ��
* Input         :
* Output        :
* Other         :
* Date          :2011.10.27  13:14:59
*******************************************************************************/
void BspInit(void)
{
  
	/*��ʼ������*/
   	USART1_Config();  	

	/*��ʼ�� ��̫��SPI�ӿ�*/
	ENC_SPI_Init(); 		

	/*��ʼ��systick�����ڶ�ʱ��ѯ������LWIP�ṩ��ʱ*/
	SysTick_Init();		

	/*��ʼ��LEDʹ�õĶ˿�*/
	LED_GPIO_Config(); 
	 
	printf("\r\n**************Ұ��STM32_enc8j60+lwip��ֲʵ��*************\r\n");
	  
  	/* ��ʼ��LWIPЭ��ջ*/
	LwIP_Init(); 

	/*��ʼ��web server ��ʾ��ҳ����*/
	httpd_init();
  
  	/* ��ʼ��telnet   Զ�̿��� ���� */   
  	CMD_init();                                       



}











