/*
********************************************************************************
*                                  uC/OS-III
*							
*                              ARM Cortex-M3 Port
*
* File      	: Config.C
* Version   	: V1.0
* By        	: 王宏强
*
* For       	: Stm32f10x
* Mode      	: Thumb2
* Toolchain 	: 
*             		RealView Microcontroller Development Kit (MDK)
*             		Keil uVision
* Description   : STM32F10x 内部 系统的配置
*
*					1，系统中断优先级模式设置
*					2，系统程序启动指定
*					3，系统时钟计时器配置
*					4，芯片引脚初始化
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
* Description   :系统定时器时间配置
* Input         :
* Output        :
* Other         :时基为1ms
* Date          :2011.11.03  12:59:13
*******************************************************************************/
void SysTickInit(void)
{
	SysTick_Config(SystemCoreClock / 1000);			//uCOS时基1ms
}






/*******************************************************************************
* Function Name :void SystemConfig(void)
* Description   :系统初始化
* Input         :
* Output        :
* Other         :
* Date          :2011.10.27  13:14:59
*******************************************************************************/
void BspInit(void)
{
  
	/*初始化串口*/
   	USART1_Config();  	

	/*初始化 以太网SPI接口*/
	ENC_SPI_Init(); 		

	/*初始化systick，用于定时轮询输入或给LWIP提供定时*/
	SysTick_Init();		

	/*初始化LED使用的端口*/
	LED_GPIO_Config(); 
	 
	printf("\r\n**************野火STM32_enc8j60+lwip移植实验*************\r\n");
	  
  	/* 初始化LWIP协议栈*/
	LwIP_Init(); 

	/*初始化web server 显示网页程序*/
	httpd_init();
  
  	/* 初始化telnet   远程控制 程序 */   
  	CMD_init();                                       



}











