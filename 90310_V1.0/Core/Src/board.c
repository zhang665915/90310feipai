/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   GPIO输出--使用固件库点亮LED灯
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32 F429 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <IO_Config.h>
#include "stm32h7xx.h"
//#include "bsp_debug_usart.h"
#include "bsp_led.h"
#include "main.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "crc.h"
#include "key.h"
#include "tim.h"
#include "tjshow.h"
#include<unistd.h>
#include <stdlib.h>
#include<stdio.h>
#include "dwt_stm32_delay.h"

void SystemClock_Config(void);
static void GPIO_CLK_Init(void);
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);


void BSP_Init(void)
{
	//防止硬件上电跟不上软件初始化导致单片机死机
	uint32_t wait_Hardware = 0;
	for(uint32_t i = 0; i < 4000000; i++)		//延时，对于K0系列串口屏我们建议延时250MS
		wait_Hardware++;
	wait_Hardware = 0;
	/* 对ETH使用的内存开启保护*/
	MPU_Config();
	CPU_CACHE_Enable();
	HAL_Init();

	 //打开FPU（硬件浮点运算）
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */


	/*将Cache设置write-through方式*/
	SCB->CACR|=1<<2;		//使用MPU时不能删

	SystemClock_Config();

	GPIO_CLK_Init();
	LED_GPIO_Config();

	MX_CRC_Init_16bit();


	ST26C32_EN_Config();		//使能差分转单端模块引脚

	/*----- 产生PWM的定时器初始化 ------- */
//	MX_TIM8_Init(500-1, 480-1);	//240M/480=500K,自动重装载为500-1,PWM频率=（500K/500）= 1KHZ
//	MX_TIM15_Init(500-1, 2400-1);
//	TIM8 ->CR1 |=  1 << 7;          //CR1的  bit7: ARPE = 1时影子寄存器有效，只有在事件更新时才 把ARR的值赋给影子寄存器
//	TIM8 ->CCMR1 |=  1 << 3;        //CCMR2的 bit3 : OC3PE = 1,当发生下一更新事件才更新CCR寄存器的值
//	TIM15 ->CR1 |=  1 << 7;			//这两句寄存器的操作是为了让每个PWM输出完整周期的波形后再修改它的频率
//	TIM15 ->CCMR1 |=  1 << 3;

    Encoder1_Init(0, 5);			//初始化TIM1
    Encoder3_Init(0, 5);			   //初始化TIM3
    Encoder4_Init(0, 5);			//初始化TIM4
    Encoder5_Init(0, 5);			//初始化TIM5

    Encoder_Start( &htim1 );		//开启编码器1计数
    Encoder_Start( &htim3 );
    Encoder_Start( &htim4 );
    Encoder_Start( &htim5 );

    MX_TIM2_Init(50-1,24-1);
    MX_TIM12_Init(1000-1,240-1);
    //

//    if(DWT_Delay_Init())
//      {
//        Error_Handler(); /* Call Error Handler */
//      }

	//串口初始化
//	MX_USART1_UART_Init(BOUND_115200);			//初始化USART1，串口屏使用
//
//	MX_USART2_UART_Init(BOUND_115200);			//初始化USART2，RS232用，上传合成速度给工控机
//	MX_USART3_UART_Init(BOUND_115200);
//	MX_USART4_UART_Init(BOUND_115200);
//	MX_USART5_UART_Init(BOUND_115200);
//	MX_USART6_UART_Init(BOUND_115200);			//初始化USART6，调试用
//	MX_USART7_UART_Init(BOUND_115200);
//	MX_USART8_UART_Init(BOUND_115200);

	Key_GPIO_Config();

	TJC_Init();						//清除串口屏的显示值

	setvbuf(stdout,NULL,_IONBF,0); 	//直接将缓冲区禁止，串口屏相关

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable
	*/
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);		//级别数值越小工作频率越高,实际上应该是SCALE1最好

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;

	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
							  	  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
								  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)				//FLASH 延迟 Latency，当调压器设置为1时，要想达到200M以上，延迟对应2，具体见手册
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_FDCAN;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	//FDCAN1时钟源配置为PLL1Q
	PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//	/** Supply configuration update enable
//	*/
//	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//	/** Configure the main internal regulator output voltage
//	*/
//	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//	/** Initializes the CPU, AHB and APB busses clocks
//	*/
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
//	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//	RCC_OscInitStruct.PLL.PLLM = 5;
//	RCC_OscInitStruct.PLL.PLLN = 160;
//	RCC_OscInitStruct.PLL.PLLP = 2;
//	RCC_OscInitStruct.PLL.PLLQ = 4;
//	RCC_OscInitStruct.PLL.PLLR = 2;
//	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
//	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//	RCC_OscInitStruct.PLL.PLLFRACN = 0;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	/** Initializes the CPU, AHB and APB busses clocks
//	*/
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//							  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART1;
//	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
//	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
//	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
//	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
////  /** Enable USB Voltage detector
////  */
////  HAL_PWREx_EnableUSBVoltageDetector();	//开启USB电压检测中断，此处打开导致CAN初始化失败
//}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_CLK_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  
}


/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


/**
  * @brief  配置MPU外设 
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as Device not cacheable 
     for ETH DMA descriptors */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Cacheable write through 
     for LwIP RAM heap which contains the Tx buffers */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
