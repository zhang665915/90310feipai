/**
  ******************************************************************
  * @file    lan8720a.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   lan8720a应锟矫猴拷锟斤拷锟接匡拷
  ******************************************************************
  * @attention
  *
  * 实锟斤拷平台:野锟斤拷 STM32H743锟斤拷锟斤拷锟斤拷 
  * 锟斤拷坛    :http://www.firebbs.cn
  * 锟皆憋拷    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
  
#include "LAN8720a.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"

#include "bsp_led.h"
#include "debug.h"

extern ETH_HandleTypeDef EthHandle;

/**
  * @brief  锟斤拷始锟斤拷ETH锟斤拷锟斤拷.
  * @param  锟斤拷
  * @retval 锟斤拷
  */    

static void ETH_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* 使锟杰端匡拷时锟斤拷 */
    ETH_MDIO_GPIO_CLK_ENABLE();
    ETH_MDC_GPIO_CLK_ENABLE();
    ETH_RMII_REF_CLK_GPIO_CLK_ENABLE();
    ETH_RMII_CRS_DV_GPIO_CLK_ENABLE();
    ETH_RMII_RXD0_GPIO_CLK_ENABLE();
    ETH_RMII_RXD1_GPIO_CLK_ENABLE();
    ETH_RMII_TX_EN_GPIO_CLK_ENABLE();
    ETH_RMII_TXD0_GPIO_CLK_ENABLE();    
    ETH_RMII_TXD1_GPIO_CLK_ENABLE();

	
    /* 锟斤拷锟斤拷ETH_MDIO锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_MDIO_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Alternate = ETH_MDIO_AF;
    HAL_GPIO_Init(ETH_MDIO_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_MDC锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_MDC_PIN;
    GPIO_InitStructure.Alternate = ETH_MDC_AF;
    HAL_GPIO_Init(ETH_MDC_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_REF_CLK锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_REF_CLK_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_REF_CLK_AF;
    HAL_GPIO_Init(ETH_RMII_REF_CLK_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_CRS_DV锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_CRS_DV_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_CRS_DV_AF;
    HAL_GPIO_Init(ETH_RMII_CRS_DV_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_RXD0锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_RXD0_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_RXD0_AF;
    HAL_GPIO_Init(ETH_RMII_RXD0_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_RXD1锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_RXD1_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_RXD1_AF;
    HAL_GPIO_Init(ETH_RMII_RXD1_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_TX_EN锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_TX_EN_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_TX_EN_AF;
    HAL_GPIO_Init(ETH_RMII_TX_EN_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_TXD0锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_TXD0_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_TXD0_AF;
    HAL_GPIO_Init(ETH_RMII_TXD0_PORT, &GPIO_InitStructure);

    /* 锟斤拷锟斤拷ETH_RMII_TXD1锟斤拷锟斤拷 */
    GPIO_InitStructure.Pin = ETH_RMII_TXD1_PIN;
    GPIO_InitStructure.Alternate = ETH_RMII_TXD1_AF;
    HAL_GPIO_Init(ETH_RMII_TXD1_PORT, &GPIO_InitStructure);      
}  
/**
  * @brief  锟斤拷始锟斤拷ETH锟斤拷锟斤拷时锟接ｏ拷锟斤拷锟斤拷.
  * @param  heth: ETH handle
  * @retval None
  */  
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  ETH_GPIO_Config();
  
  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  
  /* 使锟斤拷锟斤拷太锟斤拷时锟斤拷  */
  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();    
}  

/**
  * @brief  锟斤拷始锟斤拷LAN8720A.
  * @param  heth: ETH handle
  * @retval HAL_StatusTypeDef锟斤拷状态值
  */  
HAL_StatusTypeDef LAN8720_Init(ETH_HandleTypeDef *heth)
{
    uint32_t phyreg = 0;
    uint32_t TIME_Out = 0;
    //锟斤拷锟斤拷锟轿籐AN8720A
    if(HAL_ETH_WritePHYRegister(heth, LAN8720A_PHY_ADDRESS, PHY_BCR, PHY_RESET) != HAL_OK)
    {
        return HAL_ERROR;
    }
    //锟饺达拷LAN8720A锟斤拷位锟斤拷锟?
    HAL_Delay(PHY_RESET_DELAY);    //    HAL_Delay(PHY_RESET_DELAY);   CRJ
    
    if((HAL_ETH_WritePHYRegister(heth, LAN8720A_PHY_ADDRESS, PHY_BCR, PHY_AUTONEGOTIATION)) != HAL_OK)
    {
      return HAL_ERROR;   
    }     
    //锟饺达拷LAN8720A写锟斤拷锟斤拷锟?
    HAL_Delay(0xF);
    do
    {     
      HAL_ETH_ReadPHYRegister(heth, LAN8720A_PHY_ADDRESS, PHY_BSR, &phyreg);
      TIME_Out++;
      if(TIME_Out > PHY_READ_TO) 
        return HAL_TIMEOUT;
    } while (((phyreg & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));
    
    return HAL_OK;    
}

/**
  * @brief  锟斤拷取LAN8720锟侥癸拷锟斤拷状态
  * @param  heth: ETH handle
  * @retval phyreg锟斤拷LAN8720锟斤拷SR锟侥达拷锟斤拷值
  */
uint32_t LAN8720_GetLinkState(ETH_HandleTypeDef *heth)
{
    uint32_t phyreg = 0;   
  
    if(HAL_ETH_ReadPHYRegister(heth, LAN8720A_PHY_ADDRESS, PHY_SR, &phyreg) == HAL_OK)
        return phyreg;
    return 0;

}


void ETH_IRQHandler(void)
{
	uint32_t ulReturn;
	/*进入临界段，临界段可以嵌套 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	HAL_ETH_IRQHandler(&EthHandle);

	/* 退出临界段*/
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
extern xSemaphoreHandle s_xSemaphore;
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	LED3_TOGGLE;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
  printf("HAL_ETH_TxCpltCallback\n");;
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    printf("eth err\n");
}


/*********************************************END OF FILE**********************/
