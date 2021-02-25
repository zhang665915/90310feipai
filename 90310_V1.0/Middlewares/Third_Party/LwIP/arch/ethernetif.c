/**
 ******************************************************************************
 * @file    LwIP/LwIP_UDP_Echo_Client/Src/ethernetif.c
 * @author  MCD Application Team
 * @version V1.2.0
 * @date    29-December-2017
 * @brief   This file implements Ethernet network interface drivers for lwIP
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "LAN8720a.h" 
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_RX_BUFFER_SIZE                     (1536UL)

#define ETH_DMA_TRANSMIT_TIMEOUT               (0U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 
 @Note: This interface is implemented to operate in zero-copy mode only:
 - Rx buffers are allocated statically and passed directly to the LwIP stack
 they will return back to DMA after been processed by the stack.
 - Tx Buffers will be allocated from LwIP stack memory heap, 
 then passed to ETH HAL driver.

 @Notes: 
 1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4, 
 to customize it please redefine ETH_RX_DESC_CNT in stm32xxxx_hal_conf.h
 1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4, 
 to customize it please redefine ETH_TX_DESC_CNT in stm32xxxx_hal_conf.h

 2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
 2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
 passed to ETH DMA in the init field (EthHandle.Init.RxBuffLen)
 */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

//以太网描述符和缓冲区
//ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".ARM.__at_0x30040000")));
//ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".ARM.__at_0x30040060")));
//uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".ARM.__at_0x30040200")));
//__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];      //以太网Rx DMA描述符
//__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];      //以太网Tx DMA描述符
//__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE];  //以太网接收缓冲区
//__attribute__((section(".ARM.__at_0x30040000"))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
//__attribute__((section(".ARM.__at_0x30040060"))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
//__attribute__((section(".ARM.__at_0x30040200"))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */

//struct pbuf_custom rx_pbuf[ETH_RX_DESC_CNT];
uint32_t current_pbuf_idx = 0;

ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig;
ETH_MACConfigTypeDef MACConfig;

xSemaphoreHandle s_xSemaphore = NULL;

sys_sem_t tx_sem = NULL;
sys_mbox_t eth_tx_mb = NULL;

LWIP_MEMPOOL_DECLARE(RX_POOL, 20, sizeof(struct pbuf_custom),"Zero-copy RX PBUF pool");

/* Private function prototypes -----------------------------------------------*/
u32_t sys_now(void);
static void arp_timer(void *arg);
void pbuf_free_custom(struct pbuf *p);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 LL Driver Interface ( LwIP stack --> ETH) 
 *******************************************************************************/
/**
 * @brief 在这个函数中初始化硬件.
 *        最终被ethernetif_init函数调用.
 *
 * @param netif已经初始化了这个以太网的lwip网络接口结构       
 */
static void low_level_init(struct netif *netif)
{
    printf("调用函数：low_level_init！\n");
    uint32_t idx = 0;
    //mac地址
    uint8_t macaddress[6] =
    { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };
    
    EthHandle.Instance = ETH;
    EthHandle.Init.MACAddr = macaddress;
    //RMII模式
    EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    //接受描述符
    EthHandle.Init.RxDesc = DMARxDscrTab;
    //发送描述符
    EthHandle.Init.TxDesc = DMATxDscrTab;
    //数据长度
    EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;
    
    /* 配置以太网外设 (GPIOs, clocks, MAC, DMA)*/
    if (HAL_ETH_Init(&EthHandle) == HAL_ERROR)
        printf("HAL_ETH_Init ERROR");
    
    /* 设置netif MAC 硬件地址长度 */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    
    /* 设置netif MAC 硬件地址 */
    netif->hwaddr[0] = MAC_ADDR0;
    netif->hwaddr[1] = MAC_ADDR1;
    netif->hwaddr[2] = MAC_ADDR2;
    netif->hwaddr[3] = MAC_ADDR3;
    netif->hwaddr[4] = MAC_ADDR4;
    netif->hwaddr[5] = MAC_ADDR5;
    
    /* 设置netif最大传输单位 */
    netif->mtu = ETH_MAX_PAYLOAD;
    
    /* 接收广播地址和ARP流量  */
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    
    for (idx = 0; idx < ETH_RX_DESC_CNT; idx++)
    {
        HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
        
        /* Set Custom pbuf free function */
//    rx_pbuf[idx].custom_free_function = pbuf_free_custom;    //2019.11.26  CRJ
    }
    
    s_xSemaphore = xSemaphoreCreateCounting(40, 0);
    
    if (sys_sem_new(&tx_sem, 0) == ERR_OK)
//        PRINT_DEBUG("sys_sem_new ok\n");
    
    if (sys_mbox_new(&eth_tx_mb, 50) == ERR_OK)
//        PRINT_DEBUG("sys_mbox_new ok\n");
    
    /* create the task that handles the ETH_MAC */
    sys_thread_new("ETHIN", ethernetif_input, /* 任务入口函数 */
    netif, /* 任务入口函数参数 */
    NETIF_IN_TASK_STACK_SIZE,/* 任务栈大小 */
    NETIF_IN_TASK_PRIORITY); /* 任务的优先级 */
    
    LWIP_MEMPOOL_INIT(RX_POOL);
    
    /* 设置发送配置结构体 */
    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM
            | ETH_TX_PACKETS_FEATURES_CRCPAD;
    //发送校验
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    //CRC校验位
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    //初始化LAN8720A
    if (LAN8720_Init(&EthHandle) == HAL_OK)
    {
        printf("LAN8720_Init ok\n");
        ethernet_link_check_state(netif);
    }
}

/**
 * @brief This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
//  printf("调用网卡发送函数：low_level_output\n");
    uint32_t i = 0, framelen = 0;
    struct pbuf *q;
    err_t errval = ERR_OK;
//    uint8_t CRJ = 0;
    
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
    
    static sys_sem_t ousem = NULL;			//静态创建数据包发送同步信号量
    if (ousem == NULL)
    {
        sys_sem_new(&ousem, 0);
        sys_sem_signal(&ousem);
    }
    
    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef)); //根据发送描述符的大小和个数创建发送缓冲区  CRJ——2019.11.26
            
    sys_sem_wait(&ousem);//CRJ 阻塞等待信号量，只有发送完前一个数据包才能开始本次数据包发送	CRJ——2019.11.26
            
    for (q = p; q != NULL; q = q->next) //将上层封装完的数据包（包含其他层的包头，存于pbuf中）存在发送缓冲区中	CRJ——2019.11.26
    {
        if (i >= ETH_TX_DESC_CNT)
            return ERR_IF;
        
        Txbuffer[i].buffer = q->payload;
        Txbuffer[i].len = q->len;
        framelen += q->len;
        
        if (i > 0)
        {
            Txbuffer[i - 1].next = &Txbuffer[i];
        }
        
        if (q->next == NULL)
        {
            Txbuffer[i].next = NULL;
        }
        
        i++;
    }
    
    TxConfig.Length = framelen;
    TxConfig.TxBuffer = Txbuffer;
    
//     printf("以太网数据包的大小为：%d\n", TxConfig.Length);     // CRJ——2019.11.26
    
    /* Clean and Invalidate data cache */
    SCB_CleanInvalidateDCache();		//野火的例程把这一句注释了，但是移植后数据包发送卡死，打开后可以发送数据包
    
    HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT); //调用底层以太网专属的DMA发送函数（真正执行发送数据包）  CRJ——2019.11.26
            
//    printf("调用以太网DMA发送函数的返回值为：%d\n",CRJ);
    
    sys_sem_signal(&ousem);  //发送完一个数据包，释放信号量  CRJ——2019.11.26
            
    return errval;
    
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf* low_level_input(struct netif *netif)
{
//	 printf("网卡接收函数low_level_input：外部——>内核！\n");
    struct pbuf *p = NULL;
    ETH_BufferTypeDef RxBuff;
    uint32_t framelength = 0;
    
    struct pbuf_custom *custom_pbuf;
    
    /* Clean and Invalidate data cache */
    SCB_CleanInvalidateDCache();
    
    if (HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff) == HAL_OK)
    {
        HAL_ETH_GetRxDataLength(&EthHandle, &framelength);
        
        /* Invalidate data cache for ETH Rx Buffers */
        SCB_InvalidateDCache_by_Addr((uint32_t*) Rx_Buff,
                (ETH_RX_DESC_CNT * ETH_RX_BUFFER_SIZE));
        
        custom_pbuf = (struct pbuf_custom*) LWIP_MEMPOOL_ALLOC(RX_POOL);
        custom_pbuf->custom_free_function = pbuf_free_custom;
        
        p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf,
                RxBuff.buffer, ETH_RX_BUFFER_SIZE);
        
    }
    
    return p;
}

/**
 * @brief 当数据包准备好从接口读取时，应该调用此函数。
 *它使用应该处理来自网络接口的字节的实际接收的函数low_level_input。
 *然后确定接收到的分组的类型，并调用适当的输入功能。
 *
 * @param netif 以太网的lwip网络接口结构
 */
void ethernetif_input(void *pParams)
{
//	printf("调用函数：ethernetif_input");
    struct netif *netif;
    struct pbuf *p = NULL;
    netif = (struct netif*) pParams;
//    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n")); PRINT_DEBUG("ethernetif_input running\n");
    while (1)
    {
        if (xSemaphoreTake( s_xSemaphore, portMAX_DELAY ) == pdTRUE)
        {
            /* move received packet into a new pbuf */
            taskENTER_CRITICAL();
            TRY_GET_NEXT_FRAGMENT: p = low_level_input(netif);
            
            /* Build Rx descriptor to be ready for next data reception */
            HAL_ETH_BuildRxDescriptors(&EthHandle);
            
            taskEXIT_CRITICAL();
            /* points to packet payload, which starts with an Ethernet header */
            if (p != NULL)
            {
                taskENTER_CRITICAL();
                if (netif->input(p, netif) != ERR_OK)
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                    pbuf_free(p);
                    p = NULL;
                }
                else
                {
                    xSemaphoreTake(s_xSemaphore, 0);
                    goto TRY_GET_NEXT_FRAGMENT;
                }
                taskEXIT_CRITICAL();
            }
        }
    }
}

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
//	printf("挂载网卡与初始化！\n");
    struct ethernetif *ethernetif;
    
//	LWIP_ASSERT("netif != NULL", (netif != NULL));
    
    ethernetif = mem_malloc(sizeof(ethernetif));
    
    if (ethernetif == NULL)
    {
//        PRINT_ERR("ethernetif_init: out of memory\n");
        return ERR_MEM;
    }
    
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */
    netif->state = ethernetif;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;
    
    /* initialize the hardware */
    low_level_init(netif);
    
//  ethernetif->ethaddr = (struct eth_addr *) &(netif->hwaddr[0]);
    
    return ERR_OK;
}

/**
 * @brief  Custom Rx pbuf free callback
 * @param  pbuf: pbuf to be freed
 * @retval None
 */
void pbuf_free_custom(struct pbuf *p)
{
    struct pbuf_custom *custom_pbuf = (struct pbuf_custom*) p;
    /* invalidate data cache: lwIP and/or application may have written into buffer */
    SCB_InvalidateDCache_by_Addr((uint32_t*) p->payload, p->tot_len);
    LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}

/**
 * @brief  Returns the current time in milliseconds
 *         when LWIP_TIMERS == 1 and NO_SYS == 1
 * @param  None
 * @retval Current Time value
 */
//u32_t sys_now(void)
//{
//  return HAL_GetTick();
//}

/**
 * @brief  ethernet_link_check_state
 * @param  netif
 * @retval None
 */
void ethernet_link_check_state(struct netif *netif)
{
    
    uint32_t PHYLinkState;
    uint32_t linkchanged = 0, speed = 0, duplex = 0;
    
    PHYLinkState = LAN8720_GetLinkState(&EthHandle);
    
    if (netif_is_link_up(netif) && (PHYLinkState))
    {
        HAL_ETH_Stop_IT(&EthHandle);
        // HAL_ETH_Stop(&EthHandle);
        netif_set_down(netif);
        netif_set_link_down(netif);
    }
    else if (!netif_is_link_up(netif) && (PHYLinkState))
    {
        switch ((PHYLinkState & PHY_SPEED_Indication))
        {
            case LAN8740_100MBITS_FULLDUPLEX:
            {
                duplex = ETH_FULLDUPLEX_MODE;
                speed = ETH_SPEED_100M;
                linkchanged = 1;
                break;
            }
            case LAN8740_100MBITS_HALFDUPLEX:
            {
                duplex = ETH_HALFDUPLEX_MODE;
                speed = ETH_SPEED_100M;
                linkchanged = 1;
                break;
            }
            case LAN8740_10MBITS_FULLDUPLEX:
            {
                duplex = ETH_FULLDUPLEX_MODE;
                speed = ETH_SPEED_10M;
                linkchanged = 1;
                break;
            }
            case LAN8740_10MBITS_HALFDUPLEX:
            {
                duplex = ETH_HALFDUPLEX_MODE;
                speed = ETH_SPEED_10M;
                linkchanged = 1;
                break;
            }
            default:
                break;
        }
        
        if (linkchanged)
        {
            /* Get MAC Config MAC */
            HAL_ETH_GetMACConfig(&EthHandle, &MACConfig);
            MACConfig.DuplexMode = duplex;
            MACConfig.Speed = speed;
            HAL_ETH_SetMACConfig(&EthHandle, &MACConfig);
            HAL_ETH_Start_IT(&EthHandle);
            netif_set_up(netif);
            netif_set_link_up(netif);
        }
    }
}

static void arp_timer(void *arg)
{
    etharp_tmr();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
