#ifndef __LAN8720A_H
#define	__LAN8720A_H

#include "stm32h7xx.h"
/*
 ETH_MDIO -------------------------> PA2
 ETH_MDC --------------------------> PC1
 ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
 ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
 ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
 ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
 ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
 ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
 ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
 */
/* ETH_MDIO */
#define ETH_MDIO_GPIO_CLK_ENABLE()          __GPIOA_CLK_ENABLE()
#define ETH_MDIO_PORT                       GPIOA
#define ETH_MDIO_PIN                        GPIO_PIN_2
#define ETH_MDIO_AF                         GPIO_AF11_ETH

/* ETH_MDC */
#define ETH_MDC_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE();
#define ETH_MDC_PORT                        GPIOC
#define ETH_MDC_PIN                         GPIO_PIN_1
#define ETH_MDC_AF                          GPIO_AF11_ETH

/* ETH_RMII_REF_CLK */
#define ETH_RMII_REF_CLK_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE();
#define ETH_RMII_REF_CLK_PORT               GPIOA
#define ETH_RMII_REF_CLK_PIN                GPIO_PIN_1
#define ETH_RMII_REF_CLK_AF                 GPIO_AF11_ETH

/* ETH_RMII_CRS_DV */
#define ETH_RMII_CRS_DV_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE();
#define ETH_RMII_CRS_DV_PORT                GPIOA
#define ETH_RMII_CRS_DV_PIN                 GPIO_PIN_7
#define ETH_RMII_CRS_DV_AF                  GPIO_AF11_ETH

/* ETH_RMII_RXD0 */
#define ETH_RMII_RXD0_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE();
#define ETH_RMII_RXD0_PORT                  GPIOC
#define ETH_RMII_RXD0_PIN                   GPIO_PIN_4
#define ETH_RMII_RXD0_AF                    GPIO_AF11_ETH

/* ETH_RMII_RXD1 */
#define ETH_RMII_RXD1_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE();
#define ETH_RMII_RXD1_PORT                  GPIOC
#define ETH_RMII_RXD1_PIN                   GPIO_PIN_5
#define ETH_RMII_RXD1_AF                    GPIO_AF11_ETH

/* ETH_RMII_TX_EN */
#define ETH_RMII_TX_EN_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE();
#define ETH_RMII_TX_EN_PORT                 GPIOB
#define ETH_RMII_TX_EN_PIN                  GPIO_PIN_11
#define ETH_RMII_TX_EN_AF                   GPIO_AF11_ETH

/* 轻载 ETH_RMII_TXD0 */
#define ETH_RMII_TXD0_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE();
#define ETH_RMII_TXD0_PORT                  GPIOB
#define ETH_RMII_TXD0_PIN                   GPIO_PIN_12
#define ETH_RMII_TXD0_AF                    GPIO_AF11_ETH

/* ETH_RMII_TXD1 */
#define ETH_RMII_TXD1_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE();
#define ETH_RMII_TXD1_PORT                  GPIOB
#define ETH_RMII_TXD1_PIN                   GPIO_PIN_13
#define ETH_RMII_TXD1_AF                    GPIO_AF11_ETH

/* ETH_RMII_TXD0 野火的引脚 */
//#define ETH_RMII_TXD0_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
//#define ETH_RMII_TXD0_PORT                  GPIOG
//#define ETH_RMII_TXD0_PIN                   GPIO_PIN_13
//#define ETH_RMII_TXD0_AF                    GPIO_AF11_ETH
//
///* ETH_RMII_TXD1 */
//#define ETH_RMII_TXD1_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
//#define ETH_RMII_TXD1_PORT                  GPIOG
//#define ETH_RMII_TXD1_PIN                   GPIO_PIN_14
//#define ETH_RMII_TXD1_AF                    GPIO_AF11_ETH


/* ETH_RMII_TX_EN */
//#define ETH_RMII_TX_EN_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE();
//#define ETH_RMII_TX_EN_PORT                 GPIOG
//#define ETH_RMII_TX_EN_PIN                  GPIO_PIN_11
//#define ETH_RMII_TX_EN_AF                   GPIO_AF11_ETH

/* ETH_RMII_TXD0 */
//#define ETH_RMII_TXD0_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
//#define ETH_RMII_TXD0_PORT                  GPIOG
//#define ETH_RMII_TXD0_PIN                   GPIO_PIN_12
//#define ETH_RMII_TXD0_AF                    GPIO_AF11_ETH

/* ETH_RMII_TXD1 */
//#define ETH_RMII_TXD1_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
//#define ETH_RMII_TXD1_PORT                  GPIOG
//#define ETH_RMII_TXD1_PIN                   GPIO_PIN_13
//#define ETH_RMII_TXD1_AF                    GPIO_AF11_ETH


//MAC锟斤拷址
#define MAC_ADDR0   0x02U
#define MAC_ADDR1   0x00U
#define MAC_ADDR2   0x00U
#define MAC_ADDR3   0x00U
#define MAC_ADDR4   0x00U
#define MAC_ADDR5   0x00U

#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE //锟斤拷锟斤拷锟斤拷锟捷的筹拷锟斤拷
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE //锟斤拷锟斤拷锟斤拷锟捷的筹拷锟斤拷
#define ETH_RXBUFNB                    ((uint32_t)4)       //要锟斤拷锟斤拷锟斤拷锟捷的革拷锟斤拷
#define ETH_TXBUFNB                    ((uint32_t)4)       //要锟斤拷锟斤拷锟斤拷锟捷的革拷锟斤拷
//LAN8720A锟侥寄达拷锟斤拷锟疥定锟斤拷 
//PHY锟斤拷锟斤拷锟斤拷锟街?
#define LAN8720A_PHY_ADDRESS            0x00
//PHY锟斤拷时时锟斤拷
#define PHY_RESET_DELAY                 ((uint32_t)0x00000F)
#define PHY_CONFIG_DELAY                ((uint32_t)0x00000F)
//锟斤拷写PHY锟侥等达拷时锟斤拷
#define PHY_READ_TO                     ((uint32_t)0x0000FFFF)
#define PHY_WRITE_TO                    ((uint32_t)0x0000FFFF)

//PHY锟斤拷锟斤拷锟侥达拷锟斤拷
#define PHY_BCR                         ((uint16_t)0x00)   //R0--锟斤拷锟斤拷锟斤拷锟狡寄达拷锟斤拷
#define PHY_BSR                         ((uint16_t)0x01)   //R1--锟斤拷锟斤拷状态锟侥达拷锟斤拷
//PHY锟斤拷锟斤拷锟斤拷锟狡寄达拷锟斤拷锟斤拷锟斤拷锟斤拷
#define PHY_RESET                       ((uint16_t)0x8000)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002)  /*!< Jabber condition detected            */

#define PHY_SPEED_Indication            ((uint16_t)0x001C)

#define LAN8740_10MBITS_HALFDUPLEX      ((uint16_t)0x0004)
#define LAN8740_10MBITS_FULLDUPLEX      ((uint16_t)0x0014)
#define LAN8740_100MBITS_HALFDUPLEX     ((uint16_t)0x0008)
#define LAN8740_100MBITS_FULLDUPLEX     ((uint16_t)0x0018)

//PHY锟斤拷锟斤拷锟斤拷锟?状态锟侥达拷锟斤拷
#define PHY_SR                          ((uint16_t)0x1F)    /*!< PHY special control/ status register Offset     */

#define PHY_SPEED_STATUS                ((uint16_t)0x0004)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0010)  /*!< PHY Duplex mask                                 */

#define PHY_ISFR                        ((uint16_t)0x1D)    /*!< PHY Interrupt Source Flag register Offset       */
#define PHY_ISFR_INT4                   ((uint16_t)0x0010)  /*!< PHY Link down inturrupt */

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef LAN8720_Init(ETH_HandleTypeDef *heth);
uint32_t LAN8720_GetLinkState(ETH_HandleTypeDef *heth);
#endif /* __LAN8720A_H */
