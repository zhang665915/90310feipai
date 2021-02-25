/*
 * tjshow.h
 *
 *  Created on: 2020年10月13日
 *      Author: lyh
 */

#ifndef INC_TJSHOW_H_
#define INC_TJSHOW_H_


#define	SIZEOF_TJC_RxData 7	//接收串口屏数据长度


/* 接收串口屏的频率设置值 */
typedef union
{
	int8_t data[4];
	int freq ;
}Freq_Data;
Freq_Data FreqData ;

//extern uint8_t Set_Freq_Flag;
extern uint8_t TJC_RxData[SIZEOF_TJC_RxData];
void TJC_Show( float complex_V );
void FreqHandle(void);
void Tjshow_Task(void *parameter);
void HMISend(uint8_t k);
void TJC_Init(void);


#endif /* INC_TJSHOW_H_ */
