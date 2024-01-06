/*
 * CAN_Handle.c
 *
 *  Created on: Dec 22, 2023
 *      Author: win 10
 */
#include "main.h"
#include "can.h"
#include "CAN_Handle.h"
CAN_RxHeaderTypeDef Rxheader;
volatile uint8_t rcdata[8];
volatile uint8_t Can_RecFlag=0;

void CAN_HandleSenđata(const uint32_t arbitration_id,
                           const uint8_t* data, const uint8_t size) {
	CAN_TxHeaderTypeDef Txheader;
	uint32_t TxMailbox;
	Txheader.DLC=size;
	Txheader.IDE= CAN_ID_STD;
	Txheader.RTR= CAN_RTR_DATA;
	Txheader.StdId= arbitration_id;
	if(HAL_CAN_AddTxMessage(&hcan, &Txheader, data, &TxMailbox)!=HAL_OK){
		Error_Handler();
	}
}
void CAN_Config_filtering(void)
{
	CAN_FilterTypeDef Can_filter_init;
	Can_filter_init.FilterActivation=ENABLE;
	Can_filter_init.FilterBank=0;
	Can_filter_init.FilterFIFOAssignment=CAN_RX_FIFO0;
	Can_filter_init.FilterIdHigh=0x0000;
	Can_filter_init.FilterIdLow= 0x0000;
	Can_filter_init.FilterMaskIdHigh= 0x0000;
	Can_filter_init.FilterMaskIdLow= 0x0000;
	Can_filter_init.FilterMode=CAN_FILTERMODE_IDMASK;
	Can_filter_init.FilterScale=CAN_FILTERSCALE_32BIT;
	if(HAL_CAN_ConfigFilter(&hcan,&Can_filter_init)!=HAL_OK)
	{
		Error_Handler();
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader, rcdata)==HAL_OK){
		Can_RecFlag=1;
	}
	else
	{
		Can_RecFlag=0;
	}
}
