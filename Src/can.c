/*
 * can.c
 *
 *  Created on: Apr 10, 2021
 *      Author: preetham
 */

#include "main.h"
#include "cmsis_os.h"
#include "can.h"

osThreadId canTaskHandle;
uint32_t canTaskBuffer[ 1024 ];
osStaticThreadDef_t canTaskControlBlock;

//Tx and Rx Message structure/typdef
static CAN_RxHeaderTypeDef RxHeader;
static CAN_TxHeaderTypeDef TxHeader;

uint8_t               TxData[8];
uint8_t               RxData[8];

uint32_t              TxMailbox;

uint16_t OwnID = 0x456;
uint16_t RemoteID = 0x123;

uint8_t myBMSState = 0;


void InitCAN (void)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING| CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
	{
	  Error_Handler();
	}

	TxHeader.StdId = OwnID;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;
}

void can_task_init (void)
{
	osThreadStaticDef(canTask, can_task, osPriorityNormal, 0, 1024, canTaskBuffer, &canTaskControlBlock);
    canTaskHandle = osThreadCreate(osThread(canTask), NULL);
}


void can_task (void)
{

	for(;;)
	{
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
		{
			if(myBMSState == 4) myBMSState=0;

			//Transmit CAN packet
			TxData[0] = myBMSState++;
			TxData[1] = 0xFF;

			/* Request transmission */
			if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
			   /* Transmission request Error */
			   Error_Handler();
			}

			/* Wait transmission complete */
		    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}
		}

		taskYIELD();
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	if(RxHeader.StdId == RemoteID)
	{
		myBMSState = RxData[0];
	}
}
