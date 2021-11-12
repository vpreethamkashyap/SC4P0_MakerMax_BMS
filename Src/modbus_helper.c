/*
 * modbus_helper.c
 *
 *  Created on: Dec 24, 2020
 *      Author: preetham
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "mb.h"
#include "shared_values.h"
#include "modbus_helper.h"


#define SLAVE_ADDR		0x0A
#define BAUDRATE		38400

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 64
#define REG_HOLDING_START 2000
#define REG_HOLDING_NREGS 8

osThreadId modbusTaskHandle;
uint32_t modbusTaskBuffer[ 1024 ];
osStaticThreadDef_t modbusTaskControlBlock;

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

typedef union
{
  float     asFloat;
  uint32_t  asUInt32;
}modbusData_t;

modbusData_t powerData;
modbusData_t socData;
modbusData_t battVData;
modbusData_t currentData;
modbusData_t energyData;


void enable_modbusrtu(void)
{

	eMBErrorCode    eStatus;

	eStatus = eMBInit( MB_RTU, SLAVE_ADDR, 0, BAUDRATE, MB_PAR_NONE );

	/* Enable the Modbus Protocol Stack. */
	eStatus = eMBEnable();

	if(eStatus != MB_ENOERR){
		printf("modbus init error %d", eStatus);
	}
}

void updateModbusInputRegisters()
{
	powerData.asFloat = lastComputedPower;
	socData.asFloat = currentCellSOC;
	currentData.asFloat = lastReadCurr_mA;
	battVData.asFloat = lastReadBattV;
	energyData.asFloat = lastComputedEnergy;

	usRegInputBuf[0] = currentState;
	usRegInputBuf[1] = powerData.asUInt32 >> 16;
	usRegInputBuf[2] = powerData.asUInt32;
	usRegInputBuf[3] = socData.asUInt32 >> 16;
	usRegInputBuf[4] = socData.asUInt32;
	usRegInputBuf[5] = currentData.asUInt32 >> 16;
	usRegInputBuf[6] = currentData.asUInt32;
	usRegInputBuf[7] = battVData.asUInt32 >> 16;
	usRegInputBuf[8] = battVData.asUInt32;
	usRegInputBuf[9] = energyData.asUInt32 >> 16;
	usRegInputBuf[10] = energyData.asUInt32;
}

void modbus_task_init(void)
{
	osThreadStaticDef(modbusTask, modbus_task, osPriorityNormal, 0, 1024, modbusTaskBuffer, &modbusTaskControlBlock);
	modbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);
}

//modbus freertos task
void modbus_task(void)
{

	for(;;)
	{
		//printf("In modbus task");
		updateModbusInputRegisters();
		//Modbus Poll routine
		eMBPoll();

	    //osDelay(1);
		taskYIELD();
	}
}

//16 bit input register data type
//NOTE: "mbpoll -m rtu -a 10 -r 1000 -c 8 -t 3 -b 38400 -d 8 -P NONE /dev/ttyUSB0"
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

//mbpoll -m rtu -a 10 -r 2000 -t 4 -b 38400 -d 8 -P NONE /dev/ttyUSB0 4 5 6
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

