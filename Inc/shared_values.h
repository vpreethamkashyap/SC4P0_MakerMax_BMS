#ifndef SHARED_VALUES_H
#define SHARED_VALUES_H

typedef enum
{
	IDLE = 0,
	CHG = 1,
	DCHG = 2
}STATE;

extern STATE currentState;
extern float lastReadBattV;
extern float lastReadCurr_mA;
extern float currentCellSOC;
extern float currChargeRemaining; 	//Ah how much charge is remaining inside the cell
extern float lastComputedPower;
extern float lastComputedEnergy;	//Wh

#endif
