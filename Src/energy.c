/*
 * energy.c
 *
 *  Created on: Sep 15, 2020
 *      Author: preetham
 */

#include "energy.h"
#include "sochelper.h"

int performDischargeStep (float currentSOC){

	//Perform a simulated discharge
	float newCellAh = ((currentSOC/100) * fullCellAh) - simulatedAhStep; 		//Ah
	simulatedSOC = (newCellAh / fullCellAh) * 100.0;							//%
	float newBattV = lookupSOCByOCV(simulatedSOC, &defaultOcvTable, defaultTableSize, &defaultSocTable);	//V

	if(newBattV <= undervoltage){
		return 0;
	}

	//Calculate energy consumed - V*I*t
	float energy = newBattV*simulatedAhStep;	//Wh

	//Aggregate the energy
	energyAvailableDischarge += energy;

	return 1;
}

//This function is called once when setting up a new iteration of the energy available calculation
void setupEnergyAvailableDischarge(float soc){

	energyAvailableDischarge = 0;
	performDischargeStep(soc);
}

//Recursive call to find the energyAvailable for discharge
//When caller receives a return value of 1, it can read the energyAvailableDischarge
int calculateEnergyAvailableDischarge(){

	//Exit condition
	if(performDischargeStep(simulatedSOC) == 0)
		return 1;

	//Recursive call
	calculateEnergyAvailableDischarge();

}

float getEnergyAvaialbelDischarge(){

	return energyAvailableDischarge;

}

