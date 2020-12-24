#ifndef ENERGY_H_
#define ENERGY_H_

static float simulatedAhStep = 0.1;		//Ah
static float fullCellAh = 3;			//Ah
static float simulatedSOC = 0;			//V
static float energyAvailableDischarge = 0; //Wh
static float undervoltage = 2.8;		//V

int performDischargeStep (float currentSOC);
void setupEnergyAvailableDischarge(float soc);
int calculateEnergyAvailableDischarge();
float getEnergyAvaialbelDischarge();




#endif
