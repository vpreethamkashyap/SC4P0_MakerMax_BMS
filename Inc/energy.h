#ifndef ENERGY_H_
#define ENERGY_H_

static float simulatedAhStep = 0.1;
static float fullCellAh = 3;
static float simulatedSOC = 0;
static float energyAvailableDischarge = 0;
static float undervoltage = 2.8;

int performDischargeStep (float startBatt_mV);
void setupEnergyAvailableDischarge(float lastReadBattV);
int calculateEnergyAvailableDischarge();
float getEnergyAvaialbelDischarge();




#endif
