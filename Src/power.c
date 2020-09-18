#include "power.h"


float computePower(float cellV) //Assumption is that we are operating at nominal room temperatures
{
	//1Watt = Voltage (V) * Current (A)

	// Need to compute the number of A cell can discharge

	// Simple ohms law V = I*R

	float voltageDiff = cellV - uv_samsung25R;
	if(voltageDiff < 0){ //Quick check cellV should never be below UV
		voltageDiff = 0;
	}

	float current = voltageDiff/ (ir_samsung25R / 1000);
	//Eg: if 0.7 / 20m = 35A

	//Since our cell samsung25R can do 100A < 1s and 20A continous

	if(current > maxConstCellCurr_samsung25R)
	{
		current = maxConstCellCurr_samsung25R;
	}

	return cellV * current; //Watts


}
