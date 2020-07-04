/*
 * soc.c
 *
 *  Created on: Jul 4, 2020
 *      Author: preetham
 */

#include "sochelper.h"
#include "soc.h"

//Negative means charging current
//Positive means discharging current
/*
 * This function calculates the changing Ah for the cell
 * based on timeInterval in seconds and current in amps
 */
float calcdeltaAh(float timeInterval_s, float current_A){

	//Ah = A * hours
	float deltaAh = (-current_A * timeInterval_s / 3600.0);

	return deltaAh;
}

float socByOCV(float ocv){

	return lookupSOCByOCV(ocv, defaultOcvTable, defaultTableSize, defaultSocTable);
}


