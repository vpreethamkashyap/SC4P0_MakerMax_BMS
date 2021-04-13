/*
 * bms.h
 *
 *  Created on: Apr 10, 2021
 *      Author: preetham
 */

#ifndef BMS_H_
#define BMS_H_

extern char powerString[10];
extern char socString[10];
extern char currentString[10];
extern char battVString[10];

extern const float fullChargeCapacity; //Ah //TODO: change this as per your cell
extern float lastReadBattV;
extern float lastReadCurr_mA;
extern float currentCellSOC;
extern float currChargeRemaining; 		 //Ah how much charge is remaining inside the cell
extern float lastComputedPower;
extern float lastComputedEnergy;		 //Wh

void InitBMS ( void );
void bms_task_init(void);
void bms_task ( void );

void Charging_Enable(int chg_en);
void Change_State(int new_state);
void Discharging_Set(uint8_t pct);
void Safety_Loop();
void getLatestADCValues();
void updateOLED();
void updateSerialPort();
void calcSOC(float ocv_V, float chargeRemain_Ah);
int sprintf(char *out, const char *format, ...);


#endif /* BMS_H_ */
