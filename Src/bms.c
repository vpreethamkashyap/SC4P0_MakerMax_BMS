
#include "main.h"
#include "cmsis_os.h"

#include "oled.h"
#include "adc.h"
#include "soc.h"
#include "sochelper.h"
#include "power.h"
#include "energy.h"
#include "bms.h"

/*
 * Task parameters
 */
osThreadId bmsTaskHandle;
uint32_t bmsTaskBuffer[ 1024 ];
osStaticThreadDef_t bmsTaskControlBlock;

typedef enum
{
	CHG_ENABLE = 1,
	CHG_DISABLE = 0
}CHG_EN;

typedef enum
{
	IDLE = 0,
	CHG = 1,
	DCHG = 2
}STATE;

STATE currentState = IDLE;
uint8_t currentDchgPct = 0;

char powerString[10];
char socString[10];
char currentString[10];
char battVString[10];

const float fullChargeCapacity = 3.0; //Ah //TODO: change this as per your cell
float lastReadBattV = 0;
float lastReadCurr_mA = 0;
float currentCellSOC = 0;
float currChargeRemaining = 0; 		 //Ah how much charge is remaining inside the cell
float lastComputedPower = 0;
float lastComputedEnergy = 0;		 //Wh

extern uint8_t myBMSState;

void bms_task_init(void)
{
	osThreadStaticDef(bmsTask, bms_task, osPriorityNormal, 0, 1024, bmsTaskBuffer, &bmsTaskControlBlock);
	bmsTaskHandle = osThreadCreate(osThread(bmsTask), NULL);
}

void bms_task ( void )
{
	for(;;)
	{
		getLatestADCValues();
		calcSOC(lastReadBattV, currChargeRemaining);
		lastComputedPower = computePower(lastReadBattV);

		setupEnergyAvailableDischarge(currentCellSOC);

		if(calculateEnergyAvailableDischarge() == 1)
		  lastComputedEnergy = getEnergyAvaialbelDischarge();

		updateOLED();
		//osDelay(1);
		taskYIELD();
	}
}


void InitBMS ( void )
{

	//Initialize OLED display
	ssd1306_Init();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(20,04);
	ssd1306_WriteString("BMS", Font_16x26, White);
	ssd1306_UpdateScreen();

	//Initial ADC
	LTC2990_ConfigureControlReg(&hi2c1);
}

/*
 *
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		HAL_GPIO_TogglePin(LED_USR1_GPIO_Port, LED_USR1_Pin); //LED1 toggles every 0.5 seconds

		//If both buttons S1 and S1 pressed
		if(HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			if(currentState == IDLE)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_SET);
				Change_State(CHG);
			}
			else if(currentState == CHG)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_SET);
				Change_State(DCHG);
			}
			else if(currentState == DCHG)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_RESET);
				Change_State(IDLE);
			}
			else
			{//This should not trigger
				Change_State(IDLE);
			}
		}
		//If either S2 or S1 is pressed while in DCHG state, then change discharge current
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			uint8_t newDchgPct = currentDchgPct + 10;
			//Increase discharge current
			Discharging_Set(newDchgPct);
		}
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			uint8_t newDchgPct = currentDchgPct - 10;
			//Decrease discharge current
			Discharging_Set(newDchgPct);
		}

		HAL_TIM_Base_Stop_IT(&htim6);
	}
	/*else if(htim == &htim7){ //Used for modbus sorry!!!
		//Safety_Loop(); //Safety loop run every 0.5 seconds
	}*/
	else if(htim == &htim16) //This timer ticks every one second and is used for charge remaning calculation
	{
		if(currentState == DCHG || currentState == CHG){
			// SUpply charge remaning with updated current
			currChargeRemaining += calcdeltaAh(1, lastReadCurr_mA / 1000.0);
		}
	}
	else if(htim == &htim17)
	{
		// This timer ticks every 10 seconds and is used for polarization calculations
		//If state is idle
		//Start counting rest time
	}
}

void getLatestADCValues()
{

	float battV_2 = 0;
	float voltageADCVcc = 0;

	//Trigger a new conversion
	LTC2990_Trigger(&hi2c1);
	LTC2990_WaitForConversion(&hi2c1, 100);

	//Quick ADC test - Read Vcc
	LTC2990_ReadVoltage(&hi2c1, VCC, &voltageADCVcc);

	//Current reading
	LTC2990_ReadVoltage(&hi2c1, BATTV, &lastReadBattV);
	LTC2990_ReadVoltage(&hi2c1, BATTV_2, &battV_2);
    LTC2990_ReadCurrent(&hi2c1, lastReadBattV, battV_2, &lastReadCurr_mA);
}


void updateOLED()
{

	//State
	ssd1306_SetCursor(15,04);
	ssd1306_WriteString("State  ", Font_7x10, White);
	ssd1306_SetCursor(52,04);
	if(currentState == IDLE){
		ssd1306_WriteString("IDLE", Font_7x10, White);
	} else if(currentState == CHG){
		ssd1306_WriteString("CHG", Font_7x10, White);
	} else if(currentState == DCHG){
		ssd1306_WriteString("DCHG", Font_7x10, White);
	} else {
		ssd1306_WriteString(" ", Font_7x10, White);
	}

	//Power
	sprintf(powerString, "%.2d W", (int)lastComputedPower);
	ssd1306_SetCursor(15,14);
	ssd1306_WriteString("Pwr   ", Font_7x10, White);
	ssd1306_SetCursor(52,14);
	ssd1306_WriteString(powerString, Font_7x10, White);

	//SOC
	sprintf(socString, "%.2d ", (int)currentCellSOC);
	ssd1306_SetCursor(15,25);
	ssd1306_WriteString("SOC  ", Font_7x10, White);
	ssd1306_SetCursor(52,25);
	ssd1306_WriteString(socString, Font_7x10, White);


	//Current
	sprintf(currentString, "%.3d mA", (int)lastReadCurr_mA);
	ssd1306_SetCursor(15,37);
	ssd1306_WriteString("Cur  ", Font_7x10, White);
	ssd1306_SetCursor(52,37);
	ssd1306_WriteString(currentString, Font_7x10, White);

	//Voltage
	sprintf(battVString, "%.3d V", (int)lastReadBattV);
	ssd1306_SetCursor(15,49);
	ssd1306_WriteString("BattV   ", Font_7x10, White);
	ssd1306_SetCursor(52,49);
	ssd1306_WriteString(battVString, Font_7x10, White);

	ssd1306_UpdateScreen();

}

void updateSerialPort()
{
	//state
	if(currentState == IDLE){
		HAL_UART_Transmit(&huart1, (uint8_t*)"IDLE", 4, 10);
	} else if(currentState == CHG){
		HAL_UART_Transmit(&huart1, (uint8_t*)"CHG", 3, 10);
	} else if(currentState == DCHG){
		HAL_UART_Transmit(&huart1, (uint8_t*)"DCHG", 4, 10);
	} else {
		HAL_UART_Transmit(&huart1, (uint8_t*)" ", 1, 10);
	}

	//Power
	char powerString[10];
	sprintf(powerString, "%.2d W", (int)lastComputedPower);
	HAL_UART_Transmit(&huart1, (uint8_t*)powerString , 10, 10);

	//SOC
	char socString[10];
	sprintf(socString, "%.2d",(int)currentCellSOC);
	HAL_UART_Transmit(&huart1, (uint8_t*)socString , 10, 10);

	//Current
	char currentString[10];
	sprintf(currentString, "%.3d mA", (int)lastReadCurr_mA);
	HAL_UART_Transmit(&huart1, (uint8_t*)currentString , 10, 10);

	//Voltage
	char voltageString[10];
	sprintf(voltageString, "%.3d V", (int)lastReadBattV);
	HAL_UART_Transmit(&huart1, (uint8_t*)voltageString , 10, 10);

}

void calcSOC(float ocv_V, float charrgeRemain_Ah)
{
	//Simple switch to start with
	if(currentState == IDLE){
		//If cell is not polarized.
		currentCellSOC = socByOCV(ocv_V);
		currChargeRemaining = (currentCellSOC / 100) * fullChargeCapacity;
	} else {
		//If cell is polarized
		currentCellSOC = (currChargeRemaining/fullChargeCapacity) * 100;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == S2_INTERRUPT_Pin || GPIO_Pin == S1_INTERRUPT_Pin)
	{
		//S2 has been pressed, start software debouncing
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

//Where pct should be 0 - 100
void Discharging_Set(uint8_t pct)
{
	if(pct < 0)
	{
		pct = 0;
	}
	else if (pct > 100)
	{
		pct = 100;
	}
	//DAC is 12 bit resolution - 0 - 4095 data codes which translates to 0 - 3.3V analog

	uint32_t dacCode = (uint32_t)(( pct / 100.0 ) * 4095.0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacCode);

	currentDchgPct = pct;

	//Start DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Charging_Enable(int chg_en)
{
	if(chg_en == CHG_ENABLE)
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_SET);
	}
	else //Disabling charging
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_RESET);
	}
}

void Change_State(int new_state)
{
	currentState = new_state;

	//IDLE
	if(currentState == IDLE)
	{
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//CHARGING
	else if(currentState == CHG)
	{
		Charging_Enable(CHG_ENABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//DISCHARGING
	else if (currentState == DCHG)
	{
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(10); //Set discharge current to 10%
	}
	else
	{
		//HANDLE DEFAULT CASE - MISRA C
	}
}

void Safety_Loop()
{
	//Undervoltage - 2.8V
	if(lastReadBattV < 2.8)
	{
		//Stop charging, stop discharging
		//Change_State(IDLE);
	}

	//Overvoltage

	//Overtemperature

	//Overcurrent
}
