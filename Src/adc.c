/**
 ******************************************************************************
 * File Name          : ADC.C
 * Description        : LTC2990 ADC DRIVER
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 MAKERMAX INC.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of MAKERMAX INC. nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "adc.h"

/*
 *	Configure control register
 */
HAL_StatusTypeDef LTC2990_ConfigureControlReg(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef retStatus;

	uint8_t controlBuffer[1];
	controlBuffer[0] = 0x1F; //All modes per measurement, V1-V2 (measures current in and out of battery), V3 measures battery voltage (currently disconnected in schematic), V4 measures battery temperature (currently disconnected in schematic)

	retStatus = HAL_I2C_Mem_Write(hi2c, 0b10011000, 0x1, I2C_MEMADD_SIZE_8BIT,
			controlBuffer, I2C_MEMADD_SIZE_8BIT, 500);

	return retStatus;
}

/*
 * Trigger new ADC conversion
 */
HAL_StatusTypeDef LTC2990_Trigger(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef retStatus;
	uint8_t triggerVal = 0x0;

	retStatus = HAL_I2C_Mem_Write(hi2c, 0b10011000, 0x02,
			I2C_MEMADD_SIZE_8BIT, &triggerVal, I2C_MEMADD_SIZE_8BIT, 1500);

	return retStatus;
}

/*
 * Wait for new ADC conversion
 */
HAL_StatusTypeDef LTC2990_WaitForConversion(I2C_HandleTypeDef *hi2c,
		uint16_t timeout_ms) {
	HAL_StatusTypeDef retStatus;

	uint8_t statusBuffer = 0;
	uint8_t vccReady = 0;

	while ((timeout_ms != 0) && (vccReady == 0)) {
		retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x00,
				I2C_MEMADD_SIZE_8BIT, &statusBuffer, I2C_MEMADD_SIZE_8BIT,
				1000);
		vccReady = (statusBuffer & 0b00000100);

		HAL_Delay(10);
		timeout_ms -= 10;
	}

	return retStatus;
}


/*
 * Read Vcc voltage
 */
HAL_StatusTypeDef LTC2990_ReadVcc(I2C_HandleTypeDef *hi2c, float* Vcc_V) {
	HAL_StatusTypeDef retStatus;
	uint16_t Vcc_regData;
	uint8_t VccReadBuffer[2];

	//Read MSB of Vcc ()
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0E, I2C_MEMADD_SIZE_8BIT,
			&VccReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of Vcc ()
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0F, I2C_MEMADD_SIZE_8BIT,
			&VccReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to Vcc
	Vcc_regData = ((VccReadBuffer[0] << 8) | VccReadBuffer[1]) & ~0xC000; //D[13:0]
	*Vcc_V = 2.5 + ((Vcc_regData * 305.18) / 1000000);

	return retStatus;
}

/*
 * Read V3 voltage
 */
HAL_StatusTypeDef LTC2990_ReadV3(I2C_HandleTypeDef *hi2c, float* V3_V) {
	HAL_StatusTypeDef retStatus;
	uint16_t V3_regData;
	uint8_t V3ReadBuffer[2];

	//Read MSB of V3
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0A, I2C_MEMADD_SIZE_8BIT,
			&V3ReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of V3
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0B, I2C_MEMADD_SIZE_8BIT,
			&V3ReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to V3
	V3_regData = ((V3ReadBuffer[0] << 8) | V3ReadBuffer[1]) & ~0xC000; //D[13:0]
	*V3_V = ((V3_regData * 1.1865 * 305.18) / 1000000);

	return retStatus;
}

/*
 * Read V4 voltage
 */
HAL_StatusTypeDef LTC2990_ReadV4(I2C_HandleTypeDef *hi2c, float* V4_V) {
	HAL_StatusTypeDef retStatus;
	uint16_t V4_regData;
	uint8_t V4ReadBuffer[2];

	//Read MSB of V4
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0C, I2C_MEMADD_SIZE_8BIT,
			&V4ReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of V4
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x0D, I2C_MEMADD_SIZE_8BIT,
			&V4ReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to V4
	V4_regData = ((V4ReadBuffer[0] << 8) | V4ReadBuffer[1]) & ~0xC000; //D[13:0]
	*V4_V = ((V4_regData * 305.18) / 1000000) * vdividerFactor ;

	return retStatus;
}

/*
 * Read V2 voltage
 */
HAL_StatusTypeDef LTC2990_ReadV2(I2C_HandleTypeDef *hi2c, float* V2_V) {
	HAL_StatusTypeDef retStatus;
	uint16_t V2_regData;
	uint8_t V2ReadBuffer[2];

	//Read MSB of V2
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x08, I2C_MEMADD_SIZE_8BIT,
			&V2ReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of V2
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x09, I2C_MEMADD_SIZE_8BIT,
			&V2ReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to V4
	V2_regData = ((V2ReadBuffer[0] << 8) | V2ReadBuffer[1]) & ~0xC000; //D[13:0]
	*V2_V = (((V2_regData * 305.18) / 1000000)* vdividerFactor);

	return retStatus;
}

/*
 * Read V1 voltage
 */
HAL_StatusTypeDef LTC2990_ReadV1(I2C_HandleTypeDef *hi2c, float* V1_V) {
	HAL_StatusTypeDef retStatus;
	uint16_t V1_regData;
	uint8_t V1ReadBuffer[2];

	//Read MSB of V1
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x06, I2C_MEMADD_SIZE_8BIT,
			&V1ReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of V1
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x07, I2C_MEMADD_SIZE_8BIT,
			&V1ReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to V4
	V1_regData = ((V1ReadBuffer[0] << 8) | V1ReadBuffer[1]) & ~0xC000; //D[13:0]
	*V1_V = (((V1_regData * 305.18) / 1000000) * vdividerFactor);

	return retStatus;
}

/*
 * Read current going into or out of cell
 * Current reading is in mA
 * Charging gives negative current value when V1_V < V4_V
 * Discharging gives positive current value when V1_V > V4_V
 */
HAL_StatusTypeDef LTC2990_ReadCurrent(I2C_HandleTypeDef *hi2c, float V1_V, float V4_V, float* current_value)
{
	*current_value = (V1_V - V4_V)*1000.0/shuntR;
}

/*
 * Temperature in degrees C
 */
HAL_StatusTypeDef LTC2990_ReadTemperature(I2C_HandleTypeDef *hi2c, float* temp_value)
{
	HAL_StatusTypeDef retStatus;
	uint16_t temp_regData;
	uint8_t tempReadBuffer[2];

	//Read MSB of temp
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x04, I2C_MEMADD_SIZE_8BIT,
			&tempReadBuffer[0], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Read LSB of temp
	retStatus = HAL_I2C_Mem_Read(hi2c, 0b10011000, 0x05, I2C_MEMADD_SIZE_8BIT,
			&tempReadBuffer[1], I2C_MEMADD_SIZE_8BIT, 1000);

	if (retStatus != HAL_OK) {
		return retStatus;
	}

	//Convert register contents to V3
	temp_regData = ((tempReadBuffer[0] << 8) | tempReadBuffer[1]) & ~0xE000; //D[12:0]
	*temp_value = (temp_regData / 16.0);

	return retStatus;

}

/*
 * Read Voltage Higher Level Function
 */
HAL_StatusTypeDef LTC2990_ReadVoltage(I2C_HandleTypeDef *hi2c, ADC_CHANNEL ch, float* voltage_value)
{
	HAL_StatusTypeDef retStatus;

	if(ch == VCC)
	{
		//Vcc
		retStatus = LTC2990_ReadVcc(hi2c, voltage_value);
	}
	else if(ch == BATTV)
	{
		//Vbatt
		retStatus = LTC2990_ReadV1(hi2c, voltage_value);
	}
	else if(ch == BATTV_2)
	{
		//Vbatt2
		retStatus = LTC2990_ReadV2(hi2c, voltage_value);
	}
	else
	{
		//default case
		*voltage_value = 0;
		retStatus = HAL_OK;
	}

	return retStatus;
}
