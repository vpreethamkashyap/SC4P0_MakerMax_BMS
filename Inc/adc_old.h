/**
 ******************************************************************************
 * File Name          : ADC.H
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

#ifndef ADC_H_
#define ADC_H_

#include "stm32f3xx_hal.h"

typedef enum
{
	VCC = 0,
	BATTV = 1,
	CH2 = 2,
	CH3 = 3,
	BATTV_2 = 4,
}ADC_CHANNEL;

//Based on MakerMax SC4p0 hardware
static const float vdividerFactor = 2.0;
static const float shuntR = 0.155; //ohms

// INSTRUCTIONS

/* To read voltage
 * 1. LTC2990_Trigger(&hi2c1) to trigger a new conversion
 * 2. LTC2990_WaitForConversion(&hi2c1, 20) to wait for a new conversion.
 * 3. LTC2990_ReadVcc(&hi2c1, &Vcc_V); read your desired voltage. In this case, reading Vcc
 *
 * To read current
 * 1. LTC2990_Trigger(&hi2c1) to trigger a new conversion
 * 2. LTC2990_WaitForConversion(&hi2c1, 20) to wait for a new adc conversion.
 * 3. LTC2990_ReadVcc(&hi2c1, &Vcc_V); read your desired current. In this case, reading Vcc
 */

/*
 * Function Prototypes
 */

/*
 * ADC Configuration and trigger functions
 */
HAL_StatusTypeDef LTC2990_ConfigureControlReg(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LTC2990_Trigger(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LTC2990_WaitForConversion(I2C_HandleTypeDef *hi2c,
		uint16_t timeout_ms);

/*
 * ADC voltage functions
 */
HAL_StatusTypeDef LTC2990_ReadVoltage(I2C_HandleTypeDef *hi2c, ADC_CHANNEL ch, float* voltage_value);
HAL_StatusTypeDef LTC2990_ReadVcc(I2C_HandleTypeDef *hi2c, float* Vcc_V);
HAL_StatusTypeDef LTC2990_ReadV3(I2C_HandleTypeDef *hi2c, float* V3_V);
HAL_StatusTypeDef LTC2990_ReadV4(I2C_HandleTypeDef *hi2c, float* V4_V);
HAL_StatusTypeDef LTC2990_ReadV2(I2C_HandleTypeDef *hi2c, float* V2_V);
HAL_StatusTypeDef LTC2990_ReadV1(I2C_HandleTypeDef *hi2c, float* V1_V);

/*
 * ADC temperature read function
 */
HAL_StatusTypeDef LTC2990_ReadTemperature(I2C_HandleTypeDef *hi2c, float* temp_value);

/*
 * ADC processor current read function
 */
HAL_StatusTypeDef LTC2990_ReadCurrent(I2C_HandleTypeDef *hi2c, float V1_V, float V4_V, float* current_value);

#endif /* ADC_H_ */