/*******************************************************************************
 * @file        BSP.2.476.101.01.c
 * @brief       Board Support Package (BSP) for 2.476.101.01 curve tracer
 * @details     This file implements constants, math and hardware specific 
                functionalities
 * @version     1.0
 * @author      Simon Burkhardt
 * @date        2020.07.05
 * @copyright   (c) 2020 eta systems GmbH
 * @note        2.476.101.01 step generator for curve tracer
********************************************************************************
*/

#include <stdint.h>
#include "main.h"
#include "2.476.101.01.BSP.h"

/**
  * @brief  turns on one of the ranging relays
  * @param  range [RANGE_5mA, RANGE_2500mA]
  */
void ETA_CTGS_CurrentRangeSet(CurrentRange_t range){
	if(range == RANGE_5mA){
		HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_SET); // turn off first !!!
		HAL_Delay(50);
		HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(R5mA_ON_GPIO_Port,    R5mA_ON_Pin,    GPIO_PIN_SET);  // turn on
		HAL_Delay(50);
		HAL_GPIO_WritePin(R5mA_ON_GPIO_Port,    R5mA_ON_Pin,    GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port, R5mA_OFF_Pin, GPIO_PIN_SET); // turn off first !!!
		HAL_Delay(50);
		HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port, R5mA_OFF_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(R25A_ON_GPIO_Port,    R25A_ON_Pin,    GPIO_PIN_SET);  // turn on
		HAL_Delay(50);
		HAL_GPIO_WritePin(R25A_ON_GPIO_Port,    R25A_ON_Pin,    GPIO_PIN_RESET);
	}
}

/**
  * @brief  turns off all outut relays
  */
void ETA_CTGS_OutputOff(void){
	// turn off
	HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port,    R5mA_OFF_Pin,    GPIO_PIN_SET);
	HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port,    R5mA_OFF_Pin,    GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_RESET);
}

float ETA_CTGS_GetCurrent(float Vhi, float Vlo, CurrentRange_t range)
{
	/*
	const float k12 = ETA_CTSG_K12;
	const float k34 = ETA_CTSG_K34;
	const float R12 = ETA_CTSG_R12;
	const float R34 = ETA_CTSG_R34;

	float Vforce = Vhi / (k12 + c12);
	float Vdut = Vlo / (k34 + c34);

	float iMeas = 0.0f;
	float Rs = 0.0f;

	if(range == RANGE_5mA){
		Rs = ETA_CTSG_RS_5 + ETA_CTSG_RS_5_corr;
	} else {
		Rs = ETA_CTSG_RS_2500 + ETA_CTSG_RS_2500_corr;
	}

	iMeas = (Vforce - Vdut) / (Rs);
	*/
	return 0;
}


/**
  * @brief  calculates the real clamp voltage on the Sense input from the ADC voltage
  * @param  Vadc measured ADC voltage
  */
float ETA_CTGS_GetVoltageSense(float vadc)
{
	// do a first order correction (offset and gain) 
	vadc = (vadc * V_MEAS_ATTENUATION);
	// do a linear fit with a reference measurement
	vadc = ( vadc * V_MEAS_GAIN ) + V_MEAS_OFFSET;
	return vadc;
}


void  ETA_CTGS_VoltageOutputSet (MAX5717_t *dac, float volt)
{
	volt = (volt - V_SOURCE_OFFSET) / V_SOURCE_GAIN;
	MAX5717_SetVoltage(dac, volt);
}


