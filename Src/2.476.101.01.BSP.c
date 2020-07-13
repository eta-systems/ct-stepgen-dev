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


void ETA_CTGS_CurrentRange(uint8_t range){

}

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
