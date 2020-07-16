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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "main.h"
#include "2.476.101.01.BSP.h"
#include <arm_math.h>
#include <stdio.h>
#include "max5717.h"
#include "ads1255.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define ENABLE_BSP_DEBUG_PRINTF

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern MAX5717_t dac1;
extern ADS125X_t adcv;
extern ADS125X_t adci;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
/* USER CODE END EV */

/******************************************************************************/
/*           Board Specific Hardware Functions                                */ 
/******************************************************************************/

/**
  * @brief  initializes MAX5719 DAC
  */
void ETA_CTGS_InitDAC(void)
{
	dac1.csPort    = SPI4_CS_GPIO_Port;
	dac1.csPin     = SPI4_CS_Pin;
	dac1.latchPort = SPI4_LATCH_GPIO_Port;
	dac1.latchPin  = SPI4_LATCH_Pin;

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config MAX5717...\n");
#endif
	
	MAX5717_Init(&dac1, &hspi4, ETA_CTSG_VREF_DAC);
	
#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	
}

/**
  * @brief  initializes both ADS125x ADCs
  */
void ETA_CTGS_InitADC(void)
{
  HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, GPIO_PIN_SET);  // SYNC Pin is PDWN Pin --> turn on ADS125x
	
	adci.csPort   = SPI1_CS_GPIO_Port;
	adci.csPin    =  SPI1_CS_Pin;
	adci.drdyPort = SPI1_DRDY_GPIO_Port;
	adci.drdyPin  =  SPI1_DRDY_Pin;
	adci.vref = ETA_CTSG_VREF_ADC;
	adci.hspix = &hspi1;
	
#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config ADS1256...\n");
#endif	
	
	ADS125X_Init(&adci, &hspi1, ADS125X_DRATE_2_5SPS, ADS125X_PGA1, 0);
	
#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	
	
	HAL_Delay(500);  // wait for clkout to start ADS1255
	
	adcv.csPort   = SPI3_CS_GPIO_Port;
	adcv.csPin    =  SPI3_CS_Pin;
	adcv.drdyPort = SPI3_DRDY_GPIO_Port;
	adcv.drdyPin  =  SPI3_DRDY_Pin;
	adcv.vref = ETA_CTSG_VREF_ADC;
	adcv.hspix = &hspi3;
	
#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config ADS1255...\n");
#endif	

	ADS125X_Init(&adcv, &hspi3, ADS125X_DRATE_2_5SPS, ADS125X_PGA1, 0);
	ADS125X_ChannelDiff_Set(&adcv, ADS125X_MUXP_AIN1, ADS125X_MUXN_AIN0);
	
#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	

}

/**
  * @brief  initializes Curve Tracer State and pripherals
  */
void ETA_CTGS_Init(CurveTracer_State_t *state)
{
	// input values
	state->adcInputVoltage = 0.0f;
	state->adcInputCurrent = 0.0f;
	// output values
	state->dacOutputVoltage = 0.0f;
	state->dacOutputCurrent = 0.0f;
	
	state->current_range = RANGE_OFF;
	
	ETA_CTGS_OutputOff();  // turn Ranging relais off
	
	HAL_GPIO_WritePin(dac1.csPort, dac1.csPin, GPIO_PIN_RESET); // chip select
	HAL_GPIO_WritePin(R5mA_ON_GPIO_Port, R5mA_ON_Pin, GPIO_PIN_SET); // chip select
	
	
	// ADS125X_ChannelDiff_Set(&adci, ADS125X_MUXP_AIN0, ADS125X_MUXN_AINCOM);
	ADS125X_ChannelDiff_Set(&adci, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN5);
	ADS125X_CMD_Send(&adci, ADS125X_CMD_SYNC);
	ADS125X_CMD_Send(&adci, ADS125X_CMD_WAKEUP);
}

/**
  * @brief  turns on one of the ranging relays
  * @param  range [RANGE_5mA, RANGE_2500mA]
  */
void ETA_CTGS_CurrentRangeSet(CurveTracer_State_t *state, CurrentRange_t range){
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
	vadc = ( vadc * V_MEAS_GAIN_corr ) + V_MEAS_OFFSET_corr;
	return vadc;
}

/**
  * @brief  calculates the real clamp voltage on the Sense input from the ADC voltage
	* @param  *state the curve tracer device state
	* @param  *dac pointer to the DAC handle
  * @param  volt the desired output voltage on Vforce
  */
void  ETA_CTGS_VoltageOutputSet (CurveTracer_State_t *state, MAX5717_t *dac, float volt)
{
	// calculations are in reverse
	// input value is +/- 48V
	// DAC output is +/- 2.048V
	// so divide by the gain, subtract offset...
	volt = volt / V_SOURCE_GAIN; // ideal gain
	volt = (volt - V_SOURCE_OFFSET_corr) / V_SOURCE_GAIN_corr;
	MAX5717_SetVoltage(dac, volt);
	state->dacOutputVoltage = volt;
}


/*

	float A = ((48.0f / 4.7f)/5.6f);
	float stepsize = A/(float)DMA_BUFFER_SIZE;
	for(uint16_t i=0; i<DMA_BUFFER_SIZE; i++){
		volts = (i*stepsize)-(A/2.0f);
		uint32_t code = MAX5717_VoltageToCode(&dac1, volts);
		code = code << 4;
		dmaDacTx[3*i]   = (uint8_t)((code >> 16) & 0xFF);
		dmaDacTx[3*i+1] = (uint8_t)((code >>  8) & 0xFF);
		dmaDacTx[3*i+2] = (uint8_t)((code >>  0) & 0xFF);
		// printf("%.5f,\n", volts);
		//printf("%d\n", code);
	}





*/




