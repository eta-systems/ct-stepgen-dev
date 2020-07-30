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
//#include <arm_math.h>
#include <math.h>
#include <stdio.h>
#include "max5717.h"
#include "ads1255.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define ENABLE_BSP_DEBUG_PRINTF
//#define ENABLE_BSP_VALUES_PRINTF
#define ENABLE_BSP_WATCHDOG_PRINTF

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern MAX5717_t dac1;
extern ADS125X_t adcv;
extern ADS125X_t adci;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

uint8_t TxDMAchannelCycle[16];
uint8_t TxDMABufferOffsetStep;
uint8_t TxDMABufferOffset;
/* USER CODE END EV */

/******************************************************************************/
/*           Board Specific Hardware Functions                                */
/******************************************************************************/

/**
 * @brief  initializes MAX5719 DAC
 */
void ETA_CTGS_InitDAC(void) {
	dac1.csPort = SPI4_CS_GPIO_Port;
	dac1.csPin = SPI4_CS_Pin;
	dac1.latchPort = SPI4_LATCH_GPIO_Port;
	dac1.latchPin = SPI4_LATCH_Pin;

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config MAX5717...\n");
#endif

	MAX5717_Init(&dac1, &hspi4, V_REF_DAC);

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	
}

/**
 * @brief  initializes both ADS125x ADCs
 */
void ETA_CTGS_InitADC(void) {
	/* ADS1256 must be initialized first because of CLKOUT to ADS1255 */
	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, GPIO_PIN_SET); // SYNC Pin is PDWN Pin --> turn on ADS125x

	adci.csPort = SPI1_CS_GPIO_Port;
	adci.csPin = SPI1_CS_Pin;
	adci.drdyPort = SPI1_DRDY_GPIO_Port;
	adci.drdyPin = SPI1_DRDY_Pin;
	adci.vref = V_REF_ADS1256;
	adci.hspix = &hspi1;

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config ADS1256...\n");
#endif	

	ADS125X_Init(&adci, &hspi1, ADS125X_DRATE_100SPS, ADS125X_PGA1, 0);

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	

	HAL_Delay(500);  // wait for clkout to start ADS1255

	adcv.csPort = SPI3_CS_GPIO_Port;
	adcv.csPin = SPI3_CS_Pin;
	adcv.drdyPort = SPI3_DRDY_GPIO_Port;
	adcv.drdyPin = SPI3_DRDY_Pin;
	adcv.vref = V_REF_ADS1255;
	adcv.hspix = &hspi3;

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("config ADS1255...\n");
#endif	

	ADS125X_Init(&adcv, &hspi3, ADS125X_DRATE_50SPS, ADS125X_PGA2, 0); // PGA=2
	ADS125X_ChannelDiff_Set(&adcv, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);

#ifdef ENABLE_BSP_DEBUG_PRINTF
	printf("done\n");
#endif	

	/* prepare DMA Buffer for fast channel cycling */
	// CHANNEL 4/5
	TxDMAchannelCycle[0] = ADS125X_CMD_WREG | ADS125X_REG_MUX;
	TxDMAchannelCycle[1] = 0x00;
	TxDMAchannelCycle[2] = ADS125X_MUXP_AIN4 | ADS125X_MUXN_AIN5;  // Vhi
	TxDMAchannelCycle[3] = ADS125X_CMD_SYNC;
	TxDMAchannelCycle[4] = ADS125X_CMD_WAKEUP;
	TxDMAchannelCycle[5] = ADS125X_CMD_RDATA;

	// CHANNEL 2/3
	TxDMAchannelCycle[6] = ADS125X_CMD_WREG | ADS125X_REG_MUX;
	TxDMAchannelCycle[7] = 0x00;
	TxDMAchannelCycle[8] = ADS125X_MUXP_AIN2 | ADS125X_MUXN_AIN3;  // Vlo
	TxDMAchannelCycle[9] = ADS125X_CMD_SYNC;
	TxDMAchannelCycle[10] = ADS125X_CMD_WAKEUP;
	TxDMAchannelCycle[11] = ADS125X_CMD_RDATA;

	TxDMABufferOffsetStep = 6;
	TxDMABufferOffset = 0;

}

/**
 * @brief  initializes Curve Tracer State and pripherals
 */
void ETA_CTGS_Init(CurveTracer_State_t *state) {
	// input values
	state->adcInputVoltage = 0.0f;
	state->adcInputCurrent = 0.0f;
	// output values
	state->dacOutputVoltage = 0.0f;
	state->dacOutputCurrent = 0.0f;
	// watchdog values
	state->maxOC = 0.0051f;
	state->minOC = -0.0051f;
	state->maxOV = 5.100f;
	state->minOV = -5.100f;

	state->current_range = RANGE_OFF;
	state->pidMode = PID_MODE_VOLTAGE;
	state->resetPid = 1;

	ETA_CTGS_OutputOff(state);  // turn Ranging relais off

	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, GPIO_PIN_SET);
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
 * @note   Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
 *         based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
 *         a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
 *         than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
 *         To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
 */
void ETA_CTGS_CurrentRangeSet(CurveTracer_State_t *state, CurrentRange_t range) {
	ETA_CTGS_OutputOff(state);  // turn off  first!
	if (range == RANGE_5mA) {
		state->current_range = range;
		HAL_GPIO_WritePin(R5mA_ON_GPIO_Port, R5mA_ON_Pin, GPIO_PIN_SET); // turn on
		HAL_Delay(50);
		HAL_GPIO_WritePin(R5mA_ON_GPIO_Port, R5mA_ON_Pin, GPIO_PIN_RESET);
	} else if (range == RANGE_2500mA) {
		state->current_range = range;
		HAL_GPIO_WritePin(R25A_ON_GPIO_Port, R25A_ON_Pin, GPIO_PIN_SET); // turn on
		HAL_Delay(50);
		HAL_GPIO_WritePin(R25A_ON_GPIO_Port, R25A_ON_Pin, GPIO_PIN_RESET);
	}
}

/**
 * @brief  turns off all outut relays
 * @note   Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
 *         based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
 *         a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
 *         than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
 *         To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
 */
void ETA_CTGS_OutputOff(CurveTracer_State_t *state) {
	state->current_range = RANGE_OFF;
	// turn off
	HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port, R5mA_OFF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(R5mA_OFF_GPIO_Port, R5mA_OFF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R25A_OFF_GPIO_Port, R25A_OFF_Pin, GPIO_PIN_RESET);
}

float ETA_CTGS_GetCurrentSense(CurveTracer_State_t *state, float Vhi, float Vlo) {
	static float Vforce;
	static float Vdut;
	static float Rs;
	static float Idut;
	static float Ibias;
	static float Vshunt;
	static float k12 = V_DIV_K12;
	static float k34 = V_DIV_K34;

	if (state->current_range == RANGE_5mA) {
		Rs = RS_5 + RS_5_corr;
	} else {
		Rs = RS_2500 + RS_2500_corr;
	}

	// Vlo = (Vlo*VLO_GAIN_corr) + VLO_OFFSET_corr;
	// Vhi = (Vhi*VHI_GAIN_corr) + VHI_OFFSET_corr;

	//Vforce = ( Vhi * ((V_DIV_R1 + V_DIV_R2) / V_DIV_R2 ) );
	Vforce = (Vhi + VHI_OFFSET) * k12;
	state->adcVforce = Vforce;
	//Vforce = (Vforce*VFORCE_GAIN_corr) + VFORCE_OFFSET_corr;

	/** @see Equation (3.4) */
	//Vdut = ( Vlo * ((V_DIV_R3 + V_DIV_R4) / V_DIV_R4 ) );
	//Vdut = (Vdut*VDUT_GAIN_corr) + VDUT_OFFSET_corr;
	Vdut = (Vlo + VLO_OFFSET) * k34;
	state->adcVdut = Vdut;

	/** @see Equation (3.3) */
	Ibias = (Vlo + VLO_OFFSET) / V_DIV_R4;
	//Ibias = Vdut / (V_DIV_R4 + V_DIV_R3);

	/** @see Equation (3.2) */
	// Vshunt = ( Vhi * ((V_DIV_R1 + V_DIV_R2) / V_DIV_R2 ) ) - Vdut;
	Vshunt = Vforce - Vdut;

	/** @see Equation (3.1) */
	Idut = (Vshunt / Rs) - Ibias;

#ifdef ENABLE_BSP_VALUES_PRINTF
	//printf("%.8f\t%.8f\n", Vforce, Vdut);
	//printf("%.7f\t%.7f\n", Vhi, Vlo);
	//printf("%.4f mV, %.1f R\n", 1000*Vshunt, Rs);
	//printf("%.5f mA, %.4f V, %.2f R, %.2f mW\n", 1000.0f*Idut, Vdut, Vdut/Idut, 1000*Vdut*Idut);
	//printf("%.5f\t%.5f\n", Vdut, 1000.0f*Idut);
	//printf("%.7f\t%.8f\n", Vdut, Idut);
	printf("%.5f mA, %.4f V, %.2f R, %.2f mW\n", 1000.0f * Idut, Vdut,
			Vdut / Idut, 1000 * Vdut * Idut);
#endif

	state->adcInputCurrent = Idut;
	return Idut;
}

/**
 * @brief  calculates the real clamp voltage on the Sense input from the ADC voltage
 * @param  Vadc measured ADC voltage
 */
float ETA_CTGS_GetVoltageSense(float vadc) {
	vadc = (vadc * V_MEAS_ATTENUATION);
	// do a first order correction (offset and gain) 
	// do a linear fit with a reference measurement
	vadc = (vadc * V_MEAS_GAIN_corr) + V_MEAS_OFFSET_corr;

#ifdef ENABLE_BSP_VALUES_PRINTF
	//printf("%.4f\t", vadc);
#endif

	return vadc;
}

/**
 * @brief  calculates the real clamp voltage on the Sense input from the ADC voltage
 * @param  *state the curve tracer device state
 * @param  *dac pointer to the DAC handle
 * @param  volt the desired output voltage on Vforce
 */
void ETA_CTGS_VoltageOutputSet(CurveTracer_State_t *state, MAX5717_t *dac, float volt) {
	// calculations are in reverse
	// input value is +/- 48V
	// DAC output is +/- 2.048V
	// so divide by the gain, subtract offset...
	state->dacOutputVoltage = volt;
	volt = (volt - V_SOURCE_OFFSET_corr) / V_SOURCE_GAIN_corr;
	volt = volt / V_SOURCE_GAIN; // ideal gain
	MAX5717_SetVoltage(dac, volt);
	// printf("%.2f\n", volt);
}

/**
 * @brief  simplified control system for voltage and current regulation
 * @param  *state the curve tracer device state
 * @note   Everything in here is trial and error
 *
 */
void ETA_CTGS_ControllAlgorithm(CurveTracer_State_t *state){
	static float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;

	static float iteration_time = 1.0f/50.0f;
	static float newOutput;

	static float error = 0, error_prior = 0;
	static float integral = 0, integral_prior = 0;
	static float derivative = 0;

	if(state->resetPid){
		state->resetPid = 0;
		error = 0;
		error_prior = 0;
		integral = 0;
		integral_prior = 0;
		if(state->pidMode == PID_MODE_VOLTAGE){
			static float vKp = 0.1f;
			static float vKi = 5.0f;
			static float vKd = 0.0f;
			Kp = vKp;
			Ki = vKi;
			Kd = vKd;
		} else {
			static float iKp = 0.1f;
			static float iKd = 0.0f;
			Kp = iKp;
			if(state->current_range == RANGE_5mA)
				Ki = 8000.0f;
			else
				Ki = 20.0f;
			Kd = iKd;
		}
	}

	if(state->pidMode == PID_MODE_VOLTAGE){
		error = state->desiredVoltage - state->adcVforce;   // Vforce is always present. Vdut is after the relay
	} else {
		error = state->desiredCurrent - state->adcInputCurrent;
		/*
		// artificial Slew Rate
		if(error > 0.1)
			error = 0.1;
		if(error < -0.1)
			error = -0.1;
		*/
	}

	integral = integral_prior + error * iteration_time;
	derivative = (error - error_prior) / iteration_time;

	newOutput = Kp*error + Ki*integral + Kd*derivative;

	error_prior = error;
	integral_prior = integral;

	// boundry check / Limiter
	if(newOutput > V_SOURCE_POS_MAX)
		newOutput = V_SOURCE_POS_MAX;
	if(newOutput < V_SOURCE_NEG_MAX)
		newOutput = V_SOURCE_NEG_MAX;

	ETA_CTGS_VoltageOutputSet(state, &dac1, newOutput);
}

/**
  * @brief  Watchdog Function dealing with Over Current / Over Voltage protection
  * @param  *state the curve tracer device state
  */
CT_StatusTypeDef ETA_CTGS_Watchdog(CurveTracer_State_t *state)
{
	CT_StatusTypeDef status = CT_OK;

	/* Voltages */
	if((state->adcVhi > 6.0f) || (state->adcVhi < -6.0f))
		status |= CT_ERROR_VALUE;
	if((state->adcVlo > 6.0f) || (state->adcVlo < -6.0f))
		status |= CT_ERROR_VALUE;

	/*
	if((state->adcVdut > V_SOURCE_POS_MAX) || (state->adcVdut < V_SOURCE_NEG_MAX)){
		status |= CT_ERROR_VALUE;
	}
	if((state->adcVforce > V_SOURCE_POS_MAX) || (state->adcVforce < V_SOURCE_NEG_MAX)){
		status |= CT_ERROR_VALUE;
	}
	if((state->adcInputVoltage > V_SOURCE_POS_MAX) || (state->adcInputVoltage < V_SOURCE_NEG_MAX)){
		status |= CT_ERROR_VALUE;
	}
	if((state->dacOutputVoltage > V_SOURCE_POS_MAX) || (state->dacOutputVoltage < V_SOURCE_NEG_MAX))
		status |= CT_ERROR_VALUE;
	*/

	// voltage accross sense resistor
	if( fabsf(state->adcVdut - state->adcVforce) > 1.5f ){
		status |= CT_ERROR_OC;
		status |= CT_ERROR_VALUE;
#ifdef ENABLE_BSP_WATCHDOG_PRINTF
		printf("[wdg] Vsense > 1.5V\n");
#endif
	}

	if( (state->adcInputVoltage > state->maxOV) || (state->adcInputVoltage < state->minOV) ){
		status |= CT_ERROR_OV;
	}

	if(state->current_range != RANGE_OFF){
		if( (state->adcInputCurrent > state->maxOC) || (state->adcInputCurrent < state->minOC) ){
			status |= CT_ERROR_OC;
		}
	}

	/* Handle Errors */
	if(status | CT_ERROR_OC){
		// turn off
		ETA_CTGS_OutputOff(state);
		ETA_CTGS_VoltageOutputSet(state, &dac1, 0.0f);
#ifdef ENABLE_BSP_WATCHDOG_PRINTF
		printf("[wdg] OC\n");
#endif
	}

	if(status | CT_ERROR_OV){
		// turn off
		ETA_CTGS_OutputOff(state);
		ETA_CTGS_VoltageOutputSet(state, &dac1, 0.0f);
#ifdef ENABLE_BSP_WATCHDOG_PRINTF
		printf("[wdg] OV\n");
#endif
	}

	return status;
}


