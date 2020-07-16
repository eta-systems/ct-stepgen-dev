/*******************************************************************************
 * @file        2.476.101.01.BSP.h
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

#ifndef BSP_2_476_101_H
#define	BSP_2_476_101_H

#ifdef	__cplusplus
extern "C" {
#endif

#define STM32F7

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#endif

#include "max5717.h"

#define ETA_CTSG_VREF_DAC (4.0965f)
#define ETA_CTSG_VREF_ADC (2.6613f)

/* Voltage Divider R1, R2, R3, R4 */
#define ETA_CTSG_K12 ((1.0f)/(10.0f))
#define ETA_CTSG_K34 ((1.0f)/(10.0f))
#define ETA_CTSG_K12_corr ()
#define ETA_CTSG_K34_corr ()
#define ETA_CTSG_R12 (110000.0f)    // in Ohms
#define ETA_CTSG_R34 (110000.0f)    // in Ohms
#define ETA_CTSG_RS_5    (200.0f)   // Sense R in Ohms
#define ETA_CTSG_RS_2500 (0.4f)     // Sense R in Ohm
#define ETA_CTSG_RS_5_corr    (0.0f)  // calibrated correction value
#define ETA_CTSG_RS_2500_corr (0.0f)  // calibrated correction value

/* Voltage Measurement */
#define V_MEAS_ATTENUATION (14.000f) // from H-Attenuator
// Calibration and Correction coefficients
#define V_MEAS_GAIN   (0.651829f)// (1.0f)
#define V_MEAS_OFFSET (0.262203f)// (0.0f)

/* DAC Source */
#define V_SOURCE_GAIN   (2.0f * 4.7f * 5.6f) // = 52.64 (ideal) according to gain resistors
#define V_SOURCE_GAIN_corr   (1.000000f)  // (2.0f * 4.7f * 5.6f)  
#define V_SOURCE_OFFSET_corr (0.002198f) // (0.0f) ideal

#define V_SOURCE_POS_MAX ( 48.0f)
#define V_SOURCE_NEG_MAX (-48.0f)
#define I_SOURCE_POS_MAX ( 2.5f)
#define I_SOURCE_NEG_MAX (-2.5f)

typedef enum { 
	RANGE_5mA, 
	RANGE_2500mA,
	RANGE_OFF
} CurrentRange_t;

typedef struct {
	float dacOutputVoltage;
	float dacOutputCurrent;
	float adcInputVoltage;
	float adcInputCurrent;
	CurrentRange_t current_range;
} CurveTracer_State_t;


void ETA_CTGS_InitDAC(void);
void ETA_CTGS_InitADC(void);
void ETA_CTGS_Init(CurveTracer_State_t *state);
/* Current Sense */
void  ETA_CTGS_OutputOff        (void);
void  ETA_CTGS_CurrentRangeSet  (CurveTracer_State_t *state, CurrentRange_t range);
void  ETA_CTGS_GetCurrentSense  (float );
/* Voltage Sense */
float ETA_CTGS_GetVoltageSense  (float vadc);
/* Source (DAC) */
void  ETA_CTGS_VoltageOutputSet (CurveTracer_State_t *state, MAX5717_t *dac, float volt);
void  ETA_CTGS_CurrentOutputSet (CurveTracer_State_t *state, MAX5717_t *dac, float volt);



#endif	/* BSP_2_476_101_H */

