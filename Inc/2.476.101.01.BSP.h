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

// Voltage Divider R1, R2, R3, R4
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

// V Measurement
#define V_MEAS_ATTENUATION (14.000f) // from H-Attenuator

// Calibration and Correction coefficients
#define V_MEAS_GAIN   (1.0f)
#define V_MEAS_OFFSET (0.0f)


typedef enum { 
	RANGE_5mA, 
	RANGE_2500mA
} CurrentRange_t;


void  ETA_CTGS_OutputOff       (void);
void  ETA_CTGS_CurrentRangeSet (CurrentRange_t range);
float ETA_CTGS_GetVoltageSense (float Vadc);



#endif	/* BSP_2_476_101_H */

