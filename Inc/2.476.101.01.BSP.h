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

/* Voltage References */
#define V_REF_LM4140  (2.4985f)  /** @see Messprotokoll: MEASVREF20200717 */
#define V_REF_ADS1255 (2.4923f)  /** @see Messprotokoll: MEASVREF20200717 */
#define V_REF_ADS1256 (2.4926f)  /** @see Messprotokoll: MEASVREF20200717 */
#define V_REF_DAC     (4.0952f)  /** @see Messprotokoll: MEASVREF20200717 */

/* Voltage Divider R1, R2, R3, R4 */
#define V_DIV_R1  (100000.0f)
#define V_DIV_R2  ( 10000.0f)
#define V_DIV_R3  (100000.0f)
#define V_DIV_R4  ( 10000.0f)
#define RS_5    (200.0f)   // Sense R in Ohms
#define RS_2500 (0.4f)     // Sense R in Ohm
#define RS_5_corr    (0.0f)  // calibrated correction value
#define RS_2500_corr (0.0f)  // calibrated correction value

#define VDUT_GAIN_corr     (1.001568195399f)      /** @see Messprotokoll: MEASVREF20200717 */
#define VDUT_OFFSET_corr   (0.004727775287749f)   /** @see Messprotokoll: MEASVREF20200717 */
#define VFORCE_GAIN_corr   (1.000184160203f)      /** @see Messprotokoll: MEASVREF20200717 */
#define VFORCE_OFFSET_corr (0.002076113737264f)   /** @see Messprotokoll: MEASVREF20200717 */

/* Voltage Measurement */
#define V_MEAS_ATTENUATION (14.000f) // from H-Attenuator
// Calibration and Correction coefficients
#define V_MEAS_GAIN_corr   ( 0.9822487775312f ) /** @see Kalibrationsprotokoll: CALVADC20200723 */
#define V_MEAS_OFFSET_corr (-0.01392705932735f) /** @see Kalibrationsprotokoll: CALVADC20200723 */

/* DAC Source */
#define V_SOURCE_GAIN   (2.0f * 4.7f * 5.6f)              // = 52.64 (ideal) according to gain resistors
#define V_SOURCE_GAIN_corr   ((1.0f)/(0.98607345444812f)) /** @see Kalibrationsprotokoll: CALVFORCE20200723 */
#define V_SOURCE_OFFSET_corr (-(-0.01406580558101))       /** @see Kalibrationsprotokoll: CALVFORCE20200723 */

#define V_SOURCE_POS_MAX ( 48.0f)
#define V_SOURCE_NEG_MAX (-48.0f)
#define I_SOURCE_POS_MAX ( 2.5f)
#define I_SOURCE_NEG_MAX (-2.5f)

/* Current Ranging */
typedef enum { 
	RANGE_5mA, 
	RANGE_2500mA,
	RANGE_OFF
} CurrentRange_t;

/* Instrument State / Status */
typedef struct {
	float dacOutputVoltage;
	float dacOutputCurrent;
	float adcInputVoltage;
	float adcInputCurrent;
	float adcVhi;
	float adcVlo;
	float adcVforce;
	float adcVdut;
	
	CurrentRange_t current_range;
} CurveTracer_State_t;

typedef enum {
	DMA_STATE_Ready,
	DMA_STATE_Tx_MUX,
	DMA_STATE_Tx_WKUP,
	DMA_STATE_Tx_RDATA,
	DMA_STATE_Rx_ADC
} ADS1255_DMA_State_t;

/* Init */
void ETA_CTGS_InitDAC(void);
void ETA_CTGS_InitADC(void);
void ETA_CTGS_Init(CurveTracer_State_t *state);
/* Current Sense */
void  ETA_CTGS_OutputOff        (void);
void  ETA_CTGS_CurrentRangeSet  (CurveTracer_State_t *state, CurrentRange_t range);
float ETA_CTGS_GetCurrentSense  (float Vhi, float Vlo, CurrentRange_t range);
/* Voltage Sense */
float ETA_CTGS_GetVoltageSense  (float vadc);
/* Source (DAC) */
void  ETA_CTGS_VoltageOutputSet (CurveTracer_State_t *state, MAX5717_t *dac, float volt);
void  ETA_CTGS_CurrentOutputSet (CurveTracer_State_t *state, MAX5717_t *dac, float volt);

#endif	/* BSP_2_476_101_H */

