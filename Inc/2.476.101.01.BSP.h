/*******************************************************************************
 * @file        2.476.101.01.BSP.h
 * @brief       Board Support Package (BSP) C Library for 
 *              2.476.101.01 step generator 
 * @details     This file implements the functionalities of the DAC.
 * @version     1.0
 * @author      Simon Burkhardt
 * @date        2020.05.22
 * @copyright   (c) 2020 eta systems GmbH
*******************************************************************************/

#ifndef ETA_CT_STEP_GEN_H
#define	ETA_CT_STEP_GEN_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef STM32F4XX_H
#include "stm32f7xx_hal.h"
#endif

#ifndef STM32F4XX_HAL_I2C_H
#include "stm32f7xx_hal_spi.h"
#endif

// #define ADC_VREF_EXT
#define ADC_VREF_INT



#endif	/* ETA_CT_STEP_GEN_H */

