/*******************************************************************************
 * @file        max5717.h
 * @brief       C Library for max5717/max5719 family of Digital Analog 
 *              Converters (DAC)
 * @details     This file implements the functionalities of the DAC.
 * @version     1.0
 * @author      Simon Burkhardt
 * @date        2020.05.22
 * @copyright   (c) 2020 eta systems GmbH
*******************************************************************************/

#ifndef MAX5717_H
#define	MAX5717_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef STM32F4XX_H
#include "stm32f7xx_hal.h"
#endif

#ifndef STM32F4XX_HAL_I2C_H
#include "stm32f7xx_hal_spi.h"
#endif

// #define MAX5717 // 16 bit interface
#define MAX5719 // 20 bit interface

// reference voltage
#define MAX5717_VREF (4.096f)

#ifndef MAX5717
#define MAX5717_DATA_LENGTH 2
#define MAX5717_CODE_MAX (0xFFFF)
#endif

#ifndef MAX5719
#define MAX5717_DATA_LENGTH 3
#define MAX5717_CODE_MAX (0x0FFFFF)
#endif

void MAX5719_VoltageToCode(float volt, uint8_t* pBytes, uint8_t len);

#endif	/* MAX5717_H */

