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

#include <stdint.h>
#include "max5717.h"


/**  MSB
  * [ 7  6  5  4  3  2  1  0]:[ 7  6  5  4  3  2  1  0]:[ 7  6  5  4  3  2  1  0]
  *  19 18 17 16 15 14 13 12   11 10 09 08 07 06 05 04   03 02 01 00  x  x  x  x
  */

void MAX5719_VoltageToCode(float volt, uint8_t* pBytes, uint8_t len){
    float fltCode = volt * ((float)MAX5717_CODE_MAX) / MAX5717_VREF;
    if(fltCode < 0.0f) fltCode = 0.0f;
    uint32_t code = (uint32_t)fltCode;

    #ifndef MAX5719
    code = (code << 4) & 0x00FFFFF0;
    #endif

    uint8_t payload[MAX5717_DATA_LENGTH];
    for(uint8_t i=0; (i<MAX5717_DATA_LENGTH) && (i<len); i++){
        payload[i] = 1;
    }
}



