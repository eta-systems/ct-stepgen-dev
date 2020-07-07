/*******************************************************************************
 * @file        ads1255.h
 * @brief       C Library for ads1255/ads1256 family of Analog to Digital
 *              Conterters (ADC)
 * @details     This file implements the functionalities of the ADC.
 * @version     1.0
 * @author      Simon Burkhardt
 * @date        2020.05.22
 * @copyright   (c) 2020 eta systems GmbH
********************************************************************************
 * @note        Parts of this library are based on Adien Akhmads Arduino Library 

MIT License

Copyright (c) 2016 Adien Akhmad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include "ads1255.h"


uint8_t ADS125X_Register_Read(uint8_t reg, uint8_t* pData){
  /*
  unsigned char readValue;

  CSON();
  SPI.transfer(RREG | reg);
  SPI.transfer(0);
  __builtin_avr_delay_cycles(200);  // t6 delay (50*tCLKIN), 16Mhz avr clock is
                                    // approximately twice faster that 7.68 Mhz
                                    // ADS1256 master clock
  readValue = SPI.transfer(0);
  __builtin_avr_delay_cycles(8);  // t11 delay
  CSOFF();

  return readValue;
  */

  // except for the register reads and writes (RREG, WREG) 
// which require a second command byte plus data

  // HAL_SPI_Transmit();

}
