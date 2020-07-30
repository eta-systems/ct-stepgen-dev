/*-
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2018, Jan Breuer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * @file    scpi-def.c
 * @brief   contains user settings and definitions for SCPI command implementation
 * @details This file implements constants, math and hardware specific 
 functionalities
 * @version 1.0
 * @author  eta Systems GmbH
 * @author  Simon Burkhardt
 * @date    2020-07-13
 * @copyright (c) 2020 eta systems GmbH
 * @see https://github.com/j123b567/scpi-parser/blob/master/examples/common/scpi-def.c
 ********************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
//#include "max5717.h"
#include "2.476.101.01.BSP.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define ENABLE_SCPI_DEBUG_PRINTF
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
// extern MAX5717_t dac1;
extern volatile CurveTracer_State_t deviceState;
/* USER CODE END EV */

/******************************************************************************/
/*           SCPI Command Callback Handlers                                   */
/******************************************************************************/

/**
 * @brief sets the Current Sensing Output Range
 */
static scpi_result_t scpi_etaCT_RangeCurrent(scpi_t * context)
{
	double param1;
	/* read first parameter if present */
	if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}

	if ((param1 > 0.004f) && (param1 < 0.006f)) {
		ETA_CTGS_CurrentRangeSet( (CurveTracer_State_t*)&deviceState, RANGE_5mA);
	}
	if ((param1 > 0.006f) && (param1 < 2.6f)) {
		ETA_CTGS_CurrentRangeSet( (CurveTracer_State_t*)&deviceState, RANGE_2500mA);
	}

	return SCPI_RES_OK;
}

/**
 * @brief switches off the Output completely
 */
static scpi_result_t scpi_etaCT_RangeCurrentOff(scpi_t * context)
{
	ETA_CTGS_CurrentRangeSet( (CurveTracer_State_t*)&deviceState, RANGE_OFF);
	return SCPI_RES_OK;
}

/**
 * @brief  save and set a desired output voltage
 */
static scpi_result_t scpi_etaCT_SetVoltage(scpi_t * context)
{
	double param1;
	/* read first parameter if present */
	if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}

	/* input boundry check */
	if ((param1 > V_SOURCE_POS_MAX) || (param1 < V_SOURCE_NEG_MAX)) {
		return SCPI_RES_ERR;
		// else: ignore and don't change the output
	}
	if(deviceState.pidMode != PID_MODE_VOLTAGE){
		deviceState.resetPid = 1;
		deviceState.pidMode = PID_MODE_VOLTAGE;
	}
	deviceState.desiredVoltage = (float) (param1);

	/** @important Do not directly set the voltage.
	 * the control system is taking care of this !!
	 */
	// ETA_CTGS_VoltageOutputSet((CurveTracer_State_t*) &deviceState, &dac1, (float) (param1));

	return SCPI_RES_OK;
}

/**
 * @brief  read back the stored voltage
 */
static scpi_result_t scpi_etaCT_SetVoltageQ(scpi_t * context)
{
	double param1 = (double) (deviceState.desiredVoltage);
	// SCPI_ResultDouble(context, param1); // is this the way to do it ??
	printf("%.4f\r\n", param1);  // 100V / 20 Bit = 100uV resolution --> .4f
	return SCPI_RES_OK;
}

/**
 * @brief  save and set a desired output voltage
 */
static scpi_result_t scpi_etaCT_SetVoltageLim(scpi_t * context)
{
	double param1;
	/* read first parameter if present */
	if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}

	/* input boundry check */
	if ((param1 > V_SOURCE_POS_MAX) || (param1 < V_SOURCE_NEG_MAX)) {
		return SCPI_RES_ERR;
		// else: ignore and don't change the output
	}
	deviceState.maxOV = (float) ( param1);
	deviceState.minOV = (float) (-param1);

	return SCPI_RES_OK;
}

/**
 * @brief  read back the stored voltage
 */
static scpi_result_t scpi_etaCT_SetVoltageLimQ(scpi_t * context)
{
	double param1 = (double) (deviceState.maxOV);
	// SCPI_ResultDouble(context, param1); // is this the way to do it ??
	printf("%.4f\r\n", param1);  // 100V / 20 Bit = 100uV resolution --> .4f
	return SCPI_RES_OK;
}

/**
 * @brief  save and set a desired output current (max current for OCP)
 */
static scpi_result_t scpi_etaCT_SetCurrent(scpi_t * context)
{
	double param1;
	/* read first parameter if present */
	if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}
	/* input boundry check */
	if(deviceState.current_range == RANGE_5mA){
		if( (param1 > I_SOURCE_POS_MAX_5 ) || (param1 < I_SOURCE_NEG_MAX_5) )
			return SCPI_RES_ERR;
			// else: ignore and don't change the output
	} else if (deviceState.current_range == RANGE_2500mA){
		if( (param1 > I_SOURCE_POS_MAX_2500 ) || (param1 < I_SOURCE_NEG_MAX_2500) )
			return SCPI_RES_ERR;
			// else: ignore and don't change the output
	}

	/** @important Do not directly set the voltage.
	 * the control system is taking care of this !!
	 */
	if(deviceState.pidMode != PID_MODE_CURRENT){
			deviceState.resetPid = 1;
			deviceState.pidMode = PID_MODE_CURRENT;
		}
	deviceState.desiredCurrent = (float) ( param1);

	return SCPI_RES_OK;
}

/**
 * @brief  read back the stored current
 */
static scpi_result_t scpi_etaCT_SetCurrentQ(scpi_t * context)
{
	double param1 = (double) (deviceState.desiredCurrent);
	//SCPI_ResultDouble(context, param1);
	printf("%.6f\r\n", param1);
	return SCPI_RES_OK;
}

/**
 * @brief  save and set a desired output current (max current for OCP)
 */
static scpi_result_t scpi_etaCT_SetCurrentLim(scpi_t * context)
{
	double param1;
	/* read first parameter if present */
	if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}
	/* input boundry check */
	if(deviceState.current_range == RANGE_5mA){
		if( (param1 > I_SOURCE_POS_MAX_5 ) || (param1 < I_SOURCE_NEG_MAX_5) )
			return SCPI_RES_ERR;
			// else: ignore and don't change the output
	} else if (deviceState.current_range == RANGE_2500mA){
		if( (param1 > I_SOURCE_POS_MAX_2500 ) || (param1 < I_SOURCE_NEG_MAX_2500) )
			return SCPI_RES_ERR;
			// else: ignore and don't change the output
	}
	if(param1 < 0.0f){
		return SCPI_RES_ERR;
	}

	/** @important Do not directly set the voltage.
	 * the control system is taking care of this !!
	 */
	deviceState.maxOC = (float) ( param1);
	deviceState.minOC = (float) (-param1);

	return SCPI_RES_OK;
}

/**
 * @brief  read back the stored current
 */
static scpi_result_t scpi_etaCT_SetCurrentLimQ(scpi_t * context)
{
	double param1 = (double) (deviceState.maxOC);
	//SCPI_ResultDouble(context, param1);
	printf("%.6f\r\n", param1);
	return SCPI_RES_OK;
}

/**
 * @brief  read back the last measured DC voltage
 */
static scpi_result_t scpi_etaCT_MeasureVoltageQ(scpi_t * context)
{
	//SCPI_ResultDouble(context, param1);
	printf("%.5f\r\n", deviceState.adcInputVoltage);  // 100V / 24 Bit = 6uV ~ 10uV --> .5f
	return SCPI_RES_OK;
}

/**
 * @brief  read back the last measured DC current
 */
static scpi_result_t scpi_etaCT_MeasureCurrentQ(scpi_t * context)
{
	//SCPI_ResultDouble(context, param1);
	if(deviceState.current_range == RANGE_2500mA){
		printf("%.7f\r\n", deviceState.adcInputCurrent);  // LSB = 2.5uA --> .7f
	} else if (deviceState.current_range == RANGE_5mA) {
		printf("%.8f\r\n", deviceState.adcInputCurrent);  // LSB = 50nA  --> .8f
	}
	return SCPI_RES_OK;
}

/**
 * @brief  read back the present current range setting of the relais
 */
static scpi_result_t scpi_etaCT_RangeCurrentQ(scpi_t * context)
{
	if(deviceState.current_range == RANGE_5mA){
		printf("0.005\r\n");
	} else if(deviceState.current_range == RANGE_2500mA){
		printf("2.5\r\n");
	} else {
		printf("OFF\r\n");
	}
	return SCPI_RES_OK;
}

/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t * context) {

	SCPI_ResultInt32(context, 0);

	return SCPI_RES_OK;
}

/******************************************************************************/
/*           SCPI Command Table Initialization                                */
/******************************************************************************/

const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = SCPI_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = My_CoreTstQ,},
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?",  .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?",      .callback = SCPI_SystemVersionQ,},

    /* DMM */
    {.pattern = "MEASure:VOLTage[:DC]?",             .callback = scpi_etaCT_MeasureVoltageQ,},
    {.pattern = "MEASure:CURRent[:DC]?",             .callback = scpi_etaCT_MeasureCurrentQ,},
    {.pattern = "SENSe:CURRent[:DC]:RANGe[:UPPer]",  .callback = scpi_etaCT_RangeCurrent,},
    {.pattern = "SENSe:CURRent[:DC]:RANGe[:UPPer]?", .callback = scpi_etaCT_RangeCurrentQ,},
    /* {.pattern = "MEASure:RESistance?", .callback = SCPI_StubQ,}, */

	/* SOURCE */
    {.pattern = "SOURce:VOLTage[:LEVel]",  .callback = scpi_etaCT_SetVoltage,},
    {.pattern = "SOURce:VOLTage[:LEVel]?", .callback = scpi_etaCT_SetVoltageQ,},
    {.pattern = "SOURce:VOLTage:LIMit",  .callback = scpi_etaCT_SetVoltageLim,},
    {.pattern = "SOURce:VOLTage:LIMit?", .callback = scpi_etaCT_SetVoltageLimQ,},
    {.pattern = "SOURce:CURRent[:LEVel]",  .callback = scpi_etaCT_SetCurrent,},
    {.pattern = "SOURce:CURRent[:LEVel]?", .callback = scpi_etaCT_SetCurrentQ,},
    {.pattern = "SOURce:CURRent:LIMit",  .callback = scpi_etaCT_SetCurrentLim,},
    {.pattern = "SOURce:CURRent:LIMit?", .callback = scpi_etaCT_SetCurrentLimQ,},
    {.pattern = "OUTput:OFF",              .callback = scpi_etaCT_RangeCurrentOff,},  // this should be a parameter, not a command

    SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface = { .error = SCPI_Error, .write = SCPI_Write,
		.control = SCPI_Control, .flush = SCPI_Flush, .reset = SCPI_Reset, };

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;

