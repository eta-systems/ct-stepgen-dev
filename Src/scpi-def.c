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

/**
  * @file   scpi-def.c
  * @date   2020-07-13
  * @author eta Systems GmbH
  * @author Simon Burkhardt
  * @brief  contains user settings and definitions for SCPI command implementation
  * @see    https://github.com/j123b567/scpi-parser/blob/master/examples/common/scpi-def.c
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

#include "max5717.h"
#include "2.476.101.01.BSP.h"

extern MAX5717_t dac1;

extern volatile float dacOutputVoltage;
extern volatile float adcInputVoltage;

/**
  * @brief  save and set a desired output voltage
	*/
static scpi_result_t scpi_etaCT_SetVoltage(scpi_t * context)
{
	double param1;
  // fprintf(stderr, "conf:volt:dc\r\n"); /* debug command name */
  /* read first parameter if present */
  if (!SCPI_ParamDouble(context, &param1, TRUE)) {
		return SCPI_RES_ERR;
	}
	
	dacOutputVoltage = (float)(param1);
	ETA_CTGS_VoltageOutputSet(&dac1, dacOutputVoltage );
	
  return SCPI_RES_OK;
}

/**
  * @brief  read back the stored voltage
	*/
static scpi_result_t scpi_etaCT_SetVoltageQ(scpi_t * context)
{
	double param1 = (double)(dacOutputVoltage);
	//SCPI_ResultDouble(context, param1);
	printf("%.4f\r\n", dacOutputVoltage);
  return SCPI_RES_OK;
}

/**
  * @brief  read back the last measured DC voltage
	*/
static scpi_result_t scpi_etaCT_MeasureVoltageQ(scpi_t * context)
{
	double param1 = (double)(adcInputVoltage);
	//SCPI_ResultDouble(context, param1);
	printf("%.4f\r\n", adcInputVoltage);
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
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    /* {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,}, */

    /* DMM */
    {.pattern = "MEASure:VOLTage[:DC]?", .callback = scpi_etaCT_MeasureVoltageQ,},
    {.pattern = "MEASure:CURRent[:DC]?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:RESistance?", .callback = SCPI_StubQ,},
		
		/* SOURCE */
    {.pattern = "SOURce:VOLTage[:LEVel]", .callback = scpi_etaCT_SetVoltage,},
    {.pattern = "SOURce:VOLTage[:LEVel]?", .callback = scpi_etaCT_SetVoltageQ,},
    {.pattern = "SOURce:CURRent[:LIMit]", .callback = SCPI_StubQ,},
    {.pattern = "SOURce:CURRent[:LIMit]?", .callback = SCPI_StubQ,},
		
		

    SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;



