/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f7xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max5717.h"
#include "ads1255.h"
#include "2.476.101.01.BSP.h"   // Baord Support Package
#include "scpi/scpi.h"
#include "scpi-def.h"
/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char rx;
ADS1255_DMA_State_t ads1256_state = DMA_STATE_Ready;
int32_t adsCode;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern MAX5717_t dac1;
extern ADS125X_t adcv;
extern ADS125X_t adci;

extern volatile uint8_t dmaDacTx[];
extern volatile uint16_t dmaPtr;
extern const uint16_t dmaBufferSize;

extern scpi_t scpi_context;
extern volatile CurveTracer_State_t deviceState;

extern volatile uint8_t is_config_done;

extern uint8_t TxDMAchannelCycle[];
extern uint8_t TxDMABufferOffsetStep;
extern uint8_t TxDMABufferOffset;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/** @note DRDY Falling Edge Interrupt
	 * PF7 for SPI3 (Voltage)
	 * PA9 for SPI1 (Current)
	 * DRDY low signals that a new sample is ready to be read
	 * for the current measurement, the channel also has to be switched
	 */
	if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_9)) {
		/** @note DRDY for current ADC
		 * only execute, when not changing channels
		 */
		if (is_config_done) {
			switch (ads1256_state) {
			case DMA_STATE_Ready:
				ads1256_state = DMA_STATE_Tx_MUX;
				for (uint16_t i = 0; i < 0xff; i++)
					__NOP();
				/** Send first 4 bytes for the MUX register */
				HAL_SPI_Transmit_DMA(adci.hspix,
						(uint8_t*) &TxDMAchannelCycle[0 + TxDMABufferOffset],
						4); // MUX
				break;
			default:
				break;
				ads1256_state = DMA_STATE_Ready;
			}
		}
	}
	if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_7)) {
		/** @note DRDY for voltage ADC
		 */
		static int32_t pCode;
		static float pVolt;
		static uint8_t spiTmp[3];

		// only execute IRQ if ADC and all peripherals have been initialized
		if (is_config_done) {
			spiTmp[0] = ADS125X_CMD_RDATA;    // request Data read

			ADS125X_CS(&adcv, 1);   // Chip Select
			HAL_SPI_Transmit(adcv.hspix, spiTmp, 1, 1);
			for (uint16_t i = 0; i < 0xfff; i++)
				__NOP();
			// delay
			HAL_SPI_Receive(adcv.hspix, spiTmp, 3, 1);
			ADS125X_CS(&adcv, 0);   // Chip Un-Select

			// Convert to float voltage
			// must be signed integer for 2's complement to work
			pCode = (spiTmp[0] << 16) | (spiTmp[1] << 8) | (spiTmp[2]);
			if (pCode & 0x800000)
				pCode |= 0xff000000;  // fix 2's complement
			ADS125X_ADC_Code2Volt(&adcv, &pCode, &pVolt, 1);
			pVolt *= 2.0f; /** @todo voltage is wrong by a factor of 2 - why? */
			// printf("%.4f\t", pVolt);
			pVolt = ETA_CTGS_GetVoltageSense(pVolt);

			deviceState.adcInputVoltage = pVolt; // save to device state
			//printf("%.4f\n", pVolt);
		}
	}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

	/**
	 * @note USART (SCPI) Interface Rx Interrupt
	 * this function is executed for each byte/char entering the USART3 Bus
	 * the char can be directly forwarded to the SCPI_Input()
	 * the SCPI library has a buffer and automatically detects complete command sequences
	 */
	if (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) == SET) {
		HAL_UART_Receive_IT(&huart3, (uint8_t*) &rx, 1); // receive the single char in non-blocking mode
		SCPI_Input(&scpi_context, &rx, 1);
	}

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);  // must be enabled again

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	// SPI4 DMA Complete
	/**
	 * @note DAC DMA Handler. (Part 2/2)
	 * this interrupt handler is executed on DMA Tx Complete
	 * here the CS pin and the LATCH pin need to be toggled
	 */
	for (uint16_t i = 0; i < 0x1fff; i++)
		;  // delay
	HAL_GPIO_WritePin(dac1.csPort, dac1.csPin, GPIO_PIN_SET);

	// latch sample to DAC output
	HAL_GPIO_WritePin(dac1.latchPort, dac1.latchPin, GPIO_PIN_RESET);
	for (uint16_t i = 0; i < 128; i++)
		;    // delay
	HAL_GPIO_WritePin(dac1.latchPort, dac1.latchPin, GPIO_PIN_SET);

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi4_tx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles SPI4 global interrupt.
  */
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */

  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief SPI DMA Transmit Complete Interrupt
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == adci.hspix) {
		if (is_config_done) {
			static uint8_t dmaRx[3];
			static float volt2;

			switch (ads1256_state) {
			case DMA_STATE_Tx_MUX:
				ads1256_state = DMA_STATE_Tx_WKUP;
				for (uint16_t i = 0; i < 0xff; i++)
					;
				/** send WAKEUP Command */
				HAL_SPI_Transmit_DMA(adci.hspix,
						(uint8_t*) &TxDMAchannelCycle[4 + TxDMABufferOffset],
						1); // WAKEUP
				break;

			case DMA_STATE_Tx_WKUP:
				ads1256_state = DMA_STATE_Tx_RDATA;
				for (uint16_t i = 0; i < 0xff; i++)
					;
				/** send Request for Data Read */
				HAL_SPI_Transmit_DMA(adci.hspix,
						(uint8_t*) &TxDMAchannelCycle[5 + TxDMABufferOffset],
						1); // RDATA
				break;

			case DMA_STATE_Tx_RDATA:
				ads1256_state = DMA_STATE_Rx_ADC;
				for (uint16_t i = 0; i < 128; i++)
					;
				/** Receive 3 bytes */
				HAL_SPI_Receive_DMA(adci.hspix, (uint8_t*) &dmaRx[0], 3); // RDATA

				adsCode = (dmaRx[0] << 16) | (dmaRx[1] << 8) | (dmaRx[2]);
				if (adsCode & 0x800000)
					adsCode |= 0xff000000;  // fix 2's complement
				volt2 = ((float) adsCode * (2.0f * adci.vref))
						/ (adci.pga * 8388607.0f);  // 0x7fffff = 8388607.0f

				/** @todo Vhi and Vlo are swapped for some reason ?? */
				// shift buffer offset for other MUX bytes
				if (TxDMABufferOffset == 0) {
					TxDMABufferOffset = TxDMABufferOffsetStep;
					// this means the just sampled value is for ADS125X_MUXP_AIN2 | ADS125X_MUXN_AIN3 = Vlo
					deviceState.adcVhi = volt2;
				} else {
					TxDMABufferOffset = 0;
					// this means the just sampled value is for ADS125X_MUXP_AIN4 | ADS125X_MUXN_AIN5 = Vhi
					deviceState.adcVlo = volt2;
					deviceState.adcInputCurrent = ETA_CTGS_GetCurrentSense(deviceState.adcVhi,
							deviceState.adcVlo, RANGE_5mA);
				}
				ads1256_state = DMA_STATE_Ready;
				break;

			case DMA_STATE_Rx_ADC:
			default:
				ads1256_state = DMA_STATE_Ready;
			}
		}
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
