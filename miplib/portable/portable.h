/**
* Copyright (c) Mipot S.p.A. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file
* @date
* @version
*
*/

#ifndef MIP_UART_MIP_UART_H_
#define MIP_UART_MIP_UART_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "mip_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_TX_MIP_PIN          GPIO_PIN_2
#define UART_TX_MIP_PORT         GPIOA
#define UART_RX_MIP_PIN          GPIO_PIN_3
#define UART_RX_MIP_PORT         GPIOA
#define NDATA_INDICATE_Pin       GPIO_PIN_8
#define NDATA_INDICATE_GPIO_Port GPIOA
#define RESET_Pin                GPIO_PIN_6
#define RESET_GPIO_Port          GPIOB
#define NWAKE_Pin                GPIO_PIN_11
#define NWAKE_GPIO_Port          GPIOB
#define NWAKE_HIGH HAL_GPIO_WritePin(NWAKE_GPIO_Port, NWAKE_Pin, GPIO_PIN_SET)
#define NWAKE_LOW HAL_GPIO_WritePin(NWAKE_GPIO_Port, NWAKE_Pin, GPIO_PIN_RESET)

typedef enum{
       SER_NO_ERROR      = 0x00U,
       SER_OVERRUN_ERROR = 0x01U,
       SER_NOISE_ERROR   = 0x02U,
       SER_FRAMING_ERROR = 0x04U,
       SER_PARITY_ERROR  = 0x08U
}ser_error_t;

typedef enum
{
	TXRX_HANDLER_IDLE                  = 0x00U,
	TXRX_HANDLER_TX_DATA_TRANSMISSION  = 0x01U,
	TXRX_HANDLER_TX_WAIT_TRANSMISSION  = 0x02U,
	TXRX_HANDLER_RX_DATA_WAIT          = 0x03U,
	TXRX_HANDLER_RX_DATA_WAIT_COMPLETE = 0x04U,
	TXRX_HANDLER_DATA_COPY             = 0x05U,
	TXRX_HANDLER_COMPLETED             = 0x06U,
	TXRX_HANDLER_ERROR                 = 0x07U
}TxRx_Handler_t;

typedef enum
{
	RX_HANDLER_IDLE               = 0x00U,
	RX_HANDLER_DATA_WAIT          = 0x01U,
	RX_HANDLER_DATA_WAIT_COMPLETE = 0x02U,
	RX_HANDLER_DATA_COPY          = 0x03U,
	RX_HANDLER_COMPLETED          = 0x04U,
	RX_HANDLER_ERROR              = 0x05U
}Rx_Handler_t;

/*******************************************************************************
 * API
 ******************************************************************************/
enum mip_error_t MipTransmitAndReceiveData(uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
enum mip_error_t MipReceiveData(uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
void MipHardwareReset(void);
uint32_t GetTick(void);
void Delay_ms(uint32_t ms);
void Mip_Hardware_Init(enum UartBaudrate_t baud);

#endif
