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
