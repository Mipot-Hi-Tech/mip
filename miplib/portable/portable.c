/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "portable.h"
#include <string.h>
#ifdef MIP_EVK_HARDWARE
#include "sys_task.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART2MIP_TX_SZ 300u
#define UART2MIP_RX_SZ 300u

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MIP_TIM_Init(void);
static void MIP_GPIO_Init(void);
static void MIP_UART_Init(uint32_t baud);
static void SerStartTransmit(void);
static ser_error_t SerCheckErrors(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart2;
uint8_t ser_tx_buff[UART2MIP_TX_SZ];
uint16_t tx_buff_write_index;
uint16_t tx_buff_read_index;
uint8_t ser_rx_buff[UART2MIP_RX_SZ];
uint16_t rx_buff_write_index;
uint16_t rx_buff_read_index;
TxRx_Handler_t mip_txrx_handler;
Rx_Handler_t mip_rx_handler;
uint32_t time;
uint8_t ndata_indicate_event;
uint32_t tick_cnt;

/*******************************************************************************
 * Code
 ******************************************************************************/

void Mip_Hardware_Init(enum UartBaudrate_t baud)
{
	MIP_TIM_Init();
	MIP_GPIO_Init();
	if(baud == UartBaudrate_9600)
	{
		MIP_UART_Init(9600);
	}
	else
	{
		MIP_UART_Init(115200);
	}
}

static void MIP_TIM_Init(void) /* 1 mS interrupt */
{
#ifdef STM32F0
	RCC->APB1ENR                 |= RCC_APB1ENR_TIM14EN;
	htim14.Instance               = TIM14;
	htim14.Init.Prescaler         = 1;
	htim14.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim14.Init.Period            = 8000;
	htim14.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim14);
	HAL_NVIC_EnableIRQ(TIM14_IRQn);
	/* clear update flag */
	TIM14->SR &= ~TIM_SR_UIF;
	/* Update interrupt enabled */
	TIM14->DIER |= TIM_DIER_UIE;
	/* Counter enabled */
	TIM14->CR1 |= TIM_CR1_CEN;
#else
#error
#endif
}

static void MIP_GPIO_Init(void)
{
#ifdef STM32F0
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NWAKE_GPIO_Port, NWAKE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : NDATA_INDICATE_Pin */
    GPIO_InitStruct.Pin  = NDATA_INDICATE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NDATA_INDICATE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RESET_Pin */
	GPIO_InitStruct.Pin   = RESET_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NWAKE_Pin */
	GPIO_InitStruct.Pin   = NWAKE_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NWAKE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : UART_TX_MIP_PIN */
	GPIO_InitStruct.Pin   = UART_TX_MIP_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(UART_TX_MIP_PORT, &GPIO_InitStruct);

	/*Configure GPIO pin : UART_RX_MIP_PIN */
	GPIO_InitStruct.Pin       = UART_RX_MIP_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(UART_RX_MIP_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
#else
#error
#endif
}

static void MIP_UART_Init(uint32_t baud)
{
#ifdef STM32F0
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	huart2.Instance = USART2;
	huart2.Init.BaudRate = baud;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart2);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#else
#error
#endif
}

enum mip_error_t MipTransmitAndReceiveData(uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms)
{
	enum mip_error_t retval = unknown_error;
	mip_txrx_handler = TXRX_HANDLER_IDLE;
	uint16_t i;
	uint16_t mip_rx_sz;
	while ( (mip_txrx_handler != TXRX_HANDLER_COMPLETED) && (mip_txrx_handler != TXRX_HANDLER_ERROR) )
	{
		switch(mip_txrx_handler)
		{
			case TXRX_HANDLER_IDLE:
			{
				SerCheckErrors();
				tx_buff_read_index  = 0;
				tx_buff_write_index = 0;
				rx_buff_read_index  = 0;
				rx_buff_write_index = 0;
				mip_txrx_handler = TXRX_HANDLER_TX_DATA_TRANSMISSION;
				break;
			}

			case TXRX_HANDLER_TX_DATA_TRANSMISSION:
			{
				for(i=0; i<tx_dim; i++)
					ser_tx_buff[i] = tx_buff[i];
				tx_buff_write_index = tx_dim;
				SerStartTransmit();
				time = GetTick();
				mip_txrx_handler = TXRX_HANDLER_TX_WAIT_TRANSMISSION;
				break;
			}

			case TXRX_HANDLER_TX_WAIT_TRANSMISSION:
			{
				if( GetTick() - time >= MIP_UART_TX_TIMEOUT)
				{
					retval =  tx_timeout;
					mip_txrx_handler = TXRX_HANDLER_ERROR;
				}
				if(tx_buff_write_index == 0)
				{
					time = GetTick();
					mip_txrx_handler = TXRX_HANDLER_RX_DATA_WAIT;
				}
				break;
			}

			case TXRX_HANDLER_RX_DATA_WAIT:
			{
				if ( GetTick() - time >= timeout_ms)
				{
					retval = rx_timeout;
					mip_txrx_handler = TXRX_HANDLER_ERROR;
				}
				if(rx_buff_write_index != 0)
				{
					mip_rx_sz = rx_buff_write_index;
					time = GetTick();
					mip_txrx_handler = TXRX_HANDLER_RX_DATA_WAIT_COMPLETE;
				}
				break;
			}

			case TXRX_HANDLER_RX_DATA_WAIT_COMPLETE:
			{
				if ( GetTick() - time >= MIP_UART_END_OF_MSG_TIMEOUT)
				{
					mip_txrx_handler = TXRX_HANDLER_DATA_COPY;
				}
				/* Avoid timeout when receiving */
				if(mip_rx_sz != rx_buff_write_index)
				{
					mip_rx_sz = rx_buff_write_index;
					time = GetTick();
				}
				break;
			}

			case TXRX_HANDLER_DATA_COPY:
			{
				memcpy(rx_buff,ser_rx_buff,rx_buff_write_index);
				*rx_dim = rx_buff_write_index;
				rx_buff_write_index = 0;
				retval = no_error;
				mip_txrx_handler = TXRX_HANDLER_COMPLETED;
				break;
			}

			case TXRX_HANDLER_COMPLETED:
			case TXRX_HANDLER_ERROR:
			{
				break;
			}

			default:
			{
				mip_txrx_handler = TXRX_HANDLER_IDLE;
				break;
			}
		}
	}
	return retval;
}

enum mip_error_t MipReceiveData(uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms)
{
	enum mip_error_t retval = unknown_error;
	mip_rx_handler = RX_HANDLER_IDLE;
	uint16_t mip_rx_sz;
	while ( (mip_rx_handler != RX_HANDLER_COMPLETED) && (mip_rx_handler !=RX_HANDLER_ERROR) )
	{
		switch(mip_rx_handler)
		{
			case RX_HANDLER_IDLE:
			{
				time = GetTick();
				mip_rx_handler = RX_HANDLER_DATA_WAIT;
				break;
			}

			case RX_HANDLER_DATA_WAIT:
			{
				if ( GetTick() - time >= timeout_ms)
				{
					retval = rx_timeout;
					mip_rx_handler = RX_HANDLER_ERROR;
				}
				if(rx_buff_write_index != 0)
				{
					mip_rx_sz = rx_buff_write_index;
					time = GetTick();
					mip_rx_handler = RX_HANDLER_DATA_WAIT_COMPLETE;
				}
				break;
			}

			case RX_HANDLER_DATA_WAIT_COMPLETE:
			{
				if ( GetTick() - time >= MIP_UART_END_OF_MSG_TIMEOUT)
				{
					mip_rx_handler = RX_HANDLER_DATA_COPY;
				}
				/* Avoid timeout when receiving */
				if(mip_rx_sz != rx_buff_write_index)
				{
					mip_rx_sz = rx_buff_write_index;
					time = GetTick();
				}
				break;
			}

			case RX_HANDLER_DATA_COPY:
			{
				memcpy(rx_buff,ser_rx_buff,rx_buff_write_index);
				*rx_dim = rx_buff_write_index;
				rx_buff_write_index = 0;
				retval = no_error;
				mip_rx_handler = RX_HANDLER_COMPLETED;
				break;
			}

			case RX_HANDLER_COMPLETED:
			case RX_HANDLER_ERROR:
			{
				break;
			}

			default:
			{
				mip_txrx_handler = RX_HANDLER_IDLE;
				break;
			}
		}
	}
	return retval;
}

__WEAK void EXTI4_15_IRQHandler(void)
{
#ifdef STM32F0
	if(__HAL_GPIO_EXTI_GET_IT(NDATA_INDICATE_Pin) != 0x00u)
	{
		ndata_indicate_event++;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(NDATA_INDICATE_Pin);
#else
#error
#endif
}

void USART2_IRQHandler(void)
{
#ifdef STM32F0
	SerCheckErrors();
	/*Check Transmit register data empty flag & transmit it enable*/
	if ( ((USART2->ISR & USART_ISR_TXE) != 0 ) && ((USART2->CR1 & USART_CR1_TXEIE) != 0) )
	{
		USART2->TDR = (uint8_t) ser_tx_buff[tx_buff_read_index];
		tx_buff_read_index++;
		if (tx_buff_read_index >= tx_buff_write_index)
		{
			HAL_UART_AbortTransmit_IT(&huart2);
			tx_buff_read_index  = 0;
			tx_buff_write_index = 0;
		}
	}
	/*Check Read data register not empty flag*/
	if((USART2->ISR & USART_ISR_RXNE) != 0)
	{
		if(rx_buff_write_index < UART2MIP_RX_SZ)
		{
			ser_rx_buff[rx_buff_write_index] = (uint8_t)(USART2->RDR);
			rx_buff_write_index++;
#ifdef MIP_EVK_HARDWARE
			if ( (vcp_bridge_enable != 0) && (rx_buff_write_index == 0) )
			{
				vTaskNotifyGiveIndexedFromISR(vcp_bridge_task_handle, 0, pdFALSE);
			}
#endif
		}
		else
			USART2->RDR;
	}
#else
#error
#endif
}

__WEAK void TIM14_IRQHandler(void)
{
	tick_cnt++;
#ifdef STM32F0
	TIM14->SR &= ~TIM_SR_UIF;
#else
#error
#endif
}

void MipHardwareReset(void)
{
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	Delay_ms(100);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	Delay_ms(350);
}

static void SerStartTransmit(void)
{
#ifdef STM32F0
	USART2->CR1 |= USART_CR1_TXEIE;
#else
#error
#endif
}

uint32_t GetTick(void)
{
	return tick_cnt;
}

void Delay_ms(uint32_t ms)
{
	uint32_t ticks =  GetTick();
	while( (GetTick() - ticks) < ms);
}

static ser_error_t SerCheckErrors(void)
{
#ifdef STM32F0
	ser_error_t retval = SER_NO_ERROR;
	uint32_t sr = USART2->ISR;
	if (sr & USART_ISR_ORE)
	{
		retval |= SER_OVERRUN_ERROR;
		__HAL_UART_CLEAR_FLAG(&huart2,USART_ICR_ORECF);
	}
	if (sr & USART_ISR_NE)
	{
		retval |=  SER_NOISE_ERROR;
		__HAL_UART_CLEAR_FLAG(&huart2,USART_ICR_NCF);
	}
	if (sr & USART_ISR_FE)
	{
		retval |=  SER_FRAMING_ERROR;
		__HAL_UART_CLEAR_FLAG(&huart2,USART_ICR_FECF);
	}
	if (sr & USART_ISR_PE)
	{
		retval |=  SER_PARITY_ERROR;
		__HAL_UART_CLEAR_FLAG(&huart2,USART_ICR_PECF);
	}
	return retval;
#else
#error
#endif
}
