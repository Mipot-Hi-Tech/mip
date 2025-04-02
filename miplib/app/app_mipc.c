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

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <string.h>
#include "mip_c.h"
#include "mip_c_def.h"
#include "app_mipc.h"
#include "portable.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPC_TASK_PRIOTITY 3
#define MIPC_APP_SLAVE
#define MIPC_APP_MSG_LEN 4
#define MIPC_SINGLE_CORE
/* With single core Mip Series you need to drive nWAKE pin to put the module into low power consumption */
#ifdef MIPC_SINGLE_CORE
#define nWAKE_LOW_MIPC NWAKE_LOW
#define nWAKE_HIGH_MIPC NWAKE_HIGH
#else
#define nWAKE_LOW_MIPC
#define nWAKE_HIGH_MIPC
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipcInit(struct mip_c *const dev);
static void MipcAppMaster(void *pvParameters);
static void MipcAppSlave(void *pvParameters);
static void DoSomething(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_c mipc;
TaskHandle_t mipc_device_task_handle;
uint8_t ndata_indicate_app_mipc;

/*******************************************************************************
 * Extern
 ******************************************************************************/
extern uint8_t ndata_indicate_event;

/*******************************************************************************
 * Code
 ******************************************************************************/
void MipcTask(void)
{
#ifdef MIPC_APP_SLAVE
	xTaskCreate(MipcAppSlave,"MipcAppSlave",100, NULL,  MIPC_TASK_PRIOTITY, &mipc_device_task_handle);
#else
	xTaskCreate(MipcAppMaster,"MipcAppMaster",100, NULL, MIPC_TASK_PRIOTITY, &mipc_device_task_handle);
#endif
}

static void MipcAppMaster(void *pvParameters)
{
	enum mip_error_t retval;
	const uint8_t msg[] = {0x00, 0x01, 0x02, 0x03, 0x04};

	(void)MipcInit(&mipc);
	retval = mipc_null_ptr_check(&mipc);
	if(retval != no_error)
		(void)vTaskDelay(portMAX_DELAY);
	mipc.hardware_init_fn(UartBaudrate_115200);
	mipc.hardware_reset_fn();
	nWAKE_LOW_MIPC
	mipc.delay_ms_fn(100);
	retval = mipc_get_fw_version(&mipc);
	if(retval != no_error)
	{
		nWAKE_HIGH_MIPC
		(void)vTaskDelay(portMAX_DELAY);
	}
	/* Set device to master */
	mipc.stack_param.device_type = master;

	retval += mipc_factory_reset(&mipc);
	retval += mipc_get_serial_no(&mipc);

	retval += mipc_eeprom_write_stack_parameters(&mipc);
	retval += mipc_eeprom_write_radio_phy_param(&mipc);
	retval += mipc_eeprom_write_module_parameters(&mipc);
	/* Master enable pairing procedure */
	retval += mipc_enable_pairing(&mipc);
	nWAKE_HIGH_MIPC
	ndata_indicate_app_mipc = ndata_indicate_event;
	for(;;)
	{
		if(ndata_indicate_app_mipc != ndata_indicate_event)
		{
			/* Handle spontaneus messages from module */
			mipc_handle_IND_messages(&mipc, 100);
		}
		if(mipc.master_data.ROUTING_SIZE != 0)
		{
			/* MASTER SEND Message (0xFFFFFFFF = Broadcast Message) */
			nWAKE_LOW_MIPC
			mipc_tx_msg_cmd(UnconfirmedDataTransmission, 0xFFFFFFFF, msg, MIPC_APP_MSG_LEN, &mipc);
			nWAKE_HIGH_MIPC
		}
		ndata_indicate_app_mipc = ndata_indicate_event;
		(void)vTaskDelay(pdMS_TO_TICKS(500));
	}
	(void)vTaskDelete(NULL);
}

static void MipcAppSlave(void *pvParameters)
{
	const TickType_t xDelayPairingRequest = pdMS_TO_TICKS(20000);
	enum mip_error_t retval;
	enum mipc_app_t mipc_app_enode = mipc_app_check_activation;
	uint8_t enode_pairing_req = 0;
	uint8_t enode_msg_cnt     = 0;

	(void)MipcInit(&mipc);
	retval = mipc_null_ptr_check(&mipc);
	if(retval != no_error)
		(void)vTaskDelay(portMAX_DELAY);
	mipc.hardware_init_fn(UartBaudrate_115200);
	mipc.hardware_reset_fn();
	nWAKE_LOW_MIPC
	mipc.delay_ms_fn(100);
	retval = mipc_get_fw_version(&mipc);
	if(retval != no_error)
	{
		nWAKE_HIGH_MIPC
		(void)vTaskDelay(portMAX_DELAY);
	}
	/* Set device to end node */
	mipc.stack_param.device_type = end_node;

	retval += mipc_factory_reset(&mipc);
	retval += mipc_get_serial_no(&mipc);

	retval += mipc_eeprom_write_stack_parameters(&mipc);
	retval += mipc_eeprom_write_radio_phy_param(&mipc);
	retval += mipc_eeprom_write_module_parameters(&mipc);
	retval += mipc_eeprom_write_module_parameters(&mipc);
	/* End node sends pairing req to master */
	retval += mipc_pairing_req_cmd(&mipc);
	nWAKE_HIGH_MIPC
	ndata_indicate_app_mipc = ndata_indicate_event;
	for(;;)
	{
		if(ndata_indicate_app_mipc != ndata_indicate_event)
		{
			/* Handle spontaneus messages from module */
			mipc_handle_IND_messages(&mipc, 100);
			ndata_indicate_app_mipc = ndata_indicate_event;
		}

		switch(mipc_app_enode)
		{
			case mipc_app_check_activation:
			{
				/* Check if the end node is alredy paired */
				nWAKE_LOW_MIPC
				(void)mipc_get_activation_status(&mipc);
				nWAKE_HIGH_MIPC
				if(mipc.end_node_data.is_paired)
					mipc_app_enode = mipc_app_receive_message;
				else
				{
					mipc_app_enode = mipc_app_pairing_request;
					ndata_indicate_app_mipc = ndata_indicate_event;
					(void)vTaskDelay(xDelayPairingRequest);
				}
				break;
			}

			case mipc_app_pairing_request:
			{
				if(enode_pairing_req >= MIPC_APP_MAX_PAIRING_REQ_RETRY)
					mipc_app_enode = mipc_app_error;
				else
				{
					/* Perform a pairing request to the master */
					nWAKE_LOW_MIPC
					(void)mipc_pairing_req_cmd(&mipc);
					nWAKE_HIGH_MIPC
					enode_pairing_req++;
					mipc_app_enode = mipc_app_check_activation;
				}
				break;
			}

			case mipc_app_receive_message:
			/* Check periodically for incoming messages */
			{
				if(mipc.rx_data.last_rx_msg_len == MIPC_APP_MSG_LEN)
				{
					if( (mipc.rx_data.last_rx_msg[0] == 0x00) && (mipc.rx_data.last_rx_msg[1] == 0x01) )
					{
						if(enode_msg_cnt != mipc.rx_data.rx_msg_num)
						{
							DoSomething();
							enode_msg_cnt = mipc.rx_data.rx_msg_num;
						}
					}
				}
				ndata_indicate_app_mipc = ndata_indicate_event;
				(void)vTaskDelay(pdMS_TO_TICKS(25));
				break;
			}

			case mipc_app_error:
			{
				/* Wait indefinitely in case of error */
				(void)vTaskDelay(portMAX_DELAY);
				break;
			}

			default:
			{
				mipc_app_enode = mipc_app_check_activation;
				break;
			}
		}
	}
	(void)vTaskDelete(NULL);
}

static void MipcInit(struct mip_c *const dev)
{
	dev->hardware_init_fn                   = Mip_Hardware_Init;
	dev->send_and_receive_fn                = MipTransmitAndReceiveData;
	dev->receive_fn                         = MipReceiveData;
	dev->delay_ms_fn                        = Delay_ms;
	dev->hardware_reset_fn                  = MipHardwareReset;
	/* Default stack parameters */
	dev->stack_param.UnconfirmedTxNumber    = MIPC_UnconfirmedTxNumber_MIN_VAL;
	dev->stack_param.ConfirmedTxNumber      = MIPC_ConfirmedTxNumber_DEF;
	/* Default radio parameters */
	dev->radio_phy_param.power              = MIPC_POWER_DEF;
	dev->radio_phy_param.frequency_channel  = MIPC_CHANNEL_FREQUENCY_DEF;
	dev->radio_phy_param.RSSI_Th            = MIPC_RSSI_Th_DEF;
	/* Default module parameters */
	dev->module_param.DATA_INDICATE_TIMEOUT = DATA_IND_TIMEOUT_DEF_VAL;
	dev->module_param.UartBaudrate          = MIPC_UartBaudrate_DEF_VAL;
	dev->module_param.AppEnAes              = MIPC_AppEnAes_DEF_VAL;
	/* master specific */
	/* end node specific */
	dev->end_node_data.is_paired            = false;
	dev->link_check.result                  = LinkCheckNotPerformed;
	dev->link_check.power                   = power_level_11dBm; /* recommended value */
	dev->link_check.MessageNumber           = 5;                 /* recommended value */
	dev->link_check.MessageTh               = 4;                 /* recommended value */
}

static void DoSomething(void)
{
	;
}
