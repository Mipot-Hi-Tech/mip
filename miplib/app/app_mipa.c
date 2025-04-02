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
#include "mip_a.h"
#include "mip_a_def.h"
#include "portable.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPA_TASK_PRIOTITY 3
#define MIPA_SINGLE_CORE
/* With single core Mip Series you need to drive nWAKE pin to put the module into low power consumption */
#ifdef MIPA_SINGLE_CORE
#define nWAKE_LOW_MIPA NWAKE_LOW
#define nWAKE_HIGH_MIPA NWAKE_HIGH
#else
#define nWAKE_LOW_MIPA
#define nWAKE_HIGH_MIPA
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipaInit(struct mip_a *const dev);
static void MipaAppStationaryReceiver(void *pvParameters);
static void MipaAppTransmitterDevice(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_a mipa;
struct a_radio_phy_t mipa_radio_py;
struct a_module_param_t mipa_config;
struct a_medium_access_param_t mipa_medium_access_param;
TaskHandle_t mipa_device_task_handle;
uint8_t ndata_indicate_app_mipa;

/*******************************************************************************
 * Extern
 ******************************************************************************/
 extern uint8_t ndata_indicate_event;
 
/*******************************************************************************
 * Code
 ******************************************************************************/
void MipaTask(void)
{
#ifdef MIPA_STATIONARY_RECEIVER
	(void)xTaskCreate(MipaAppStationaryReceiver, "Receiver", 100, NULL, MIPA_TASK_PRIOTITY, &mipa_device_task_handle);
#else
	(void)xTaskCreate(MipaAppTransmitterDevice, "Transmitter", 100, NULL, MIPA_TASK_PRIOTITY, &mipa_device_task_handle);
#endif
}

static void MipaAppStationaryReceiver(void *pvParameters)
{
	const TickType_t xDelayReceiver = pdMS_TO_TICKS(100);
	enum mip_error_t retval;
	(void)MipaInit(&mipa);
	retval = mipa_null_ptr_check(&mipa);
	if(retval != no_error)
	{
		(void)vTaskDelay(portMAX_DELAY);
	}
	mipa.hardware_init_fn(UartBaudrate_115200);
	mipa.hardware_reset_fn();
	nWAKE_LOW_MIPA
	mipa.delay_ms_fn(100);
	retval = mipa_get_fw_version(&mipa);
	if(retval != no_error)
	{
		nWAKE_HIGH_MIPA
		(void)vTaskDelay(portMAX_DELAY);
	}
	retval += mipa_get_serial_no(&mipa);
	retval += mipa_eeprom_write_module_parameters(&mipa);
	retval += mipa_eeprom_read_module_parameters(&mipa, &mipa_config);
	retval += mipa_eeprom_write_radio_phy_param(&mipa);
	retval += mipa_eeprom_read_radio_phy_param(&mipa, &mipa_radio_py);
	retval += mipa_eeprom_write_wmbus_medium_access_parameters(&mipa);
	retval += mipa_eeprom_read_wmbus_medium_access_parameters(&mipa, &mipa_medium_access_param);
	nWAKE_HIGH_MIPA
	ndata_indicate_app_mipa = ndata_indicate_event;
	for(;;)
	{
		if(ndata_indicate_app_mipa != ndata_indicate_event)
		{
			/* Message is coming from the module */
			(void)mipa_rx_msg_IND(&mipa, 100);
			ndata_indicate_app_mipa = ndata_indicate_event;
		}
		(void)vTaskDelay(xDelayReceiver);
	}
	(void)vTaskDelete(NULL);
}

static void MipaAppTransmitterDevice(void *pvParameters)
{
	const TickType_t xDelayTxMsg    = pdMS_TO_TICKS(5000);
	const uint8_t test_msg[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
	enum mip_error_t retval;
	(void)MipaInit(&mipa);
	retval = mipa_null_ptr_check(&mipa);
	if(retval != no_error)
	{
		(void)vTaskDelay(portMAX_DELAY);
	}
	mipa.hardware_init_fn(UartBaudrate_115200);
	mipa.hardware_reset_fn();
	nWAKE_LOW_MIPA
	mipa.delay_ms_fn(100);
	retval = mipa_get_fw_version(&mipa);
	if(retval != no_error)
	{
		nWAKE_HIGH_MIPA
		(void)vTaskDelay(portMAX_DELAY);
	}
	retval += mipa_get_serial_no(&mipa);
	retval += mipa_eeprom_write_module_parameters(&mipa);
	retval += mipa_eeprom_read_module_parameters(&mipa, &mipa_config);
	retval += mipa_eeprom_write_radio_phy_param(&mipa);
	retval += mipa_eeprom_read_radio_phy_param(&mipa, &mipa_radio_py);
	retval += mipa_eeprom_write_wmbus_medium_access_parameters(&mipa);
	retval += mipa_eeprom_read_wmbus_medium_access_parameters(&mipa, &mipa_medium_access_param);
	if(retval != no_error)
		(void)vTaskDelay(portMAX_DELAY);
	for(;;)
	{
		/* Trasmit a message every xDelayTxMsg ms */
		nWAKE_LOW_MIPA
		(void)mipa_tx_msg_cmd(test_msg, 10, &mipa);
		nWAKE_HIGH_MIPA
		(void)vTaskDelay(xDelayTxMsg);
	}
	(void)vTaskDelete(NULL);
}

static void MipaInit(struct mip_a *const dev)
{
	dev->hardware_init_fn                    = Mip_Hardware_Init;
	dev->send_and_receive_fn                 = MipTransmitAndReceiveData;
	dev->receive_fn                          = MipReceiveData;
	dev->delay_ms_fn                         = Delay_ms;
	dev->hardware_reset_fn                   = MipHardwareReset;
	/* Setup radio parameters */
	dev->radio_phy.wmbusmode                 = MIPA_WMBUS_MODE_DEF;
	dev->radio_phy.frequency_channel         = MIPA_RF_CHANNEL_DEF;
	dev->radio_phy.power                     = MIPA_POWER_DEF;
	dev->radio_phy.rfautosleep               = MIPA_RF_AUTOSLEEP_DEF_VAL;
	dev->radio_phy.rx_window                 = MIPA_RX_WINDOW_DEF_VAL;
	/* Setup module parameters */
	dev->module_param.block1                 = MIPA_BLOCK1_FROM_MODULE_DEF_VAL;
	dev->module_param.rssi                   = MIPA_RSSI_DEF_VAL;
	dev->module_param.ndata_indicate_timeout = MIPA_NDATA_IND_TIMEOUT_DEF_VAL;
	dev->module_param.UartBaudrate           = MIPA_UART_BAUDRATE_DEF_VAL;
	/* Setup medium access parameters */
	dev->medium_access_param.c_field         = 0x44;
	dev->medium_access_param.manufacturer_id = 0x1234;
	dev->medium_access_param.device_id       = 0x12345678;
	dev->medium_access_param.version         = 0x01;
	dev->medium_access_param.device_type     = 0x01;
}
