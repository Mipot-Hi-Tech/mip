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
#include "mip_d.h"
#include "mip_d_def.h"
#include "portable.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPD_TASK_PRIOTITY 3
#define MIPD_SINGLE_CORE
/* With single core Mip Series you need to drive nWAKE pin to put the module into low power consumption */
#ifdef MIPD_SINGLE_CORE
#define nWAKE_LOW_MIPD NWAKE_LOW
#define nWAKE_HIGH_MIPD NWAKE_HIGH
#else
#define nWAKE_LOW_MIPD
#define nWAKE_HIGH_MIPD
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipdInit(struct mip_d *const dev);
static void MipdApp(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_d mipd;
struct d_radio_phy_t mipd_radio_py;
struct d_module_param_t mipd_config;
TaskHandle_t mipd_device_task_handle;
uint8_t ndata_indicate_app_mipd;

/*******************************************************************************
 * Extern
 ******************************************************************************/
extern uint8_t ndata_indicate_event;

/*******************************************************************************
 * Code
 ******************************************************************************/
void MipdTask(void)
{
	enum mip_error_t retval;
	(void)MipdInit(&mipd);
	retval = mipd_null_ptr_check(&mipd);
	if(retval != no_error)
		return;
	mipd.hardware_init_fn(UartBaudrate_9600);
	mipd.hardware_reset_fn();
	nWAKE_LOW_MIPD
	mipd.delay_ms_fn(100);
	retval = mipd_get_fw_version(&mipd);
	if(retval != no_error)
	{
		nWAKE_HIGH_MIPD
		return;
	}
	retval += mipd_get_serial_no(&mipd);
	retval += mipd_eeprom_write_module_parameters(&mipd);
	retval += mipd_eeprom_read_module_parameters(&mipd, &mipd_config);
	retval += mipd_eeprom_write_radio_phy_param(&mipd);
	retval += mipd_eeprom_read_radio_phy_param(&mipd, &mipd_radio_py);
	nWAKE_HIGH_MIPD
	if(retval != no_error)
		return;
	(void)xTaskCreate(MipdApp, "MipdApp", 100, NULL, MIPD_TASK_PRIOTITY, &mipd_device_task_handle);
}

static void MipdApp(void *pvParameters)
{
	const TickType_t xDelayTxMsg = pdMS_TO_TICKS(1000);
	const uint8_t test_msg[] = {0x01, 0x02, 0x03, 0x04};
	for(;;)
	{
		if(ndata_indicate_app_mipd != ndata_indicate_event)
		{
			/* Handle received message */
			mipd_receive_message(&mipd);
			ndata_indicate_app_mipd = ndata_indicate_event;
		}
		/* Trasmit a message every xDelayTxMsg ms */
		nWAKE_HIGH_MIPD
		(void)mipd_tx_msg_cmd(test_msg, 4, &mipd);
		nWAKE_LOW_MIPD
		ndata_indicate_app_mipd = ndata_indicate_event;
		(void)vTaskDelay(xDelayTxMsg);
	}
	(void)vTaskDelete(NULL);
}

static void MipdInit(struct mip_d *const dev)
{
	dev->hardware_init_fn                   = Mip_Hardware_Init;
	dev->send_and_receive_fn                = MipTransmitAndReceiveData;
	dev->receive_fn                         = MipReceiveData;
	dev->hardware_reset_fn                  = MipHardwareReset;
	dev->delay_ms_fn                        = HAL_Delay;
	dev->radio_phy.power                    = MIPD_POWER_DEF;
	dev->radio_phy.frequency_channel        = MIPD_CHANNEL_FREQUENCY_DEF;
	dev->radio_phy.bandwidth                = MIPD_CHANNEL_BANDWIDTH_DEF;
	dev->radio_phy.spreading_factor         = MIPD_SPREADING_FACTOR_DEF;
	dev->radio_phy.code_rate                = MIPD_CODE_RATE_DEF;
	dev->module_param.DATA_INDICATE_TIMEOUT = DATA_IND_TIMEOUT_DEF_VAL;
	dev->module_param.UartBaudrate          = MIPD_UartBaudrate_DEF_VAL;
	dev->module_param.AppEnAes              = MIPD_AppEnAes_DEF_VAL;
}
