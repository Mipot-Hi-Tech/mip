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
* @file       mip_b.c
* @date
* @version
*
*/

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <string.h>
#include "mip_b.h"
#include "mip_b_def.h"
#include "app_mipb.h"
#include "portable.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPB_TASK_PRIOTITY 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipbInit(struct mip_b *const dev);
static void MipbApp(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_b mipb;
struct b_stack_param_t mipb_stack_param;
TaskHandle_t mipb_device_task_handle;
const uint8_t test_msg[]     = {0x01, 0x02, 0x03, 0x04};
const uint8_t JoinEUI_mipb[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t DevEUI_mipb[]  = {0xDB, 0xAC, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
const uint8_t AppKey_mipb[]  = {0xE9, 0x12, 0xD1, 0xDB, 0x5E, 0x9F, 0x17, 0x85, 0x0E, 0xDC, 0x55, 0x60, 0xA5, 0x37, 0x49, 0xCF};
uint8_t ndata_indicate_app_mipb;
extern uint8_t ndata_indicate_event;

/*******************************************************************************
 * Code
 ******************************************************************************/
void MipbTask(void)
{
	enum mip_error_t retval;
	(void)MipbInit(&mipb);
	retval = mipb_null_ptr_check(&mipb);
	if(retval != no_error)
		return;
	mipb.hardware_init_fn(UartBaudrate_115200);
	mipb.delay_ms_fn(100);
	mipb.hardware_reset_fn();
	retval  = mipb_factory_reset(&mipb);
	retval += mipb_enable_ndata_indicate_pin(&mipb);
	retval += mipb_get_fw_version(&mipb);
	retval += mipb_get_serial_no(&mipb);
	retval += mipb_eeprom_write_network_parameters(&mipb);
	retval += mipb_eeprom_read_network_parameters(&mipb, &mipb_stack_param);
	retval += mipb_eeprom_write_stack_parameters(&mipb);
	retval += mipb_eeprom_read_stack_parameters(&mipb, &mipb_stack_param);
	retval += mipb_set_app_key(AppKey_mipb, &mipb);
	retval += mipb_join(Join_mode_OTAA, &mipb);
	retval += mipb_get_activation_status(&mipb);
	if(retval != no_error)
		return;
	(void)xTaskCreate(MipbApp,"MipbApp",100, NULL,  MIPB_TASK_PRIOTITY, &mipb_device_task_handle);
}

static void MipbApp(void *pvParameters)
{
	const TickType_t xDelayTxMsg = pdMS_TO_TICKS(10000);
	const TickType_t xDelayJoin  = pdMS_TO_TICKS(20000);
	enum mip_error_t retval;
	enum mipb_app_t app = mipb_app_check_activation;
	uint8_t join_num    = 0;
	for(;;)
	{
		if(ndata_indicate_app_mipb != ndata_indicate_event)
		{
			/* Handle spontaneus messages from module */
			mipb_handle_IND_messages(&mipb, 100);
			ndata_indicate_app_mipb = ndata_indicate_event;
		}
		switch(app)
		{
			case mipb_app_idle:
			break;

			case mipb_app_check_activation:
			{
				/* Check if device has succesfully joined the Network */
				retval = mipb_get_activation_status(&mipb);
				if(mipb.join_status == Joined)
				{
					app = mipb_app_send_message;
				}
				else if(mipb.join_status == Joining)
				{
					ndata_indicate_app_mipb = ndata_indicate_event;
					(void)vTaskDelay(pdMS_TO_TICKS(5000));
				}
				else if(mipb.join_status == MAC_error)
				{
					app = mipb_app_error;
				}
				else if (mipb.join_status == Device_not_activated)
				{
					ndata_indicate_app_mipb = ndata_indicate_event;
					(void)vTaskDelay(xDelayJoin);
					app = mipb_app_join;
				}
				else
				{
					app = mipb_app_error;
				}
				break;
			}

			case mipb_app_join:
			{
				/* OTAA join */
				if(join_num >= MIPB_APP_MAX_JOIN_RETRY)
					app = mipb_app_error;
				retval = mipb_join(Join_mode_OTAA, &mipb);
				join_num++;
				app = mipb_app_check_activation;
				break;
			}

			case mipb_app_send_message:
			{
				/* Send The message every xDelayTxMsg ms */
				retval = mipb_tx_msg_cmd(UnconfirmedDataTransmission, 1, test_msg, 4, &mipb);
				ndata_indicate_app_mipb = ndata_indicate_event;
				(void)vTaskDelay(xDelayTxMsg);
				break;
			}

			case mipb_app_error:
			{
				/* Wait indefinitely in case of error */
				(void)vTaskDelay(portMAX_DELAY);
				break;
			}

			default:
			{
				app = mipb_app_check_activation;
			}
		}
	}
	(void)vTaskDelete(NULL);
}

static void MipbInit(struct mip_b *const dev)
{
	uint8_t i;
	dev->hardware_init_fn                             = Mip_Hardware_Init;
	dev->send_and_receive_fn                          = MipTransmitAndReceiveData;
	dev->receive_fn                                   = MipReceiveData;
	dev->hardware_reset_fn                            = MipHardwareReset;
	dev->delay_ms_fn                                  = Delay_ms;
	dev->join_status                                  = Device_not_activated;
	dev->stack_param.class                            = MIPB_LORAWAN_CLASS_DEF;
	dev->stack_param.dr_sf                            = MIPB_DR_SF_DEF;
	dev->stack_param.power                            = MIPB_POWER_DEF;
	dev->stack_param.ADR                              = ADR_Enabled;
	dev->stack_param.Duty_Cycle_control               = DC_Enabled;
	dev->stack_param.Unconfirmed_TX_Repetition_number = MIPB_UNCONF_TX_NUM_DEF;
	dev->stack_param.Enable_Customer_EUI              = Customer_EUI_Disabled;
	dev->stack_param.RX2_Data_Rate                    = MIPB_RX2_DATA_RATE_DEF;
	dev->stack_param.RX2_Frequency                    = MIPB_RX2_FREQUENCY_DEF;
	dev->stack_param.Public_Network_Enable            = Public_Network_Disabled;
	for(i=0; i<8; i++)
	{
		dev->stack_param.Customer_DevEUI[i] = DevEUI_mipb[i];
	}
	for(i=0; i<8; i++)
	{
		dev->stack_param.AppEUI[i] = JoinEUI_mipb[i];
	}
	for(i=0; i<16; i++)
	{
		dev->stack_param.AppKey[i] = AppKey_mipb[i];
	}
}
