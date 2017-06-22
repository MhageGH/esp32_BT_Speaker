// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// modified by Mhage in 2017

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#define PIN_NUM_MISO 17		// SO
#define PIN_NUM_MOSI 23		// SI
#define PIN_NUM_CLK  19		// SCLK
#define PIN_NUM_XCS  22		// CS
#define PIN_NUM_XDCS 21		// BSYNC
#define PIN_NUM_DREQ 18		// DREQ
spi_device_handle_t spi2;

/* event for handler "bt_av_hdl_stack_up */
enum {
	BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

void Clockup(spi_device_handle_t spi) {
	while (!gpio_get_level(PIN_NUM_DREQ));
	unsigned char clkup[4] = { 2, 3, 168, 0 };
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(trans));
	trans.user = (void*)0;
	trans.length = 4 * 8;
	trans.tx_buffer = clkup;
	spi_device_transmit(spi, &trans);
	vTaskDelay(100 / portTICK_RATE_MS);
}

void SetVol(spi_device_handle_t spi, unsigned char att) {
	while (!gpio_get_level(PIN_NUM_DREQ));
	unsigned char setVol[4] = { 2, 11, 0, 0 };
	setVol[2] = setVol[3] = att;
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(trans));
	trans.user = (void*)0;
	trans.length = 4 * 8;
	trans.tx_buffer = setVol;
	spi_device_transmit(spi, &trans);
}

void SendHeader(spi_device_handle_t spi) {
	while (!gpio_get_level(PIN_NUM_DREQ));
	const unsigned char header[44] = {
		0x52,0x49,0x46,0x46,0xff,0xff,0xff,0xff,0x57,0x41,0x56,0x45,0x66,0x6d,0x74,0x20,
		0x10,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x44,0xac,0x00,0x00,0x10,0xb1,0x02,0x00,
		0x04,0x00,0x10,0x00,0x64,0x61,0x74,0x61,0xff,0xff,0xff,0xff
	};
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(trans));
	trans.user = (void*)0;
	trans.length = 44 * 8;
	trans.tx_buffer = header;
	spi_device_transmit(spi, &trans);
}

void app_main()
{
	spi_device_handle_t spi1;
	esp_err_t ret;
	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret == ESP_OK);
	spi_device_interface_config_t devcfg1 = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 4000000,                //Clock
		.mode = 0,                                //SPI mode 0
		.spics_io_num = PIN_NUM_XCS,			  //CS pin
		.queue_size = 1,                          //We want to be able to queue 1 transactions at a time
	};
	ret = spi_bus_add_device(HSPI_HOST, &devcfg1, &spi1);
	assert(ret == ESP_OK);
	spi_device_interface_config_t devcfg2 = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 4000000,                //Clock
		.mode = 0,                                //SPI mode 0
		.spics_io_num = PIN_NUM_XDCS,			  //CS pin
		.queue_size = 1,                          //We want to be able to queue 1 transactions at a time
	};
	ret = spi_bus_add_device(HSPI_HOST, &devcfg2, &spi2);
	assert(ret == ESP_OK);

	gpio_set_direction(PIN_NUM_DREQ, GPIO_MODE_INPUT);

	Clockup(spi1);
	SetVol(spi1, 60);
	SendHeader(spi2);


	nvs_flash_init();

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
		ESP_LOGE(BT_AV_TAG, "%s initialize controller failed\n", __func__);
		return;
	}

	if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
		ESP_LOGE(BT_AV_TAG, "%s enable controller failed\n", __func__);
		return;
	}

	if (esp_bluedroid_init() != ESP_OK) {
		ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed\n", __func__);
		return;
	}

	if (esp_bluedroid_enable() != ESP_OK) {
		ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed\n", __func__);
		return;
	}

	/* create application task */
	bt_app_task_start_up();

	/* Bluetooth device name, connection mode and profile set up */
	bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}


static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
	ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
	switch (event) {
	case BT_APP_EVT_STACK_UP: {
		/* set up device name */
		char *dev_name = "ESP_SPEAKER";
		esp_bt_dev_set_device_name(dev_name);

		/* initialize A2DP sink */
		esp_a2d_register_callback(&bt_app_a2d_cb);
		esp_a2d_register_data_callback(bt_app_a2d_data_cb);
		esp_a2d_sink_init();

		/* initialize AVRCP controller */
		esp_avrc_ct_init();
		esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

		/* set discoverable and connectable mode, wait to be connected */
		esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
		break;
	}
	default:
		ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
		break;
	}
}
