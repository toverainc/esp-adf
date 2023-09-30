/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2022 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include "board.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "soc/soc_caps.h"

static const char *TAG = "M5STACK_CORES3";

esp_err_t get_i2c_pins(i2c_port_t port, i2c_config_t *i2c_config)
{
    AUDIO_NULL_CHECK(TAG, i2c_config, return ESP_FAIL);
    if (port == I2C_NUM_0 || port == I2C_NUM_1) {
        i2c_config->sda_io_num = GPIO_NUM_12;
        i2c_config->scl_io_num = GPIO_NUM_11;
    } else {
        i2c_config->sda_io_num = GPIO_NUM_NC;
        i2c_config->scl_io_num = GPIO_NUM_NC;
        ESP_LOGE(TAG, "i2c port %d is not supported", port);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "get_i2c_pins(): mode='%d' sda_io_num='%d' scl_io_num='%d' sda_pullup_en='%d' scl_pullup_en='%d' clk_flags='%" PRIu32"'", i2c_config->mode, i2c_config->sda_io_num, i2c_config->scl_io_num, i2c_config->sda_pullup_en, i2c_config->scl_pullup_en, i2c_config->clk_flags);
    return ESP_OK;
}

esp_err_t get_i2s_pins(i2s_port_t port, board_i2s_pin_t *i2s_config)
{
    AUDIO_NULL_CHECK(TAG, i2s_config, return ESP_FAIL);
    if (port == I2S_NUM_0) {
        i2s_config->bck_io_num = GPIO_NUM_34;
        i2s_config->ws_io_num = GPIO_NUM_33;
        i2s_config->data_out_num = GPIO_NUM_13;
        i2s_config->data_in_num = GPIO_NUM_14;
        i2s_config->mck_io_num = GPIO_NUM_0;
    } else if (port == I2S_NUM_1) {
        i2s_config->bck_io_num = GPIO_NUM_NC;
        i2s_config->ws_io_num = GPIO_NUM_NC;
        i2s_config->data_out_num = GPIO_NUM_NC;
        i2s_config->data_in_num = GPIO_NUM_NC;
        i2s_config->mck_io_num = GPIO_NUM_NC;
    } else {
        memset(i2s_config, -1, sizeof(board_i2s_pin_t));
        ESP_LOGE(TAG, "i2s port %d is not supported", port);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t get_spi_pins(spi_bus_config_t *spi_config, spi_device_interface_config_t *spi_device_interface_config)
{
    AUDIO_NULL_CHECK(TAG, spi_config, return ESP_FAIL);
    AUDIO_NULL_CHECK(TAG, spi_device_interface_config, return ESP_FAIL);

    spi_config->mosi_io_num = GPIO_NUM_37;
    spi_config->miso_io_num = GPIO_NUM_35;
    spi_config->sclk_io_num = GPIO_NUM_36;
    spi_config->quadwp_io_num = GPIO_NUM_NC;
    spi_config->quadhd_io_num = GPIO_NUM_NC;

    spi_device_interface_config->spics_io_num = GPIO_NUM_4;

    ESP_LOGW(TAG, "SPI interface is not supported");
    return ESP_OK;
}

// sdcard

int8_t get_sdcard_intr_gpio(void)
{
    return SDCARD_INTR_GPIO;
}

int8_t get_sdcard_open_file_num_max(void)
{
    return SDCARD_OPEN_FILE_NUM_MAX;
}

int8_t get_sdcard_power_ctrl_gpio(void)
{
    return SDCARD_PWR_CTRL;
}

// input-output pins

int8_t get_headphone_detect_gpio(void)
{
    return HEADPHONE_DETECT;
}

int8_t get_pa_enable_gpio(void)
{
    return PA_ENABLE_GPIO;
}

int8_t get_es7243_mclk_gpio(void)
{
    return GPIO_NUM_0;
}

// adc button id

int8_t get_input_rec_id(void)
{
    return BUTTON_REC_ID;
}

int8_t get_input_mode_id(void)
{
    return BUTTON_MODE_ID;
}

int8_t get_input_set_id(void)
{
    return BUTTON_SET_ID;
}

int8_t get_input_play_id(void)
{
    return BUTTON_PLAY_ID;
}

int8_t get_input_volup_id(void)
{
    return BUTTON_VOLUP_ID;
}

int8_t get_input_voldown_id(void)
{
    return BUTTON_VOLDOWN_ID;
}

// led pins

int8_t get_green_led_gpio(void)
{
    return GPIO_NUM_NC;
}

int8_t get_blue_led_gpio(void)
{
    return GPIO_NUM_NC;
}

