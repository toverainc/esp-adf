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
#include "board.h"
#include "audio_mem.h"
#include "esp_lcd_panel_ops.h"
#include "i2c_bus.h"
#include "periph_sdcard.h"
#include "periph_lcd.h"
#include "periph_adc_button.h"
#include "tca9554.h"

#define AW9523_ADDR 0x58

#define AXP2101_ADDR 0x34

#define AW_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static const char *TAG = "AUDIO_BOARD_CORES3";

static audio_board_handle_t board_handle = 0;
static i2c_bus_handle_t i2c_handle;

audio_board_handle_t audio_board_init(void)
{
    ESP_LOGI(TAG, "audio_board_init()");
    if (board_handle) {
        ESP_LOGW(TAG, "The board has already been initialized!");
        return board_handle;
    }
    board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);
    board_handle->audio_hal = audio_board_codec_init();
    board_handle->adc_hal = audio_board_adc_init();
    return board_handle;
}

audio_hal_handle_t audio_board_adc_init(void)
{
    ESP_LOGI(TAG, "audio_board_adc_init()");
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t adc_hal = NULL;
    adc_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES7210_DEFAULT_HANDLE);
    // AUDIO_NULL_CHECK(TAG, adc_hal, return NULL);
    if (!adc_hal) {
        ESP_LOGE(TAG, "audio_board_adc_init(): !adc_hal");
    }
    return adc_hal;
}

audio_hal_handle_t audio_board_codec_init(void)
{
    ESP_LOGI(TAG, "audio_board_codec_init()");
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t codec_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_AW88298_DEFAULT_HANDLE);
    // AUDIO_NULL_CHECK(TAG, codec_hal, return NULL);
    if (codec_hal == NULL) {
        ESP_LOGE(TAG, "audio_board_codec_init(): !codec_hal");
    }
    return codec_hal;
}

static int i2c_init()
{
    ESP_LOGI(TAG, "i2c_init()");
    int res = 0;
    i2c_config_t aw_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    res = get_i2c_pins(I2C_NUM_0, &aw_i2c_cfg);
    AW_ASSERT(res, "getting i2c pins error", -1);
    ESP_LOGI(TAG, "sca=%d scl=%d", aw_i2c_cfg.sda_io_num, aw_i2c_cfg.scl_io_num);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &aw_i2c_cfg);
    if (i2c_handle == NULL) {
        ESP_LOGE(TAG, "failed to initialize I2C bus");
    }
    return res;
}

static esp_err_t aw9523_write_reg(uint8_t reg_addr, uint8_t data)
{
    ESP_LOGI(TAG, "aw9523: writing %04x" PRIu8 " to I2C register %02x" PRIu8, data, reg_addr);
    return i2c_bus_write_bytes(i2c_handle, AW9523_ADDR, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

static esp_err_t axp2101_read_reg(uint8_t reg_addr, uint8_t *data)
{
    esp_err_t ret = ESP_OK;
    ret = i2c_bus_read_bytes(i2c_handle, AXP2101_ADDR, &reg_addr, sizeof(reg_addr), data, sizeof(data));
    ESP_LOGI(TAG, "axp2101: read %02x" PRIu8 " from I2C register %02x" PRIu8, *data, reg_addr);
    return ret;
}

static esp_err_t axp2101_write_reg(uint8_t reg_addr, uint8_t data)
{
    ESP_LOGI(TAG, "axp2101: writing %02x" PRIu8 " to I2C register %02x" PRIu8, data, reg_addr);
    return i2c_bus_write_bytes(i2c_handle, AXP2101_ADDR, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

static void axp2101_turn_on_bl(void)
{
    uint8_t cfg = 0;
    axp2101_read_reg(0x90, &cfg);
    axp2101_write_reg(0x90, cfg | 0x80);
    // from 0.5V to 3.4 in 29 100mv steps
    // 0b00000  0.5V
    // 0b00001  0.6V
    // 0b00010  0.7V
    // 0b00011  0.8V
    // 0b00100  0.9V
    // 0b00101  1.0V
    // 0b00110  1.1V
    // 0b00111  1.2V
    // 0b01000  1.3V
    // 0b01001  1.4V
    // 0b01010  1.5V
    // 0b01011  1.6V
    // 0b01100  1.7V
    // 0b01101  1.8V
    // 0b01110  1.9V
    // 0b01111  2.0V
    // 0b10000  2.1V
    // 0b10001  2.2V
    // 0b10010  2.3V
    // 0b10011  2.4V
    // 0b10100  2.5V
    // 0b10101  2.6V
    // 0b10110  2.7V
    // 0b10111  2.8V
    // 0b11000  2.9V
    // 0b11001  3.0V
    // 0b11010  3.1V
    // 0b11011  3.2V
    // 0b11100  3.3V
    axp2101_write_reg(0x99, 0b11001);  // DLDO1
}

// AW9523B is a 16 port GPIO expander
// from CoreS3-Box repo
static esp_err_t aw9523_init(void) {
    esp_err_t ret = ESP_OK;

    i2c_init();

    // software reset
    ret |= aw9523_write_reg(0x7F, 0x00);    // Int_Port1
    vTaskDelay(30);

    // aw9523 default settings

    // set pin directions (0 = output, 1 = input)
    ret |= aw9523_write_reg(0x04, 0xd8);    // Config_Port0     0b11011000
    ret |= aw9523_write_reg(0x05, 0x7c);    // Config_Port1     0b01111100

    // set pin mode (0 = LED, 1 = GPIO)
    ret |= aw9523_write_reg(0x12, 0xff);    // LED Mode Switch  0b11111111
    ret |= aw9523_write_reg(0x13, 0xff);    // LED Mode Switch  0b11111111

    // set P0 port GPIO output mode (bit 4 = 1 --> Push-Pull mode)
    ret |= aw9523_write_reg(0x11, 0x10);    // CTL              0b00010001

    // set GPIO output value (0 = low, 1 = high)
    // this resets FT6336U and ES7210
    ret |= aw9523_write_reg(0x02, 0x05);    // Output_Port0     0b00000101
    // this resets GC0308 and ?
    ret |= aw9523_write_reg(0x03, 0x03);    // Output_Port1     0b00000011

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize AW9523 GPIO expander: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t _lcd_rest(esp_periph_handle_t self, void *ctx)
{
    // Reset the LCD
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t _get_lcd_io_bus (void *bus, esp_lcd_panel_io_spi_config_t *io_config,
                           esp_lcd_panel_io_handle_t *out_panel_io)
{
    return esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)bus, io_config, out_panel_io);
}

void *audio_board_lcd_init(esp_periph_set_handle_t set, void *cb)
{
    aw9523_init();
    axp2101_turn_on_bl();

    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        /*!< Prevent left shift negtive value warning */
        .pin_bit_mask = LCD_CTRL_GPIO > 0 ? 1ULL << LCD_CTRL_GPIO : 0ULL,
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(LCD_CTRL_GPIO, true);



    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_CLK_GPIO,
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_V_RES * LCD_H_RES * 2
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC_GPIO,
        .cs_gpio_num = LCD_CS_GPIO,
        .pclk_hz = 10 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = cb,
        .user_ctx = NULL,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST_GPIO,
        .color_space = LCD_COLOR_SPACE,
        .bits_per_pixel = 16,
    };
    periph_lcd_cfg_t cfg = {
        .io_bus = (void *)SPI2_HOST,
        .new_panel_io = _get_lcd_io_bus,
        .lcd_io_cfg = &io_config,
        .new_lcd_panel = esp_lcd_new_panel_st7789,
        .lcd_dev_cfg = &panel_config,
        .rest_cb = NULL,
        .rest_cb_ctx = NULL,
        .lcd_swap_xy = LCD_SWAP_XY,
        .lcd_mirror_x = LCD_MIRROR_X,
        .lcd_mirror_y = LCD_MIRROR_Y,
        .lcd_color_invert = LCD_COLOR_INV,
    };
    esp_periph_handle_t periph_lcd = periph_lcd_init(&cfg);
    AUDIO_NULL_CHECK(TAG, periph_lcd, return NULL);
    esp_periph_start(set, periph_lcd);

    return (void *)periph_lcd_get_panel_handle(periph_lcd);
}

esp_err_t audio_board_key_init(esp_periph_set_handle_t set)
{
    periph_adc_button_cfg_t adc_btn_cfg = PERIPH_ADC_BUTTON_DEFAULT_CONFIG();
    adc_arr_t adc_btn_tag = ADC_DEFAULT_ARR();
    adc_btn_tag.total_steps = 3;
    adc_btn_tag.adc_ch = ADC1_CHANNEL_0;
    int btn_array[4] = {190, 1000, 2195, 3000};
    adc_btn_tag.adc_level_step = btn_array;
    adc_btn_cfg.arr = &adc_btn_tag;
    adc_btn_cfg.arr_size = 1;
    esp_periph_handle_t adc_btn_handle = periph_adc_button_init(&adc_btn_cfg);
    AUDIO_NULL_CHECK(TAG, adc_btn_handle, return ESP_ERR_ADF_MEMORY_LACK);
    return esp_periph_start(set, adc_btn_handle);
}

esp_err_t audio_board_sdcard_init(esp_periph_set_handle_t set, periph_sdcard_mode_t mode)
{
    if (mode != SD_MODE_1_LINE) {
        ESP_LOGE(TAG, "Current board only support 1-line SD mode!");
        return ESP_FAIL;
    }
    periph_sdcard_cfg_t sdcard_cfg = {
        .root = "/sdcard",
        .card_detect_pin = get_sdcard_intr_gpio(),
        .mode = mode
    };
    esp_periph_handle_t sdcard_handle = periph_sdcard_init(&sdcard_cfg);
    esp_err_t ret = esp_periph_start(set, sdcard_handle);
    int retry_time = 5;
    bool mount_flag = false;
    while (retry_time --) {
        if (periph_sdcard_is_mounted(sdcard_handle)) {
            mount_flag = true;
            break;
        } else {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    if (mount_flag == false) {
        ESP_LOGE(TAG, "Sdcard mount failed");
        return ESP_FAIL;
    }
    return ret;
}

audio_board_handle_t audio_board_get_handle(void)
{
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board)
{
    esp_err_t ret = ESP_OK;
    ret |= audio_hal_deinit(audio_board->audio_hal);
    ret |= audio_hal_deinit(audio_board->adc_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}
