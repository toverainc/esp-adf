#include <string.h>
#include "i2c_bus.h"
#include "board.h"
#include "esp_log.h"
#include "aw88298.h"
#include "audio_volume.h"

#define AW88298_ADDR 0x36
#define AW88298_REG_SYSCTL 0x04

#define AW9523_ADDR 0x58

#define AXP2101_ADDR 0x34

#define AW_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

static char *TAG = "DRV88298";

static codec_dac_volume_config_t *dac_vol_handle;
static i2c_bus_handle_t i2c_handle;

static esp_err_t aw88298_write_reg(uint8_t reg_addr, uint16_t data)
{
    ESP_LOGI(TAG, "aw88298: writing %04x" PRIu16 " to I2C register %02x" PRIu8, data, reg_addr);
    return i2c_bus_write_16(i2c_handle, AW88298_ADDR, &reg_addr, sizeof(reg_addr), &data);
}

static esp_err_t aw88298_read_reg(uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret = ESP_OK;
    ret = i2c_bus_read_bytes(i2c_handle, AW88298_ADDR, &reg_addr, sizeof(reg_addr), data, sizeof(data));
    ESP_LOGI(TAG, "aw88298: read %04x" PRIu16 " from I2C register %02x" PRIu8, *data, reg_addr);
    return ret;
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
    // from 0.5V to 3.5V 100mv/step
    // 0b00000  0.5V
    // 0b11110  3.5V
    axp2101_write_reg(0x99, 0b11110 - 5);  // DLDO1
}

static int i2c_init()
{
    ESP_LOGI(TAG, "i2c_init()");
    int res = 0;
    i2c_config_t aw_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
    };
    res = get_i2c_pins(I2C_NUM_1, &aw_i2c_cfg);
    AW_ASSERT(res, "getting i2c pins error", -1);
    ESP_LOGI(TAG, "sca=%d scl=%d", aw_i2c_cfg.sda_io_num, aw_i2c_cfg.scl_io_num);
    i2c_handle = i2c_bus_create(I2C_NUM_1, &aw_i2c_cfg);
    if (i2c_handle == NULL) {
        ESP_LOGE(TAG, "failed to initialize I2C bus");
    }
    return res;
}

// AW9523B is a 16 port GPIO expander
// from CoreS3-Box repo
static esp_err_t aw9523_init(void) {
    esp_err_t ret = ESP_OK;

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

esp_err_t aw88298_codec_deinit()
{
    ESP_LOGI(TAG, "aw88298_codec_deinit()");
    i2c_bus_delete(i2c_handle);
    audio_codec_volume_deinit(dac_vol_handle);
    return ESP_OK;
}

esp_err_t aw88298_codec_init(audio_hal_codec_config_t *codec_cfg)
{
    ESP_LOGI(TAG, "aw88298_codec_init()");
    //uint8_t datmp, regv;
    //int coeff;
    esp_err_t ret = ESP_OK;

    i2c_init();
    aw9523_init();
    vTaskDelay(100);

    axp2101_turn_on_bl();

    // from CoreS3-Box repo
    aw88298_write_reg(0x61, 0x0673);
    aw88298_write_reg(0x04, 0x4040);
    aw88298_write_reg(0x05, 0x0008);
    // aw88298_write_reg(0x06, 0x14C8);
    aw88298_write_reg(0x06, 0x0a61);
    aw88298_write_reg(0x0C, 0x0064);

    return ret;
}

esp_err_t aw88298_codec_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    ESP_LOGI(TAG, "aw88298_codec_config_i2s()");
    int ret = ESP_OK;
    //ret |= es8311_set_bits_per_sample(iface->bits);
    //ret |= es8311_config_fmt(iface->fmt);
    return ret;
}

esp_err_t aw88298_codec_get_voice_volume(int *volume)
{
    ESP_LOGI(TAG, "aw88298_codec_get_voice_volume()");
    esp_err_t res = ESP_OK;
    /*
    int regv = 0;
    regv = es8311_read_reg(ES8311_DAC_REG32);
    if (regv == ESP_FAIL) {
        *volume = 0;
        res = ESP_FAIL;
    } else {
        if (regv == dac_vol_handle->reg_value) {
            *volume = dac_vol_handle->user_volume;
        } else {
            *volume = 0;
            res = ESP_FAIL;
        }
    }
    ESP_LOGD(TAG, "Get volume:%.2d reg_value:0x%.2x", *volume, regv);
    */
    return res;
}

/**
 * @brief Set voice volume
 *
 * @note Register values. 0x00: -95.5 dB, 0x5B: -50 dB, 0xBF: 0 dB, 0xFF: 32 dB
 * @note Accuracy of gain is 0.5 dB
 *
 * @param volume: voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t aw88298_codec_set_voice_volume(int volume)
{
    ESP_LOGI(TAG, "aw88298_codec_set_voice_volume()");
    esp_err_t res = ESP_OK;
    // uint8_t reg = 0;
    // reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);
    // res = es8311_write_reg(ES8311_DAC_REG32, reg);
    //ESP_LOGD(TAG, "Set volume:%.2d reg_value:0x%.2x dB:%.1f", dac_vol_handle->user_volume, reg,
    //        audio_codec_cal_dac_volume(dac_vol_handle));
    return res;
}

esp_err_t aw88298_codec_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    ESP_LOGI(TAG, "aw88298_codec_ctrl_state()");
    esp_err_t ret = ESP_OK;

    return ret;
}

esp_err_t aw88298_set_voice_mute(bool enable)
{
    ESP_LOGD(TAG, "Aw88298SetVoiceMute volume:%d", enable);
    // es8311_mute(enable);
    return ESP_OK;
}

/*
* enable pa power
*/
esp_err_t aw88298_pa_power(bool enable)
{
    ESP_LOGI(TAG, "aw88298_pa_power()");
    esp_err_t ret = ESP_OK;
    uint16_t regval;

    aw88298_read_reg(AW88298_REG_SYSCTL, &regval);

    if (enable) {
        ret = aw88298_write_reg(AW88298_REG_SYSCTL, regval | (0 << 1));
    } else {
        ret = aw88298_write_reg(AW88298_REG_SYSCTL, regval | (1 << 1));
    }
    return ret;
}

/*
 * operate function of codec
 */
audio_hal_func_t AUDIO_CODEC_AW88298_DEFAULT_HANDLE = {
    .audio_codec_initialize = aw88298_codec_init,
    .audio_codec_deinitialize = aw88298_codec_deinit,
    .audio_codec_ctrl = aw88298_codec_ctrl_state,
    .audio_codec_config_iface = aw88298_codec_config_i2s,
    .audio_codec_set_mute = aw88298_set_voice_mute,
    .audio_codec_set_volume = aw88298_codec_set_voice_volume,
    .audio_codec_get_volume = aw88298_codec_get_voice_volume,
    .audio_codec_enable_pa = aw88298_pa_power,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

