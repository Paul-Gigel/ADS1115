#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define TAG "ADS1115"

#define I2C_SDA_GPIO 5
#define I2C_SCL_GPIO 6
#define I2C_FREQ_HZ  100000

#define ADS_ALERT_GPIO 43   // TXS A3 -> XIAO GPIO7, choose your actual GPIO

#define ADS1115_ADDR       0x48

#define ADS1115_REG_CONV      0x00
#define ADS1115_REG_CONFIG    0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t ads_dev = NULL;

static TaskHandle_t ads_task_handle = NULL;

typedef enum {
    ADS1115_DR_8_SPS   = 0,
    ADS1115_DR_16_SPS  = 1,
    ADS1115_DR_32_SPS  = 2,
    ADS1115_DR_64_SPS  = 3,
    ADS1115_DR_128_SPS = 4,
    ADS1115_DR_250_SPS = 5,
    ADS1115_DR_475_SPS = 6,
    ADS1115_DR_860_SPS = 7,
} ads1115_data_rate_t;

// Config register bits
#define ADS_CFG_OS_START_SINGLE        (1u << 15)

// MUX[2:0]
#define ADS_CFG_MUX_AIN0_GND           (4u << 12)
#define ADS_CFG_MUX_AIN1_GND           (5u << 12)
#define ADS_CFG_MUX_AIN2_GND           (6u << 12)
#define ADS_CFG_MUX_AIN3_GND           (7u << 12)

// PGA[2:0]
#define ADS_CFG_PGA_6V144              (0u << 9)
#define ADS_CFG_PGA_4V096              (1u << 9)
#define ADS_CFG_PGA_2V048              (2u << 9)
#define ADS_CFG_PGA_1V024              (3u << 9)
#define ADS_CFG_PGA_0V512              (4u << 9)
#define ADS_CFG_PGA_0V256              (5u << 9)

// MODE
#define ADS_CFG_MODE_CONTINUOUS        (0u << 8)
#define ADS_CFG_MODE_SINGLE_SHOT       (1u << 8)

// DR[2:0]
#define ADS_CFG_DR(rate)               (((uint16_t)(rate) & 0x7u) << 5)

// Comparator bits
#define ADS_CFG_COMP_MODE_TRADITIONAL  (0u << 4)
#define ADS_CFG_COMP_POL_ACTIVE_LOW    (0u << 3)
#define ADS_CFG_COMP_LAT_NON_LATCHING  (0u << 2)

// COMP_QUE[1:0]
// 00 = assert after one conversion
// 11 = disable comparator / ALERT pin
#define ADS_CFG_COMP_QUE_AFTER_1       (0u)
#define ADS_CFG_COMP_QUE_DISABLE       (3u)

static int ads1115_rate_to_timeout_ms(ads1115_data_rate_t rate)
{
    switch (rate) {
        case ADS1115_DR_8_SPS:   return 150;
        case ADS1115_DR_16_SPS:  return 100;
        case ADS1115_DR_32_SPS:  return 60;
        case ADS1115_DR_64_SPS:  return 40;
        case ADS1115_DR_128_SPS: return 30;
        case ADS1115_DR_250_SPS: return 20;
        case ADS1115_DR_475_SPS: return 20;
        case ADS1115_DR_860_SPS: return 20;
        default:                 return 100;
    }
}

static esp_err_t ads1115_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };

    return i2c_master_transmit(ads_dev, data, sizeof(data), 1000);
}

static esp_err_t ads1115_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t out = reg;
    uint8_t in[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(
        ads_dev,
        &out,
        1,
        in,
        2,
        1000
    );

    if (err != ESP_OK) {
        return err;
    }

    *value = ((uint16_t)in[0] << 8) | in[1];
    return ESP_OK;
}

static esp_err_t ads1115_enable_conversion_ready_pin(void)
{
    // ADS1115 conversion-ready mode:
    // Set Hi_thresh MSB = 1 and Lo_thresh MSB = 0.
    //
    // Common values:
    //   Lo_thresh = 0x0000
    //   Hi_thresh = 0x8000
    //
    // Also ensure COMP_QUE != 11 in config, otherwise ALERT/RDY is disabled.
    esp_err_t err;

    err = ads1115_write_register(ADS1115_REG_LO_THRESH, 0x0000);
    if (err != ESP_OK) {
        return err;
    }

    err = ads1115_write_register(ADS1115_REG_HI_THRESH, 0x8000);
    return err;
}

static esp_err_t ads1115_start_single_a0(ads1115_data_rate_t rate)
{
    uint16_t config =
        ADS_CFG_OS_START_SINGLE |
        ADS_CFG_MUX_AIN0_GND |
        ADS_CFG_PGA_6V144 |
        ADS_CFG_MODE_SINGLE_SHOT |
        ADS_CFG_DR(rate) |
        ADS_CFG_COMP_MODE_TRADITIONAL |
        ADS_CFG_COMP_POL_ACTIVE_LOW |
        ADS_CFG_COMP_LAT_NON_LATCHING |
        ADS_CFG_COMP_QUE_AFTER_1;

    return ads1115_write_register(ADS1115_REG_CONFIG, config);
}

static esp_err_t ads1115_read_conversion(int16_t *raw)
{
    uint16_t value = 0;
    esp_err_t err = ads1115_read_register(ADS1115_REG_CONV, &value);

    if (err != ESP_OK) {
        return err;
    }

    *raw = (int16_t)value;
    return ESP_OK;
}

static void IRAM_ATTR ads_alert_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (ads_task_handle != NULL) {
        vTaskNotifyGiveFromISR(ads_task_handle, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void gpio_alert_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << ADS_ALERT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(ADS_ALERT_GPIO, ads_alert_isr, NULL));
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t ads_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &ads_config, &ads_dev));
}