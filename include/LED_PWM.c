#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_err.h"

#define LED_GPIO 21

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY_HZ       5000

#define LEDC_MAX_DUTY           ((1 << 10) - 1)

static void led_pwm_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num   = LED_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER,
        .duty       = LEDC_MAX_DUTY,   // active-low LED: max duty = mostly off
        .hpoint     = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static void led_set_from_int16(int16_t value)
{
    int32_t shifted = (int32_t)value - (int32_t)INT16_MIN;
    // shifted: 0..65535

    uint32_t duty_active_high = ((uint32_t)shifted * LEDC_MAX_DUTY) / 65535u;

    // XIAO ESP32S3 onboard LED is active-low, so invert duty.
    uint32_t duty = LEDC_MAX_DUTY - duty_active_high;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}