#include <ADS1115.c>

static void ads_task(void *arg)
{
    ads1115_data_rate_t rate = ADS1115_DR_128_SPS;

    ESP_ERROR_CHECK(ads1115_enable_conversion_ready_pin());

    while (true) {
        // Clear any stale notification before starting the next conversion.
        ulTaskNotifyTake(pdTRUE, 0);

        esp_err_t err = ads1115_start_single_a0(rate);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "start conversion failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int timeout_ms = ads1115_rate_to_timeout_ms(rate);

        uint32_t notified = ulTaskNotifyTake(
            pdTRUE,
            pdMS_TO_TICKS(timeout_ms)
        );

        if (notified == 0) {
            ESP_LOGW(TAG, "ADS1115 conversion timeout");
            continue;
        }

        int16_t raw = 0;
        err = ads1115_read_conversion(&raw);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read conversion failed: %s", esp_err_to_name(err));
            continue;
        }

        // Integer scaling for +/-6.144 V range:
        // 1 count = 0.1875 mV.
        int32_t millivolts = ((int32_t)raw * 1875) / 10000;

        ESP_LOGI(TAG, "raw=%d voltage=%ld mV", raw, (long)millivolts);

        // Optional pacing. If you want max rate, remove this delay.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ADS1115 interrupt-driven example");

    i2c_init();
    gpio_alert_init();

    xTaskCreatePinnedToCore(
        ads_task,
        "ads_task",
        4096,
        NULL,
        5,
        &ads_task_handle,
        1
    );
}