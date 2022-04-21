#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>

/*Pin mapping*/
#define HX711DoutPin 26
#define HX711SckPin 27
#define vibrationSensorPin 23
#define btnTarePin 13
#define btnCalibratePin 14

#define SamplesHX711 5

static const char *TAG = "HX711 DATA";

void test(void *pvParameters)
{
    hx711_t dev = {
        .dout = HX711DoutPin,
        .pd_sck = HX711SckPin,
        .gain = HX711_GAIN_A_64};

    // initialize device
    ESP_ERROR_CHECK(hx711_init(&dev));

    esp_err_t r = hx711_wait(&dev, 500);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
    }

    int32_t data;
    r = hx711_read_average(&dev, SamplesHX711, &data);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
    }

    int32_t tare;

    tare = data;

    ESP_LOGI(TAG, "Tare init: %d\n", tare);

    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        r = hx711_read_average(&dev, SamplesHX711, &data);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        ESP_LOGI(TAG, "Raw data: %d\n", data);
        ESP_LOGI(TAG, "Tare data: %d\n", data - tare);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}
