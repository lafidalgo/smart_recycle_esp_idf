#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <hx711.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "smbus.h"
#include "i2c-lcd1602.h"

/*Pin mapping*/
#define HX711DoutPin 26
#define HX711SckPin 27
#define vibrationSensorPin 23
#define btnTarePin 13
#define btnCalibratePin 14

#define SamplesHX711 5

// LCD1602
#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22

/*Variáveis para armazenamento do handle das tasks, queues, semaphores e timers*/
QueueHandle_t xMessageLCD;

static const char *TAG = "App";

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

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

    ESP_LOGI(TAG, "Tare init: %d", tare);

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

        ESP_LOGI(TAG, "Raw data: %d", data);
        ESP_LOGI(TAG, "Tare data: %d", data - tare);

        int32_t weight = data - tare;

        xQueueOverwrite(xMessageLCD, &weight);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void lcd1602_task(void * pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = 0x27;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    int32_t count = 0;
    char buffer[15];
    while(1){

        xQueueReceive(xMessageLCD, &count, portMAX_DELAY);

        itoa(count, buffer, 10);
        ESP_LOGI(TAG, "Count: %d", count);
        ESP_LOGI(TAG, "Buffer: %s", buffer);

        // turn on backlight
        ESP_LOGI(TAG, "backlight on");
        i2c_lcd1602_clear(lcd_info);
        i2c_lcd1602_set_backlight(lcd_info, true);
        i2c_lcd1602_move_cursor(lcd_info, 0, 0);
        i2c_lcd1602_write_string(lcd_info, buffer);

        // turn off backlight
        //ESP_LOGI(TAG, "backlight off");
        //i2c_lcd1602_set_backlight(lcd_info, false);
    }
}

void app_main()
{
    /*Criação Queues*/
    xMessageLCD = xQueueCreate(1, sizeof(int32_t));

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
}
