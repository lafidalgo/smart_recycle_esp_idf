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
#include <string.h>

#include "smbus.h"
#include "i2c-lcd1602.h"

/*HX711*/
#define HX711DoutPin 26
#define HX711SckPin 27

// Interrupts
#define vibrationSensorPin 23
#define btnTarePin 13
#define btnCalibratePin 14
#define GPIO_INPUT_PIN_SEL ((1ULL << vibrationSensorPin) | (1ULL << btnTarePin) | (1ULL << btnCalibratePin))
#define ESP_INTR_FLAG_DEFAULT 0

#define SamplesHX711 5

// LCD1602
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLUMNS 32
#define LCD_NUM_VISIBLE_COLUMNS 16

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0 // disabled
#define I2C_MASTER_RX_BUF_LEN 0 // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22

typedef struct
{
    char line;
    char message[16];
} LCDMessage;

/*Variáveis para armazenamento do handle das tasks, queues, semaphores e timers*/
QueueHandle_t xMessageLCD;
QueueHandle_t xVibration;
QueueHandle_t xTare;
QueueHandle_t xCalibrate;

static const char *TAG = "App";

//******************** INTERRUPTS ********************

static void IRAM_ATTR vibration_sensor_isr_handler(void *arg)
{
    uint32_t gpio_num = 1;
    xQueueSendFromISR(xVibration, &gpio_num, NULL);
}

static void IRAM_ATTR tare_isr_handler(void *arg)
{
    uint32_t gpio_num = 1;
    xQueueSendFromISR(xTare, &gpio_num, NULL);
}

static void IRAM_ATTR calibrate_isr_handler(void *arg)
{
    uint32_t gpio_num = 1;
    xQueueSendFromISR(xCalibrate, &gpio_num, NULL);
}

//******************** FUNCTIONS ********************

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

void init_interrupts(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(vibrationSensorPin, vibration_sensor_isr_handler, (void *)vibrationSensorPin);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(btnTarePin, tare_isr_handler, (void *)btnTarePin);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(btnCalibratePin, calibrate_isr_handler, (void *)btnCalibratePin);
}

//******************** TASKS ********************

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

    LCDMessage mensagem;

    char weightString[16];
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

        mensagem.line = 0;
        itoa(weight, weightString, 10);
        strcpy(mensagem.message, "");
        strcat(mensagem.message, "Peso: ");
        strcat(mensagem.message, weightString);
        strcat(mensagem.message, "kg");

        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lcd1602_task(void *pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = 0x27;

    // Set up the SMBus
    smbus_info_t *smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    LCDMessage mensagem;
    while (1)
    {
        xQueueReceive(xMessageLCD, &mensagem, portMAX_DELAY);

        ESP_LOGI(TAG, "Mensagem LCD: %s", mensagem.message);
        // turn on backlight
        // i2c_lcd1602_clear(lcd_info);
        i2c_lcd1602_set_backlight(lcd_info, true);

        switch (mensagem.line)
        {
        case 0:
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            for (int n = 0; n < LCD_NUM_VISIBLE_COLUMNS; n++) // 20 indicates symbols in line. For 2x16 LCD write - 16
            {
                i2c_lcd1602_write_char(lcd_info, 0x20);
            }
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            break;
        case 1:
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            for (int n = 0; n < LCD_NUM_VISIBLE_COLUMNS; n++) // 20 indicates symbols in line. For 2x16 LCD write - 16
            {
                i2c_lcd1602_write_char(lcd_info, 0x20);
            }
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            break;
        default:
            ESP_LOGW(TAG, "Linha LCD fora do limite");
        }

        i2c_lcd1602_write_string(lcd_info, mensagem.message);

        // i2c_lcd1602_set_backlight(lcd_info, false);
    }
}

void vibration_task(void *pvParameter)
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(xVibration, &io_num, portMAX_DELAY))
        {
            printf("Vibration: %d\n", io_num);
        }
    }
}

void tare_task(void *pvParameter)
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(xTare, &io_num, portMAX_DELAY))
        {
            printf("Tare: %d\n", io_num);
        }
    }
}

void calibrate_task(void *pvParameter)
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(xCalibrate, &io_num, portMAX_DELAY))
        {
            printf("Calibrate: %d\n", io_num);
        }
    }
}

void app_main()
{
    init_interrupts();

    /*Criação Queues*/
    xMessageLCD = xQueueCreate(10, sizeof(LCDMessage));
    xVibration = xQueueCreate(1, sizeof(uint32_t));
    xTare = xQueueCreate(1, sizeof(uint32_t));
    xCalibrate = xQueueCreate(1, sizeof(uint32_t));

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(lcd1602_task, "lcd1602_task", 5120, NULL, 5, NULL);
    xTaskCreate(vibration_task, "vibration_task", 2048, NULL, 10, NULL);
    xTaskCreate(tare_task, "tare_task", 2048, NULL, 10, NULL);
    xTaskCreate(calibrate_task, "calibrate_task", 2048, NULL, 10, NULL);
}
