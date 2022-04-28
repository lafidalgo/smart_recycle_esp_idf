#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include <esp_log.h>
#include <esp_sleep.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "cJSON.h"

#include "smbus.h"
#include "i2c-lcd1602.h"
#include <hx711.h>

/*HX711*/
#define HX711DoutPin 26
#define HX711SckPin 27
#define SamplesHX711 5
#define weightReference 2000
#define measureTimeout 30000      // Timeout in ms to measure weight
#define weightUnitReference 0.768 // Em kg

hx711_t dev = {
    .dout = HX711DoutPin,
    .pd_sck = HX711SckPin,
    .gain = HX711_GAIN_A_64};
int32_t tare = 0;
float calibration = 1;

// Interrupts
#define vibrationSensorPin 23
#define btnTarePin 13
#define btnCalibratePin 14
#define GPIO_INPUT_PIN_SEL ((1ULL << vibrationSensorPin) | (1ULL << btnTarePin) | (1ULL << btnCalibratePin))
#define ESP_INTR_FLAG_DEFAULT 0

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

#define LCDOffTimeout 2000

typedef struct
{
    char lcdEnTimeoutOff;
    bool clear;
    char line;
    char message[16];
} LCDMessage;

// SPIFFS
esp_vfs_spiffs_conf_t conf = {
    .base_path = "/data",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true};

#define BUF_PATH_SIZE 70

typedef struct
{
    char filename[60];
    char json_object[16];
    float value;
} SpiffsUpdate;

#define row1 19
#define row2 18
#define row3 5
#define row4 17
#define GPIO_INPUT_PIN_SEL_KEYPAD ((1ULL << row1) | (1ULL << row2) | (1ULL << row3) | (1ULL << row4))
#define column1 16
#define column2 4
#define column3 15
#define GPIO_OUTPUT_PIN_SEL_KEYPAD ((1ULL << column1) | (1ULL << column2) | (1ULL << column3))
char rows[] = {row1, row2, row3, row4};
#define rowCount 4
char cols[] = {column1, column2, column3};
#define colCount 3
char keys[colCount][rowCount];

/*Variáveis para armazenamento do handle das tasks, queues, semaphores e timers*/
TaskHandle_t taskReadWeightHandle = NULL;
TaskHandle_t taskKeypadHandle = NULL;
TaskHandle_t taskLCDHandle = NULL;
TaskHandle_t taskTareHandle = NULL;
TaskHandle_t taskCalibrateHandle = NULL;
TaskHandle_t taskSleepHandle = NULL;
TaskHandle_t taskFileHandlerHandle = NULL;

QueueHandle_t xMessageLCD;
QueueHandle_t xVibration;
QueueHandle_t xTare;
QueueHandle_t xCalibrate;
QueueHandle_t xSpiffsUpdate;

SemaphoreHandle_t xSemaphoreTare;
SemaphoreHandle_t xSemaphoreCalibrate;
SemaphoreHandle_t xSemaphoreStopKeypad;
SemaphoreHandle_t xSemaphoreStopReadWeight;
SemaphoreHandle_t xSemaphoreSleep;

TimerHandle_t xTimerReadWeightTimeout;
TimerHandle_t xTimerLCDOff;

static const char *TAG = "App";

//******************** INTERRUPTS ********************

static void IRAM_ATTR vibration_sensor_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskResumeFromISR(taskReadWeightHandle);
    xTaskResumeFromISR(taskKeypadHandle);
    xTimerStartFromISR(xTimerReadWeightTimeout, &xHigherPriorityTaskWoken);
}

static void IRAM_ATTR tare_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphoreTare, &xHigherPriorityTaskWoken);
}

static void IRAM_ATTR calibrate_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphoreCalibrate, &xHigherPriorityTaskWoken);
}

//******************** TIMERS ********************
void callBackTimerReadWeightTimeout(TimerHandle_t xTimer)
{
    LCDMessage mensagem;

    xSemaphoreGive(xSemaphoreStopReadWeight);
    xSemaphoreGive(xSemaphoreStopKeypad);
    xTimerStop(xTimerReadWeightTimeout, 0);
    mensagem.lcdEnTimeoutOff = 1;
    mensagem.clear = 1;
    mensagem.line = 0;
    sprintf(mensagem.message, "Tempo esgotado");
    xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);
}

void callBackTimerLCDOff(TimerHandle_t xTimer)
{
    LCDMessage mensagem;

    xTimerStop(xTimerLCDOff, 0);
    mensagem.lcdEnTimeoutOff = 2;
    mensagem.clear = 1;
    mensagem.line = 0;
    sprintf(mensagem.message, "LCD OFF");
    xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);
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

void init_hx711(void)
{
    // initialize device
    ESP_ERROR_CHECK(hx711_init(&dev));

    esp_err_t r = hx711_wait(&dev, 500);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
    }
}

void init_spiffs(void)
{
    // verificação do sistema de arquivos (montagem, partição existente, formatação)
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        while (true)
            ;
    }
    ESP_LOGI(TAG, "SPIFFS iniciado com sucesso");
}

float readFile(char *filename, char *json_object)
{
    ESP_LOGI(TAG, "Lendo...");
    char path[BUF_PATH_SIZE];
    float value = 1;
    sprintf(path, "/data/%s", filename);
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Falha na leitura");
        return value;
    }
    char line[100];
    fread(line, sizeof(line), 1, f);
    fclose(f);

    cJSON *root = cJSON_Parse(line);
    if (cJSON_GetObjectItem(root, json_object))
    {
        value = cJSON_GetObjectItem(root, json_object)->valuedouble;
        ESP_LOGI(TAG, "%s: %f", json_object, value);
    }
    cJSON_Delete(root);

    // ESP_LOGI(TAG, "Read from file: '%s'", line);

    return value;
}

void writeFile(char *filename, char *json_object, float value)
{
    ESP_LOGI(TAG, "Escrevendo...");
    char path[BUF_PATH_SIZE];
    sprintf(path, "/data/%s", filename);
    ESP_LOGI(TAG, "%s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Falhou...");
        return;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, json_object, value);
    char *my_json_string = cJSON_Print(root);
    ESP_LOGI(TAG, "%s: %f", json_object, value);

    fprintf(f, my_json_string);
    fclose(f);
    ESP_LOGI(TAG, "Escrita feita com sucesso...");
    cJSON_Delete(root);
    cJSON_free(my_json_string);
}

void updateFile(char *filename, char *json_object, float value)
{
    ESP_LOGI(TAG, "Atualizando...");
    char path[BUF_PATH_SIZE];
    sprintf(path, "/data/%s", filename);
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Falha na leitura");
        return;
    }
    char line[100];
    fread(line, sizeof(line), 1, f);
    fclose(f);

    cJSON *root = cJSON_Parse(line);
    if (cJSON_GetObjectItem(root, json_object))
    {
        cJSON_SetIntValue(cJSON_GetObjectItem(root, json_object), value);
    }
    else
    {
        cJSON_AddNumberToObject(root, json_object, value);
    }

    char *my_json_string = cJSON_Print(root);

    ESP_LOGI(TAG, "%s: %f", json_object, value);

    f = fopen(path, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Falha na leitura");
        return;
    }
    fprintf(f, my_json_string);
    fclose(f);
    ESP_LOGI(TAG, "Atualização feita com sucesso...");
    cJSON_Delete(root);
    cJSON_free(my_json_string);
}

void init_keypad(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL_KEYPAD;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_KEYPAD;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void readKeypad(void)
{
    // iterate the columns
    for (int colIndex = 0; colIndex < colCount; colIndex++)
    {
        // col: set to output to low
        char curCol = cols[colIndex];
        gpio_set_level(curCol, 0);

        // row: interate through the rows
        for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
        {
            char rowCol = rows[rowIndex];
            keys[colIndex][rowIndex] = gpio_get_level(rowCol);
        }
        // disable the column
        gpio_set_level(curCol, 1);
    }
}

void printKeypad(void)
{
    for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
    {
        printf("0%d: ", rowIndex);

        for (int colIndex = 0; colIndex < colCount; colIndex++)
        {
            printf("%d", keys[colIndex][rowIndex]);
            if (colIndex < colCount)
                printf(", ");
        }
        printf("\t");
    }
    printf(" \n");
}

char getKeypadChar(void)
{
    char keymap[rowCount][colCount] = {
        {'1', '2', '3'},
        {'4', '5', '6'},
        {'7', '8', '9'},
        {'*', '0', '#'}};

    for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
    {
        for (int colIndex = 0; colIndex < colCount; colIndex++)
        {
            if (keys[colIndex][rowIndex] == 0)
            {
                //printf("Botão pressionado: %c\n", keymap[rowIndex][colIndex]);
                return (keymap[rowIndex][colIndex]);
            }
        }
    }
    return 0;
}

//******************** TASKS ********************

void readWeight_task(void *pvParameters)
{
    int32_t data;
    LCDMessage mensagem;

    // read from device
    while (1)
    {
        esp_err_t r = hx711_power_down(&dev, false);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not power up: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }
        
        r = hx711_wait(&dev, 500);
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

        float calibratedWeight = ((data - tare) / calibration) / 1000;
        // float quantityUnits = calibratedWeight / weightUnitReference;

        // ESP_LOGI(TAG, "Raw data: %d", data);
        // ESP_LOGI(TAG, "Tare data: %d", data - tare);
        ESP_LOGI(TAG, "Calibrated data: %f", calibratedWeight);
        // ESP_LOGI(TAG, "Quantity units: %f", quantityUnits);

        mensagem.lcdEnTimeoutOff = 0;
        mensagem.clear = 0;
        mensagem.line = 0;
        sprintf(mensagem.message, "Peso: %0.2f kg", calibratedWeight);
        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);

        /*mensagem.lcdEnTimeoutOff = 0;
        mensagem.clear = 0;
        mensagem.line = 1;
        sprintf(mensagem.message, "Qntd: %0.0f", quantityUnits);
        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);*/

        if (xSemaphoreTake(xSemaphoreStopReadWeight, pdMS_TO_TICKS(10)) == pdPASS)
        {
            r = hx711_power_down(&dev, true);
            if (r != ESP_OK)
            {
                ESP_LOGE(TAG, "Could not power down: %d (%s)\n", r, esp_err_to_name(r));
                continue;
            }
            ESP_LOGI(TAG, "Read Weight Task suspended");
            xSemaphoreGive(xSemaphoreSleep);
            vTaskSuspend(taskReadWeightHandle);
        }

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

    i2c_lcd1602_set_backlight(lcd_info, false);

    LCDMessage mensagem;

    mensagem.lcdEnTimeoutOff = 1;
    mensagem.clear = 1;
    mensagem.line = 0;
    sprintf(mensagem.message, "Balanca ligada!");
    xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);
    while (1)
    {
        xQueueReceive(xMessageLCD, &mensagem, portMAX_DELAY);

        ESP_LOGI(TAG, "Mensagem LCD: %s", mensagem.message);

        if (mensagem.lcdEnTimeoutOff == 2)
        {
            i2c_lcd1602_clear(lcd_info);
            i2c_lcd1602_set_backlight(lcd_info, false);
        }
        else
        {
            i2c_lcd1602_set_backlight(lcd_info, true);

            if (mensagem.clear)
            {
                i2c_lcd1602_clear(lcd_info);
            }

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

            if (mensagem.lcdEnTimeoutOff == 1)
            {
                xTimerReset(xTimerLCDOff, 0);
            }

            i2c_lcd1602_write_string(lcd_info, mensagem.message);
        }
    }
}

void tare_task(void *pvParameter)
{
    int32_t data;
    SpiffsUpdate update;
    LCDMessage mensagem;
    while (1)
    {
        xSemaphoreTake(xSemaphoreTare, portMAX_DELAY);
        //xSemaphoreGive(xSemaphoreStopReadWeight);
        xSemaphoreGive(xSemaphoreStopKeypad);

        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
        }

        r = hx711_read_average(&dev, SamplesHX711, &data);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
        }

        tare = data;

        ESP_LOGI(TAG, "Tare value: %d", tare);

        sprintf(update.filename, "%s", "hx711_config.txt");
        sprintf(update.json_object, "%s", "tareValue");
        update.value = tare;
        xQueueSend(xSpiffsUpdate, &update, portMAX_DELAY);

        mensagem.lcdEnTimeoutOff = 1;
        mensagem.clear = 1;
        mensagem.line = 0;
        sprintf(mensagem.message, "Tara concluida!");
        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);

        xSemaphoreGive(xSemaphoreSleep);
    }
}

void calibrate_task(void *pvParameter)
{
    int32_t data;
    SpiffsUpdate update;
    LCDMessage mensagem;
    while (1)
    {
        xSemaphoreTake(xSemaphoreCalibrate, portMAX_DELAY);
        //xSemaphoreGive(xSemaphoreStopReadWeight);
        xSemaphoreGive(xSemaphoreStopKeypad);

        esp_err_t r = hx711_wait(&dev, 500);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
        }

        r = hx711_read_average(&dev, SamplesHX711, &data);
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
        }

        calibration = (data - tare) / (float)weightReference;

        ESP_LOGI(TAG, "Calibration value: %f", calibration);

        sprintf(update.filename, "%s", "hx711_config.txt");
        sprintf(update.json_object, "%s", "tareValue");
        update.value = tare;
        xQueueSend(xSpiffsUpdate, &update, portMAX_DELAY);

        mensagem.lcdEnTimeoutOff = 1;
        mensagem.clear = 1;
        mensagem.line = 0;
        sprintf(mensagem.message, "Calibracao");
        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);

        mensagem.clear = 0;
        mensagem.line = 1;
        sprintf(mensagem.message, "concluida!");
        xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);

        xSemaphoreGive(xSemaphoreSleep);
    }
}

void fileHandler_task(void *pvParameters)
{
    tare = readFile("hx711_config.txt", "tareValue");
    calibration = readFile("hx711_config.txt", "calibrationValue");
    SpiffsUpdate update;
    while (1)
    {
        xQueueReceive(xSpiffsUpdate, &update, portMAX_DELAY);

        ESP_LOGI(TAG, "Update SPIFFS: %s", update.json_object);

        updateFile(update.filename, update.json_object, update.value);
    }
}

void keypad_task(void *pvParameters)
{
    init_keypad();
    char keypadChar;
    LCDMessage mensagem;
    int trashTypeKeypad = 0;
    bool messageSent = 0;

    while (1)
    {
        readKeypad();
        // printKeypad();
        keypadChar = getKeypadChar();
        if (keypadChar)
        {
            ESP_LOGI(TAG, "Botão pressionado: %c", keypadChar);
            switch (keypadChar)
            {
            case '*':
                trashTypeKeypad = trashTypeKeypad / 10;
                break;
            case '#':
                ESP_LOGI(TAG, "Tipo selecionado: %d", trashTypeKeypad);
                messageSent = 1;
                trashTypeKeypad = 0;
                xSemaphoreGive(xSemaphoreStopReadWeight);
                xSemaphoreGive(xSemaphoreStopKeypad);
                xTimerStop(xTimerReadWeightTimeout, 0);
                mensagem.lcdEnTimeoutOff = 1;
                mensagem.clear = 1;
                mensagem.line = 0;
                sprintf(mensagem.message, "Peso enviado!");
                xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);
                break;
            default:
                keypadChar -= 48;
                trashTypeKeypad = trashTypeKeypad * 10 + keypadChar;
            }

            if(!messageSent){
                mensagem.lcdEnTimeoutOff = 0;
                mensagem.clear = 0;
                mensagem.line = 1;
                sprintf(mensagem.message, "Tipo: %d", trashTypeKeypad);
                xQueueSend(xMessageLCD, &mensagem, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            else{messageSent = 0;}
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (xSemaphoreTake(xSemaphoreStopKeypad, pdMS_TO_TICKS(10)) == pdPASS)
        {
            trashTypeKeypad = 0;
            ESP_LOGI(TAG, "Keypad Task suspended");
            vTaskSuspend(taskKeypadHandle);
        }
    }
}

void sleep_task(void *pvParameters)
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);
    gpio_wakeup_enable(vibrationSensorPin, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(btnTarePin, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(btnCalibratePin, GPIO_INTR_LOW_LEVEL);

    esp_err_t r = esp_sleep_enable_gpio_wakeup();
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Couldn't config gpio wakeup mode: %d (%s)\n", r, esp_err_to_name(r));
    }

    while(1){
        xSemaphoreTake(xSemaphoreSleep, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "Entering sleep mode");

        vTaskDelay(pdMS_TO_TICKS(3000));

        esp_err_t r = esp_light_sleep_start();
        if (r != ESP_OK)
        {
            ESP_LOGE(TAG, "Couldn't enter sleep mode: %d (%s)\n", r, esp_err_to_name(r));
        }
    }

}

void app_main()
{
    init_interrupts();
    init_hx711();
    init_spiffs();

    /*Criação Queues*/
    xMessageLCD = xQueueCreate(10, sizeof(LCDMessage));
    xVibration = xQueueCreate(1, sizeof(uint32_t));
    xTare = xQueueCreate(1, sizeof(uint32_t));
    xCalibrate = xQueueCreate(1, sizeof(uint32_t));
    xSpiffsUpdate = xQueueCreate(2, sizeof(SpiffsUpdate));

    /*Criação Semaphores*/
    xSemaphoreTare = xSemaphoreCreateBinary();
    if (xSemaphoreTare == NULL)
    {
        ESP_LOGW(TAG, "Não foi possível criar o semáforo");
        esp_restart();
    }

    xSemaphoreCalibrate = xSemaphoreCreateBinary();
    if (xSemaphoreCalibrate == NULL)
    {
        ESP_LOGW(TAG, "Não foi possível criar o semáforo");
        esp_restart();
    }

    xSemaphoreStopKeypad = xSemaphoreCreateBinary();
    if (xSemaphoreStopKeypad == NULL)
    {
        ESP_LOGW(TAG, "Não foi possível criar o semáforo");
        esp_restart();
    }

    xSemaphoreStopReadWeight = xSemaphoreCreateBinary();
    if (xSemaphoreStopReadWeight == NULL)
    {
        ESP_LOGW(TAG, "Não foi possível criar o semáforo");
        esp_restart();
    }

    xSemaphoreSleep = xSemaphoreCreateBinary();
    if (xSemaphoreSleep == NULL)
    {
        ESP_LOGW(TAG, "Não foi possível criar o semáforo");
        esp_restart();
    }

    /*Criação Timers*/
    xTimerReadWeightTimeout = xTimerCreate("TIMER READ WEIGHT TIMEOUT", pdMS_TO_TICKS(measureTimeout), pdTRUE, 0, callBackTimerReadWeightTimeout);
    xTimerLCDOff = xTimerCreate("TIMER LCD OFF", pdMS_TO_TICKS(LCDOffTimeout), pdTRUE, 0, callBackTimerLCDOff);

    /*Criação Tasks*/
    xTaskCreatePinnedToCore(fileHandler_task, "fileHandler_task", 10000, NULL, 1, &taskFileHandlerHandle, 0);
    xTaskCreate(readWeight_task, "readWeight_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &taskReadWeightHandle);
    vTaskSuspend(taskReadWeightHandle);
    xTaskCreate(lcd1602_task, "lcd1602_task", 5120, NULL, 5, &taskLCDHandle);
    xTaskCreate(tare_task, "tare_task", 2048, NULL, 10, &taskTareHandle);
    xTaskCreate(calibrate_task, "calibrate_task", 2048, NULL, 10, &taskCalibrateHandle);
    xTaskCreate(keypad_task, "keypad_task", 2048, NULL, 10, &taskKeypadHandle);
    xSemaphoreGive(xSemaphoreStopKeypad);
    xTaskCreate(sleep_task, "sleep_task", 2048, NULL, 2, &taskSleepHandle);
}
