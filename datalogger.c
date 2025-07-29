#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "config.h"
#include "pico/bootrom.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/rtc.h"

#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "sd_card.h"

SemaphoreHandle_t xBuzzerSemaphore, xButtonASemaphore, xButtonBSemaphore;
QueueHandle_t xDisplayQueue, xLedQueue;

void gpio_irq_handler(uint gpio, uint32_t events)
{
    static uint32_t last_time_button_A = 0;
    static uint32_t last_time_button_B = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == BUTTON_A && (now - last_time_button_A > DEBOUNCE_TIME))
    {
        last_time_button_A = now;
        xSemaphoreGiveFromISR(xButtonASemaphore, &xHigherPriorityTaskWoken);
    }
    if (gpio == BUTTON_B && (now - last_time_button_B > DEBOUNCE_TIME))
    {
        last_time_button_B = now;
        xSemaphoreGiveFromISR(xButtonBSemaphore, &xHigherPriorityTaskWoken);
    }
    if (gpio == BUTTON_J)
    {
        reset_usb_boot(0, 0);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vControlTask(void *pvParameters)
{
    enum MODE current_mode = WAITING;
    bool is_mounted = false;
    FIL file;
    static ImuSample data_buffer[MAX_SAMPLES];
    uint32_t samples_in_buffer = 0;

    void update_system_state(enum MODE new_mode)
    {
        current_mode = new_mode;
        DisplayMessage msg = {.new_mode = current_mode, .sample_count = samples_in_buffer};
        xQueueSend(xDisplayQueue, &msg, 0);
        xQueueSend(xLedQueue, &msg, 0);
    };

    update_system_state(WAITING);

    while (true)
    {
        if (xSemaphoreTake(xButtonASemaphore, 0) == pdTRUE)
        {
            if (current_mode == READY)
            {
                samples_in_buffer = 0;
                xSemaphoreGive(xBuzzerSemaphore);
                update_system_state(CAPTURING);
            }
            else if (current_mode == CAPTURING)
            {
                xSemaphoreGive(xBuzzerSemaphore);
                vTaskDelay(pdMS_TO_TICKS(80));
                xSemaphoreGive(xBuzzerSemaphore);
                update_system_state(ACESSING);
            }
            else if (current_mode == ACESSING)
            {
                xSemaphoreGive(xBuzzerSemaphore);
                vTaskDelay(pdMS_TO_TICKS(80));
                xSemaphoreGive(xBuzzerSemaphore);
                update_system_state(READY);
            }
            else if (current_mode == ERROR)
            {
                update_system_state(WAITING);
            }
        }

        if (xSemaphoreTake(xButtonBSemaphore, 0) == pdTRUE)
        {
            if (!is_mounted)
            {
                update_system_state(SDMOUNT);
                vTaskDelay(pdMS_TO_TICKS(50));
                if (mount_sd_card())
                {
                    is_mounted = true;
                    xSemaphoreGive(xBuzzerSemaphore);
                    update_system_state(READY);
                }
                else
                {
                    is_mounted = false;
                    update_system_state(ERROR);
                }
            }
            else if (current_mode == READY)
            {
                update_system_state(ACESSING);
                unmount_sd_card();
                is_mounted = false;
                xSemaphoreGive(xBuzzerSemaphore);
                update_system_state(WAITING);
            }
        }

        if (current_mode == CAPTURING)
        {
            if (samples_in_buffer < MAX_SAMPLES)
            {
                int16_t aceleracao[3], gyro[3], temp;
                mpu6050_read_raw(aceleracao, gyro, &temp);

                rtc_get_datetime(&data_buffer[samples_in_buffer].timestamp);
                data_buffer[samples_in_buffer].sample_num = samples_in_buffer + 1;
                data_buffer[samples_in_buffer].accel_x = aceleracao[0] / 16384.0f;
                data_buffer[samples_in_buffer].accel_y = aceleracao[1] / 16384.0f;
                data_buffer[samples_in_buffer].accel_z = aceleracao[2] / 16384.0f;
                data_buffer[samples_in_buffer].gyro_x = gyro[0] / 131.0f;
                data_buffer[samples_in_buffer].gyro_y = gyro[1] / 131.0f;
                data_buffer[samples_in_buffer].gyro_z = gyro[2] / 131.0f;
                samples_in_buffer++;
                if (samples_in_buffer % 5 == 0)
                {
                    update_system_state(CAPTURING);
                }
            }
            else
            {
                update_system_state(ACESSING);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (current_mode == ACESSING)
        {
            FRESULT fr;
            UINT bytes_written;

            fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
            if (fr != FR_OK)
            {
                update_system_state(ERROR);
                continue;
            }

            const char *header = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z,data_hora\n";
            f_write(&file, header, strlen(header), &bytes_written);

            char buffer[150];
            bool write_error = false;
            for (int i = 0; i < samples_in_buffer; i++)
            {
                ImuSample *s = &data_buffer[i];
                int len = snprintf(buffer, sizeof(buffer),
                                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%02d/%02d/%04d-%02d:%02d:%02d\n",
                                   s->sample_num, s->accel_x, s->accel_y, s->accel_z,
                                   s->gyro_x, s->gyro_y, s->gyro_z,
                                   s->timestamp.day, s->timestamp.month, s->timestamp.year,
                                   s->timestamp.hour, s->timestamp.min, s->timestamp.sec);

                fr = f_write(&file, buffer, len, &bytes_written);
                if (fr != FR_OK || bytes_written < len)
                {
                    write_error = true;
                    break;
                }

                if (i > 0 && i % 100 == 0)
                {
                    DisplayMessage prog_msg = {.new_mode = ACESSING, .sample_count = i};
                    xQueueSend(xDisplayQueue, &prog_msg, 0);
                }
            }

            f_sync(&file);
            f_close(&file);

            if (write_error)
            {
                update_system_state(ERROR);
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void vBuzzerTask(void *pvParameters)
{
    pwm_init_buzzer(BUZZER_PIN_A);
    while (true)
    {
        if (xSemaphoreTake(xBuzzerSemaphore, portMAX_DELAY) == pdTRUE)
        {
            beep(BUZZER_PIN_A, 100);
        }
    }
}

void vDisplayTask(void *pvParameters)
{
    ssd1306_t ssd;
    ssd1306_init(&ssd, 128, 64, false, DISPLAY_ADDRESS, I2C_PORT_DISP);
    ssd1306_config(&ssd);

    DisplayMessage msg = {.new_mode = WAITING, .sample_count = 0};

    ssd1306_fill(&ssd, 0);
    ssd1306_send_data(&ssd);

    while (true)
    {
        xQueueReceive(xDisplayQueue, &msg, portMAX_DELAY);
        ssd1306_fill(&ssd, 0);
        ssd1306_rect(&ssd, 2, 2, 124, 60, true, false);
        ssd1306_draw_string(&ssd, "DATALOGGER", 25, 5);
        ssd1306_line(&ssd, 3, 15, 125, 15, true);
        char buffer[32];
        switch (msg.new_mode)
        {
        case WAITING:
            ssd1306_draw_string(&ssd, "Aperte B para", 8, 22);
            ssd1306_draw_string(&ssd, "montar o SD", 8, 34);
            ssd1306_draw_string(&ssd, "card", 8, 46);
            break;
        case SDMOUNT:
            ssd1306_draw_string(&ssd, "Montando o ", 8, 22);
            ssd1306_draw_string(&ssd, "cartao SD...", 8, 34);
            break;
        case READY:
            ssd1306_draw_string(&ssd, "Pronto para", 8, 22);
            ssd1306_draw_string(&ssd, "gravar.", 8, 34);
            break;
        case CAPTURING:
            ssd1306_draw_string(&ssd, "Capturando...", 8, 22);
            sprintf(buffer, "Amostras: %lu", msg.sample_count);
            ssd1306_draw_string(&ssd, buffer, 8, 34);
            break;
        case ACESSING:
            ssd1306_draw_string(&ssd, "Salvando...", 8, 22);
            sprintf(buffer, "Salvos: %lu", msg.sample_count);
            ssd1306_draw_string(&ssd, buffer, 8, 34);
            break;
        case ERROR:
            ssd1306_draw_string(&ssd, "ERRO!", 8, 22);
            ssd1306_draw_string(&ssd, "Aperte A para", 8, 34);
            ssd1306_draw_string(&ssd, "Reset", 8, 46);

            break;
        default:
            ssd1306_draw_string(&ssd, "Estado desconhecido", 8, 22);
            break;
        }
        ssd1306_send_data(&ssd);
    }
}

void vLedTask(void *pvParameters)
{
    gpio_init(LED_PIN_GREEN);
    gpio_init(LED_PIN_RED);
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

    DisplayMessage msg = {.new_mode = WAITING};

    while (true)
    {
        xQueueReceive(xLedQueue, &msg, pdMS_TO_TICKS(250));
        bool led_state = (to_ms_since_boot(get_absolute_time()) / 250) % 2;

        switch (msg.new_mode)
        {
        case WAITING: // Branco
            gpio_put(LED_PIN_RED, true);
            gpio_put(LED_PIN_GREEN, true);
            gpio_put(LED_PIN_BLUE, true);
            break;
        case SDMOUNT: // Amarelo
            gpio_put(LED_PIN_RED, true);
            gpio_put(LED_PIN_GREEN, true);
            gpio_put(LED_PIN_BLUE, false);
            break;
        case READY: // Verde
            gpio_put(LED_PIN_RED, false);
            gpio_put(LED_PIN_GREEN, true);
            gpio_put(LED_PIN_BLUE, false);
            break;
        case CAPTURING: // Vermelho
            gpio_put(LED_PIN_RED, true);
            gpio_put(LED_PIN_GREEN, false);
            gpio_put(LED_PIN_BLUE, false);
            break;
        case ACESSING: // Azul Piscando
            gpio_put(LED_PIN_RED, false);
            gpio_put(LED_PIN_GREEN, false);
            gpio_put(LED_PIN_BLUE, led_state);
            break;
        case ERROR: // Roxo Piscando
            gpio_put(LED_PIN_RED, led_state);
            gpio_put(LED_PIN_GREEN, false);
            gpio_put(LED_PIN_BLUE, led_state);
            break;
        default: // Apagado
            gpio_put(LED_PIN_RED, false);
            gpio_put(LED_PIN_GREEN, false);
            gpio_put(LED_PIN_BLUE, false);
        }
    }
}

int main()
{
    stdio_init_all();
    rtc_init();
    datetime_t t = {
        .year = 2024,
        .month = 7,
        .day = 26,
        .dotw = 5,
        .hour = 18,
        .min = 00,
        .sec = 00};
    rtc_set_datetime(&t);

    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_init(BUTTON_J);
    gpio_set_dir(BUTTON_J, GPIO_IN);
    gpio_pull_up(BUTTON_J);

    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTON_J, GPIO_IRQ_EDGE_FALL, true);

    xBuzzerSemaphore = xSemaphoreCreateBinary();
    xButtonASemaphore = xSemaphoreCreateBinary();
    xButtonBSemaphore = xSemaphoreCreateBinary();
    xDisplayQueue = xQueueCreate(5, sizeof(DisplayMessage));
    xLedQueue = xQueueCreate(5, sizeof(DisplayMessage));

    xTaskCreate(vDisplayTask, "DisplayTask", 1024, NULL, 2, NULL);
    xTaskCreate(vLedTask, "LedTask", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "BuzzerTask", 256, NULL, 1, NULL);
    xTaskCreate(vControlTask, "ControlTask", 2048, NULL, 3, NULL);

    vTaskStartScheduler();

    while (true)
    {
    };
}