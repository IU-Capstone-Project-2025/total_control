#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#define ENC_LINEAR_GPIO_1 22
#define ENC_LINEAR_GPIO_2 23
#define GPIO_PIN_MASK(PIN) (1ULL<<PIN)
#define ESP_INTR_FLAG_DEFAULT 0

volatile int64_t enc_linear_position = 0;

static void IRAM_ATTR enc_linear_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    enc_linear_position += (gpio_get_level(ENC_LINEAR_GPIO_1) != gpio_get_level(ENC_LINEAR_GPIO_2)) ? 1 : -1;
}

static void periodic_timer_callback(void* arg)
{
    printf("%lld\n", enc_linear_position);
}

void app_main(void)
{
    gpio_config_t io_conf;
    
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_PIN_MASK(ENC_LINEAR_GPIO_1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_PIN_MASK(ENC_LINEAR_GPIO_2);
    // Other settings are same as for previous pin
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENC_LINEAR_GPIO_1, enc_linear_isr_handler, (void*) ENC_LINEAR_GPIO_1);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000)); // ns

    //    printf("%lld\n", enc_linear_position);
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}