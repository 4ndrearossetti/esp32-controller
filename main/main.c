#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_GPIO GPIO_NUM_2

static volatile int loop_count = 0;

void control_timer_cb(void *arg)
{
    gpio_set_level(LED_GPIO, !gpio_get_level(LED_GPIO));
    loop_count++;
}

void timer_init(void)
{
    const esp_timer_create_args_t args = {
        .callback = &control_timer_cb,
        .name = "control_loop",
        .dispatch_method = ESP_TIMER_TASK,
    };

    esp_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 5000)); // 200 Hz
}

void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    timer_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("200 Hz loop: %d ticks in last second\n", loop_count);
        loop_count = 0;
    }
}

