#include <stdio.h>
#include <math.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "imu.h"
#include "mpu6050.h"
#include "control_loop.h"

static TaskHandle_t control_task_handle = NULL;

static void control_task(void *arg) {
        mpu6050_t imu;
        mpu6050_sample_t sample;
        imu_t attitude;

        imu_init(&attitude, 0.98f);

        if (mpu6050_init(&imu, I2C_NUM_0, 0x68) != ESP_OK) {
                printf("init failed\n");
                vTaskDelete(NULL);
        }

        if (mpu6050_probe(&imu) != ESP_OK) {
                printf("probe failed\n");
                vTaskDelete(NULL);
        }

        if (mpu6050_wake(&imu) != ESP_OK) {
                printf("wake failed\n");
                vTaskDelete(NULL);
        }

        if (mpu6050_set_gyro_range(&imu) != ESP_OK) {
                printf("set gyro range failed\n");
                vTaskDelete(NULL);
        }

        if (mpu6050_calibrate(&imu, 200) != ESP_OK) {
                printf("calibrate failed\n");
                vTaskDelete(NULL);
        }

        const float DT = 0.005f;
        int log_counter = 0;

        while (1) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // block until timer ticks

                if (mpu6050_read(&imu, &sample) == ESP_OK) {
                        imu_update(&attitude, &sample, DT);

                        if (++log_counter >= 40) {              // 5Hz logging
                                log_counter = 0;
                                printf("roll=%+6.1f pitch=%+6.1f yaw=%+6.1f (deg)\n",
                                        attitude.roll  * 180.0f / M_PI,
                                        attitude.pitch * 180.0f / M_PI,
                                        attitude.yaw   * 180.0f / M_PI);
                        }
                }
        }
}

static void timer_callback(void *arg) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(control_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
}

void control_loop_start(void) {
        xTaskCreatePinnedToCore(
                control_task,
                "control",
                4096,
                NULL,
                10,
                &control_task_handle,
                1
        );

        const esp_timer_create_args_t timer_args = {
                .callback = &timer_callback,
                .name = "control_tick",
                .dispatch_method = ESP_TIMER_TASK,
        };
        esp_timer_handle_t timer;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 5000));  // 5000 us = 200Hz
}

