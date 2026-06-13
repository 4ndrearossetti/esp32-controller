#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "mpu6050.h"

#define PIN_SDA      21
#define PIN_SCL      22
#define I2C_FREQ_HZ  400000

static void i2c_init(void) {
        i2c_config_t conf = {
                .mode             = I2C_MODE_MASTER,
                .sda_io_num       = PIN_SDA,
                .scl_io_num       = PIN_SCL,
                .sda_pullup_en    = GPIO_PULLUP_DISABLE,
                .scl_pullup_en    = GPIO_PULLUP_DISABLE,
                .master.clk_speed = I2C_FREQ_HZ,
        };
        ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void) {
        i2c_init();

        mpu6050_t imu;
        mpu6050_sample_t imu_sample;
        mpu6050_init(&imu, I2C_NUM_0, 0x68);

        if (mpu6050_probe(&imu) != ESP_OK) {
                printf("probe failed\n");
                return;
        }
        printf("probe OK\n");

        if (mpu6050_wake(&imu) != ESP_OK) {
                printf("wake failed\n");
                return;
        }
        printf("wake OK\n");

        if (mpu6050_calibrate(&imu, 200) != ESP_OK) {
                printf("calibrate failed\n");
                return;
        }
        printf("calibrate OK\n");

        if (mpu6050_read(&imu, &imu_sample) != ESP_OK) {
            printf("read failed\n");
            return;
        }
        printf("read OK\n");

        printf("\n=== SENSOR DATA ===\n");
        printf("ax≈ %f\t ay≈ %f\t az≈ %f\n gx≈ %f\t gy≈ %f\t gz≈ %f\n", 
                imu_sample.ax, imu_sample.ay, imu_sample.az,
                imu_sample.gx, imu_sample.gy, imu_sample.gz);

        printf("\n=== SENSOR BIASES ===\n");
        printf("bias accel: %f\t %f\t %f\nbias gyro: %f\t %f\t %f\n", 
                imu.bias_ax, imu.bias_ay, imu.bias_az,
                imu.bias_gx, imu.bias_gy, imu.bias_gz);
}

