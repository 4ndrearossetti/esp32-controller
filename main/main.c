#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_PORT        I2C_NUM_0
#define PIN_SDA         21
#define PIN_SCL         22
#define I2C_FREQ_HZ     400000
#define MPU6050_ADDR    0x68
#define REG_WHO_AM_I    0x75

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_SDA,
        .scl_io_num       = PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

static esp_err_t mpu6050_read_register(uint8_t reg, uint8_t *out)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);   // repeated START
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, out, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    i2c_init();

    uint8_t who_am_i = 0;
    esp_err_t ret = mpu6050_read_register(REG_WHO_AM_I, &who_am_i);

    if (ret != ESP_OK) {
        printf("I2C transaction failed: %s\n", esp_err_to_name(ret));
        printf("Check wiring: SDA->GPIO%d, SCL->GPIO%d, VCC->3V3\n",
               PIN_SDA, PIN_SCL);
        return;
    }

    if (who_am_i == MPU6050_ADDR) {
        printf("WHO_AM_I: 0x%02X -- OK\n", who_am_i);
    } else {
        printf("WHO_AM_I: 0x%02X -- unexpected (wiring issue or wrong device)\n",
               who_am_i);
    }
}

