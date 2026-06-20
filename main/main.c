#include "driver/i2c.h"
#include "control_loop.h"

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
        control_loop_start();
}

