#include "mpu6050.h"

#define REG_WHO_AM_I    0x75
#define WHO_AM_I_VALUE  0x68

static esp_err_t mpu6050_read_register(const mpu6050_t* m, uint8_t reg, uint8_t *out)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m->addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, out, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(m->port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(mpu6050_t* m, i2c_port_t port, uint8_t addr) {
        m->port = port;
        m->addr = addr;
        return ESP_OK;
}

esp_err_t mpu6050_probe(const mpu6050_t* m) {
        uint8_t who_am_i;
        esp_err_t ret = mpu6050_read_register(m, REG_WHO_AM_I, &who_am_i);
        if (ret != ESP_OK) return ret;
        return (who_am_i == WHO_AM_I_VALUE) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

