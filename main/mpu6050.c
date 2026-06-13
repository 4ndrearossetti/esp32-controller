#include "mpu6050.h"
#include "hal/i2c_types.h"
#include <stdint.h>
#include <math.h>

#define REG_WHO_AM_I     0x75
#define WHO_AM_I_VALUE   0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B

static const float ACCEL_SCALE = 9.81f / 16384.0f;
static const float GYRO_SCALE = (float)(M_PI / 180.0) / 131.0f;

static esp_err_t mpu6050_read_register(const mpu6050_t* m, uint8_t reg, uint8_t* out) {
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

static esp_err_t mpu6050_write_register(const mpu6050_t* m, uint8_t reg, uint8_t val) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (m->addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, val, true);

        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(m->port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        return ret;
}

esp_err_t mpu6050_init(mpu6050_t* m, i2c_port_t port, uint8_t addr) {
        m->port = port;
        m->addr = addr;
        m->bias_ax = m->bias_ay = m->bias_az = 0.0f;
        m->bias_gx = m->bias_gy = m->bias_gz = 0.0f;
        return ESP_OK;
}

esp_err_t mpu6050_probe(const mpu6050_t* m) {
        uint8_t who_am_i;
        esp_err_t ret = mpu6050_read_register(m, REG_WHO_AM_I, &who_am_i);
        if (ret != ESP_OK) return ret;
        return (who_am_i == WHO_AM_I_VALUE) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t mpu6050_wake(const mpu6050_t* m) {
        return mpu6050_write_register(m, REG_PWR_MGMT_1, 0x00);
}

esp_err_t mpu6050_read(const mpu6050_t* m, mpu6050_sample_t* out) {
        uint8_t buf[14];

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (m->addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, REG_ACCEL_XOUT_H, true);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (m->addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, buf, 14, I2C_MASTER_LAST_NACK);

        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(m->port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) return ret;

        int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

        out->ax = ax_raw * ACCEL_SCALE - m->bias_ax;
        out->ay = ay_raw * ACCEL_SCALE - m->bias_ay;
        out->az = az_raw * ACCEL_SCALE - m->bias_az;
        out->gx = gx_raw * GYRO_SCALE  - m->bias_gx;
        out->gy = gy_raw * GYRO_SCALE  - m->bias_gy;
        out->gz = gz_raw * GYRO_SCALE  - m->bias_gz;

        return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_t* m, int n_samples) {
        mpu6050_sample_t s;
        float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
        float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;

        for (int i = 0; i < n_samples; i++) {
                esp_err_t ret = (mpu6050_read(m, &s));
                if (ret != ESP_OK) return ret;

                sum_ax += s.ax;
                sum_ay += s.ay;
                sum_az += s.az;
                sum_gx += s.gx;
                sum_gy += s.gy;
                sum_gz += s.gz;

                vTaskDelay(pdMS_TO_TICKS(5));
        }

        m->bias_ax = sum_ax / n_samples;
        m->bias_ay = sum_ay / n_samples;
        m->bias_az = sum_az / n_samples - 9.81f;
        m->bias_gx = sum_gx / n_samples;
        m->bias_gy = sum_gy / n_samples;
        m->bias_gz = sum_gz / n_samples;

        return ESP_OK;
}

