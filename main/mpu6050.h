#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

typedef struct {
        i2c_port_t      port;
        uint8_t         addr;
        float           bias_ax, bias_ay, bias_az;
        float           bias_gx, bias_gy, bias_gz;
} mpu6050_t;

typedef struct {
        float ax, ay, az;       // m/s^2, sensor frame
        float gx, gy, gz;       // rad/s, sensor frame
} mpu6050_sample_t;

esp_err_t mpu6050_init(mpu6050_t *m, i2c_port_t port, uint8_t addr);
esp_err_t mpu6050_probe(const mpu6050_t *m);
esp_err_t mpu6050_wake(const mpu6050_t *m);
esp_err_t mpu6050_read(const mpu6050_t *m, mpu6050_sample_t *out);
esp_err_t mpu6050_set_gyro_range(const mpu6050_t *m);
esp_err_t mpu6050_calibrate(mpu6050_t *m, int n_samples);

#endif

