#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

typedef struct {
    i2c_port_t port;
    uint8_t    addr;        // 7-bit, usually 0x68
} mpu6050_t;

typedef struct {
    float ax, ay, az;       // m/s^2, sensor frame
    float gx, gy, gz;       // rad/s, sensor frame
} mpu6050_sample_t;

esp_err_t mpu6050_init(mpu6050_t *m, i2c_port_t port, uint8_t addr);
esp_err_t mpu6050_probe(const mpu6050_t *m);           // checks WHO_AM_I
esp_err_t mpu6050_wake(const mpu6050_t *m);            // clears SLEEP bit
esp_err_t mpu6050_read(const mpu6050_t *m, mpu6050_sample_t *out);

#endif

