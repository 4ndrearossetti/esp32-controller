#ifndef IMU_H
#define IMU_H

#include "mpu6050.h"

typedef struct {
        float roll;
        float pitch;
        float yaw;
        float alpha;
} imu_t;

void imu_init(imu_t *imu, float alpha);
void imu_update(imu_t *imu, const mpu6050_sample_t *sample, float dt);

#endif

