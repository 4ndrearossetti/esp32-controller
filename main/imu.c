#include "imu.h"
#include <math.h>

void imu_init(imu_t* imu, float alpha) {
        imu->roll  = 0.0f;
        imu->pitch = 0.0f;
        imu->yaw   = 0.0f;
        imu->alpha = alpha;
}

void imu_update(imu_t* imu, const mpu6050_sample_t* sample, float dt) {
        float roll_acc = atan2f(sample->ay, sample->az);
        float pitch_acc = atan2f(-sample->ax, sqrtf(sample->ay * sample->ay
                                                   + sample->az * sample->az));

        imu->roll = imu->alpha * (imu->roll + sample->gx*dt) + (1.0f - imu->alpha) * roll_acc;
        imu->pitch = imu->alpha * (imu->pitch + sample->gy*dt) + (1.0f - imu->alpha) * pitch_acc;
        imu->yaw = imu->yaw + sample->gz*dt;
}

