#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"

typedef struct {
        float x, y, z;
        float vx, vy, vz;
        float roll, pitch, yaw;
        float p, q, r;
} vehicle_state_t;

typedef struct {
        float altitude;
        float roll;
        float pitch;
        float yaw_rate;
} setpoints_t;

typedef struct {
        float thrust_cmd;
        float roll_cmd;
        float pitch_cmd;
        float yaw_cmd;
} control_output_t;

typedef struct {
        pid_controller_t altitude;
        pid_controller_t roll_angle;
        pid_controller_t roll_rate;
        pid_controller_t pitch_angle;
        pid_controller_t pitch_rate;
        pid_controller_t yaw_rate;
} controllers_t;

control_output_t control_update(controllers_t* c, const vehicle_state_t* s,
                             const setpoints_t* sp, float dt);

void control_reset(controllers_t* c);

#endif

