#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"

typedef struct {
        float x, y, z;
        float vx, vy, vz;
        float roll, pitch, yaw;
        float p, q, r;
} VehicleState;

typedef struct {
        float altitude;
        float roll;
        float pitch;
        float yaw_rate;
} Setpoints;

typedef struct {
        float thrust_cmd;
        float roll_cmd;
        float pitch_cmd;
        float yaw_cmd;
} ControlOutput;

typedef struct {
        PID_Controller altitude;
        PID_Controller roll_angle;
        PID_Controller roll_rate;
        PID_Controller pitch_angle;
        PID_Controller pitch_rate;
        PID_Controller yaw_rate;
} Controllers;

ControlOutput control_update(Controllers* c, const VehicleState* s,
                             const Setpoints* sp, float dt);

void control_reset(Controllers* c);

#endif

