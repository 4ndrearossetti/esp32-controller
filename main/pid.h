#ifndef PID_H
#define PID_H

typedef struct {
        // Inputs
        float setpoint;          // desired value
        float kp, ki, kd;        // gains
        float output_min;        // saturation lower bound
        float output_max;        // saturation upper bound

        // Internal state
        float prev_feedback;     // feedback from previous call
        float integral;          // accumulated error · dt
        int   has_prev;          // false until the first call completes
        int   integral_clamped;  // diagnostic: was integral frozen?
} PID_Controller;

void pid_init(PID_Controller* pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

float pid_update(PID_Controller* pid, float feedback, float dt);

void pid_reset(PID_Controller* pid);

#endif

