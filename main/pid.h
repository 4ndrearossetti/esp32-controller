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
} pid_controller_t;

void pid_init(pid_controller_t* pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

float pid_update(pid_controller_t* pid, float feedback, float dt);

void pid_reset(pid_controller_t* pid);

#endif

