#ifndef MIXER_H
#define MIXER_H

#define F_MOTOR_MAX 2.5f  // N, per motor, 4:1 TWR airframe

// Convert abstract control commands into per-motor thrusts.
void mixer_compute(float thrust_cmd, float roll_cmd,
                   float pitch_cmd, float yaw_cmd,
                   float motor_thrusts_N[4]);

#endif

