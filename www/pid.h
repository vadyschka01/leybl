#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;
} PID_t;

void PID_Init(void);
void PID_Update(void);

extern float pid_roll_output;
extern float pid_pitch_output;
extern float pid_yaw_output;

extern float anti_torque; // ---anty torque


#endif
