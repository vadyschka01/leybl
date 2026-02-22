#ifndef PID_H
#define PID_H

extern float pid_roll_output;
extern float pid_pitch_output;
extern float pid_yaw_output;

void PID_Init(void);
void PID_Update(void);

#endif
