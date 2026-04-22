#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

void Motors_Init(void);
void Set_Motors(int val);
void Set_Motor_Individual(int m1, int m2, int m3, int m4);
void Motors_StartupDelay(void);
void Motors_Stop(void);
void Motors_Idle(void);
void Motors_Arm(void);
void Motors_Disarm(void);



#endif
