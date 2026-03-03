#include "motors.h"
#include "sbus.h"
#include "pid.h"

int debug_m1, debug_m2, debug_m3, debug_m4;


void Mixer_Update(void) {

    float thr = rc_channels[2];

    float roll  = pid_roll_output;
    float pitch = pid_pitch_output;
    float yaw   = -pid_yaw_output; 

    // === масштаб газа ===
    thr = 1050.0f + (thr - 240.0f) * 1.2f;
    if (thr < 1050) thr = 1050;
    if (thr > 2000) thr = 2000;

    // === миксер под схему:
    // M1 (front-left, CW)
    // M4 (front-right, CCW)
    // M2 (rear-left, CCW)
    // M3 (rear-right, CW)




    
    int m1 = (int)(thr - pitch + roll - yaw);
    int m4 = (int)(thr - pitch - roll + yaw);
    int m2 = (int)(thr + pitch + roll + yaw);
    int m3 = (int)(thr + pitch - roll - yaw); 



    if (m1 < 1050) m1 = 1050; if (m1 > 2000) m1 = 2000;
    if (m2 < 1050) m2 = 1050; if (m2 > 2000) m2 = 2000;
    if (m3 < 1050) m3 = 1050; if (m3 > 2000) m3 = 2000;
    if (m4 < 1050) m4 = 1050; if (m4 > 2000) m4 = 2000;

    debug_m1 = m1;
    debug_m2 = m2;
    debug_m3 = m3;
    debug_m4 = m4;

    Set_Motor_Individual(m1, m2, m3, m4);
}