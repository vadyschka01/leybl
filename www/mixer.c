#include "motors.h"
#include "sbus.h"
#include "pid.h"

void Mixer_Update(void) {

    float thr = rc_channels[2];

    // === yaw: комбинируем прямой стик + PID ===
    float yaw_stick = -(rc_channels[3] - 1024) * 0.5f;   // как у тебя работало
    float yaw_pid   = pid_yaw_output * 0.5f;             // мягкая добавка

    float yaw = pid_yaw_output; // без *0.5f



    // roll/pitch пока 0
    float roll  = 0.0f;
    float pitch = 0.0f;

    // === масштаб газа ===
    thr = 1050.0f + (thr - 240.0f) * 1.2f;
    if (thr < 1050) thr = 1050;
    if (thr > 2000) thr = 2000;

    // === миксер под твою схему:
    // M1 (front-left, CW)
    // M4 (front-right, CCW)
    // M2 (rear-left, CCW)
    // M3 (rear-right, CW)
    int m1 = (int)(thr - roll + pitch + yaw);
    int m4 = (int)(thr + roll + pitch - yaw);
    int m2 = (int)(thr - roll - pitch - yaw);
    int m3 = (int)(thr + roll - pitch + yaw);

    if (m1 < 1050) m1 = 1050; if (m1 > 2000) m1 = 2000;
    if (m2 < 1050) m2 = 1050; if (m2 > 2000) m2 = 2000;
    if (m3 < 1050) m3 = 1050; if (m3 > 2000) m3 = 2000;
    if (m4 < 1050) m4 = 1050; if (m4 > 2000) m4 = 2000;

    Set_Motor_Individual(m1, m2, m3, m4);
}
