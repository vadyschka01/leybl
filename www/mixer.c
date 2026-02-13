#include "sbus.h"
#include "motors.h"
#include "pid.h"
void Mixer_Update(void) {

    // === 1. Читаем каналы ===
    int thr   = rc_channels[2];   // throttle
    int roll  = rc_channels[0];   // roll
    int pitch = rc_channels[1];   // pitch
    int yaw   = rc_channels[3];   // yaw

    // === 2. Приводим к диапазону -500..+500 ===
    roll  = pid_roll_output;  // pid ссылка
    pitch = pid_pitch_output;
    yaw = pid_yaw_output + anti_torque;  // + anty torque



    // === 3. Масштабируем ===
    roll  *= 0.5;
    pitch *= 0.5;
    yaw   *= 0.5;

    // === 4. Газ в диапазон 1050–2000 ===
    thr = 1050 + (thr - 240) * 1.2;
    if (thr < 1050) thr = 1050;
    if (thr > 2000) thr = 2000;

    // === 5. Миксер ===
    int m1 = thr - roll + pitch - yaw;   // front-left
    int m4 = thr + roll + pitch + yaw;   // front-right
    int m2 = thr - roll - pitch + yaw;   // rear-left
    int m3 = thr + roll - pitch - yaw;   // rear-right


    // === 6. Ограничение ===
    if (m1 < 1050) m1 = 1050; if (m1 > 2000) m1 = 2000;
    if (m2 < 1050) m2 = 1050; if (m2 > 2000) m2 = 2000;
    if (m3 < 1050) m3 = 1050; if (m3 > 2000) m3 = 2000;
    if (m4 < 1050) m4 = 1050; if (m4 > 2000) m4 = 2000;

    // === 7. Отправляем ===
    Set_Motor_Individual(m1, m2, m3, m4);
}
