#include "pid.h"
#include "imu.h"
#include "sbus.h"

float anti_torque = 0; // --- anty torque

// PID структуры
PID_t pid_roll;
PID_t pid_pitch;
PID_t pid_yaw;

// Выходы PID
float pid_roll_output = 0;
float pid_pitch_output = 0;
float pid_yaw_output = 0;

// Настройки PID (можешь менять)
#define DT 0.002f   // 2 мс (500 Гц)
#define PID_LIMIT 400.0f

void PID_Init(void) {
    pid_roll.kp  = 0.01f;
    pid_roll.ki  = 0.0f;
    pid_roll.kd  = 1.2f;

    pid_pitch.kp = 0.01f;
    pid_pitch.ki = 0.0f;
    pid_pitch.kd = 1.2f;

    pid_yaw.kp   = 0.01f;
    pid_yaw.ki   = 0.0f;
    pid_yaw.kd   = 1.2f;
}


static float PID_Compute(PID_t *pid, float error) {

    pid->integral += error * DT;
    float derivative = (error - pid->last_error) / DT;
    pid->last_error = error;

    float out = pid->kp * error +
                pid->ki * pid->integral +
                pid->kd * derivative;

    // Ограничение
    if (out > PID_LIMIT) out = PID_LIMIT;
    if (out < -PID_LIMIT) out = -PID_LIMIT;

    return out;
}

void PID_Update(void) {
    
    // === 1. Читаем стики ===
    float roll_set  = (rc_channels[0] - 1024) * 0.2f;
    float pitch_set = (rc_channels[1] - 1024) * 0.2f;
    float yaw_set   = (rc_channels[3] - 1024) * 0.2f;

    // === 2. Читаем гироскоп ===
    float roll_rate  = gyro_x;   // deg/s
    float pitch_rate = gyro_y;
    float yaw_rate   = gyro_z;

    // === 3. Ошибки ===
    float roll_error  = roll_set  - roll_rate;
    float pitch_error = pitch_set - pitch_rate;
    float yaw_error   = yaw_set   - yaw_rate;

    // === 4. PID ===
    pid_roll_output  = PID_Compute(&pid_roll,  roll_error);
    pid_pitch_output = PID_Compute(&pid_pitch, pitch_error);
    pid_yaw_output   = PID_Compute(&pid_yaw,   yaw_error);
    
        // === Anti-torque compensation ===
    // Компенсация реактивного момента при газе
    float anti_torque = gyro_z * 0.02f;

}
