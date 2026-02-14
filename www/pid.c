#include "pid.h"
#include "imu.h"   // тут объявлен gyro_z
#include "sbus.h"  // тут rc_channels

float pid_roll_output  = 0.0f;
float pid_pitch_output = 0.0f;
float pid_yaw_output   = 0.0f;

// Коэффициенты только для yaw
static float pid_yaw_kp = 0.09f;   // очень мягко было 0.03 рабочий
static float pid_yaw_ki = 0.0f;
static float pid_yaw_kd = 0.0f;


// Для простоты — без интеграла и D, только P
void PID_Init(void) {
    pid_roll_output  = 0.0f;
    pid_pitch_output = 0.0f;
    pid_yaw_output   = 0.0f;
}


void PID_Update(void) {

    float yaw_set = -(rc_channels[3] - 1024) * 0.5f;

    // 1) Сильно уменьшаем влияние гиры
    float gyro_scale = 0.017f;           // начинаем с ОЧЕНЬ маленького
    float raw_rate   = gyro_z * gyro_scale;

    // 2) Фильтр по yaw_rate
    static float yaw_rate_f = 0;
    yaw_rate_f = yaw_rate_f * 0.7f + raw_rate * 0.3f;
    float yaw_rate = yaw_rate_f;

    float yaw_error = yaw_set - yaw_rate;

    float kp = 1.0f;
    pid_yaw_output = kp * yaw_error;
}


