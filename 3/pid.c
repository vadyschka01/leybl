#include "pid.h"
#include "imu.h"   // тут объявлен gyro_z
#include "sbus.h"  // тут rc_channels

float pid_roll_output  = 0.0f;
float pid_pitch_output = 0.0f;
float pid_yaw_output   = 0.0f;

float pitch_angle_offset = 0.0f;   // софт-трим
float roll_angle_offset  = 0.0f;

static float last_roll_error = 0.0f;  // для производной

// Коэффициенты только для yaw (ось Z)
static float pid_yaw_kp = 0.0f;   // 0.5 рабочий
static float pid_yaw_ki = 0.0f;
static float pid_yaw_kd = 0.0f;

// Коэффициенты только для roll (ось X)
static float pid_roll_kp = 0.0f;   //0.3
static float pid_roll_ki = 0.0f;
static float pid_roll_kd = 0.0f;   //0.5

// Коэффициенты только для pitch (ось Y)
static float pid_pitch_kp = 0.0f;   //  0.5
static float pid_pitch_ki = 0.0f;
static float pid_pitch_kd = 0.0f;   // 1.0




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

    pid_yaw_output = pid_yaw_kp * yaw_error;
    
    // === ВНЕШНИЙ ANGLE PID для pitch/roll ===

    // желаемый угол от стиков (макс ~ ±25°)
    float pitch_angle_set = -(rc_channels[0] - 1024) * 0.05f;
    float roll_angle_set  =  -(rc_channels[1] - 1024) * 0.05f;

    // ошибка по углу
    /*float pitch_angle_error = pitch_angle_set - pitch_angle;
    float roll_angle_error  = roll_angle_set  - roll_angle; */
    
    float pitch_angle_error = pitch_angle_set - (pitch_angle - pitch_angle_offset);
    float roll_angle_error  = roll_angle_set  - (roll_angle  - roll_angle_offset);


    // коэффициенты внешнего контура
    float angle_kp = 1.0f;   // 3–5???для roll 3.0 

    // выход внешнего PID — это желаемая скорость вращения
    float pitch_rate_set = angle_kp * pitch_angle_error;
    float roll_rate_set  = angle_kp * roll_angle_error;

    // === PITCH rate PID (ось Y) ===

    // 1) Setpoint от внеш контура
    float pitch_set = pitch_rate_set;;

    // 2) Скорость вращения по оси Y
    float gyro_scale_pitch = 0.0017f;
    float raw_pitch_rate   = gyro_y * gyro_scale_pitch;

    // 3) Фильтр скорости
    static float pitch_rate_f = 0;
    pitch_rate_f = pitch_rate_f * 0.7f + raw_pitch_rate * 0.3f;
    float pitch_rate = pitch_rate_f;

    // 4) Ошибка
    float pitch_error = pitch_set - pitch_rate;

    // 5) P-контур
    pid_pitch_output = pid_pitch_kp * pitch_error;
    
    
    
    // === ROLL rate PID (ось X) ===

    // 1) Setpoint от внеш контура
    float roll_set = roll_rate_set;


    // 2) Скорость вращения по оси X
    float gyro_scale_roll = 0.007f;
    float raw_roll_rate   = gyro_x * gyro_scale_roll;

    // 3) Фильтр скорости
    static float roll_rate_f = 0;
    roll_rate_f = roll_rate_f * 0.7f + raw_roll_rate * 0.3f;
    float roll_rate = roll_rate_f;

    // 4) Ошибка
    float roll_error = roll_set - roll_rate;

    // 5) PID-контур (P + D)
    float roll_deriv = roll_error - last_roll_error;
    last_roll_error = roll_error;
    
    pid_roll_output = pid_roll_kp * roll_error + pid_roll_kd * roll_deriv;



}