#include "pid.h"
#include "imu.h"   
#include "sbus.h"  

float pid_roll_output  = 0.0f;
float pid_pitch_output = 0.0f;
float pid_yaw_output   = 0.0f;

float roll_rate_set = 0.0f;
float pitch_rate_set = 0.0f;
float yaw_rate_set = 0.0f;

float roll_rate = 0.0f;
float pitch_rate = 0.0f;
float yaw_rate = 0.0f;


static float last_roll_error = 0.0f;  // для производной

// Коэффициенты только для yaw (ось Z)
static float pid_yaw_kp = 0.0f;   // 0.15
static float pid_yaw_ki = 0.0f;
static float pid_yaw_kd = 3.0f;  //0.17

// Коэффициенты только для roll (ось X)
static float pid_roll_kp = 0.5f;   //0.09
static float pid_roll_ki = 0.0f;
static float pid_roll_kd = 0.1f;   //0.2

// Коэффициенты только для pitch (ось Y)
static float pid_pitch_kp = 0.5f;   //   0.1
static float pid_pitch_ki = 0.0f;
static float pid_pitch_kd = 0.1f;   // 0.16


void PID_Init(void) {
    pid_roll_output  = 0.0f;
    pid_pitch_output = 0.0f;
    pid_yaw_output   = 0.0f;
}


void PID_Update(void)
{
    // === 1. Скорости из IMU ===
    roll_rate  = gyro_roll_rate;
    pitch_rate = gyro_pitch_rate;
    yaw_rate = gyro_yaw_rate;

    // === 2. Стики → желаемые углы ===
    float roll_angle_set  = (rc_channels[0] - 1024) * 0.05f;
    float pitch_angle_set = (rc_channels[1] - 1024) * 0.05f;
    float yaw_angle_set    = (rc_channels[3] - 1024) * 0.5f;

    // === 3. Ошибка по углам ===
    float roll_angle_error  = roll_angle_set  - roll_angle;
    float pitch_angle_error = pitch_angle_set - pitch_angle;
   // float yaw_angle_error = yaw_angle_set - yaw_angle;

    // === 4. Внешний контур (ANGLE → RATE) ===
    float angle_kp = 3.0f;

    float roll_rate_set  = angle_kp * roll_angle_error;
    float pitch_rate_set = angle_kp * pitch_angle_error;
 //   float yaw_rate_set = angle_kp * yaw_angle_error;

    // === 5. Внутренний RATE PID ===

    // --- Roll ---
    float roll_error = roll_rate_set - roll_rate;
    float roll_deriv = -roll_rate;   // классический D по гире
    pid_roll_output = pid_roll_kp * roll_error + pid_roll_kd * roll_deriv;

    // --- Pitch ---
    float pitch_error = pitch_rate_set - pitch_rate;
    float pitch_deriv = - pitch_rate;
    pid_pitch_output = pid_pitch_kp * pitch_error + pid_pitch_kd * pitch_deriv;

    // --- Yaw ---
    float yaw_error = yaw_rate_set - yaw_rate;
    float yaw_deriv = -yaw_rate; // классический D по гире
    pid_yaw_output = pid_yaw_kp * yaw_error + pid_yaw_kd * yaw_deriv;
}




