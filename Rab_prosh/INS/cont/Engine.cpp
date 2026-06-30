#include <math.h>

#include "Engine.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

static inline float Trimm(float val, float lim)
{
  if (val < -lim) return -lim;
  if (val > lim) return lim;
  return val;
}

void Engine::SetTiltThrot(float Tilt)
{
  Tilt = fabsf(Tilt);

  const float max_tilt = 60.0f;
  if (Tilt > max_tilt) Tilt = max_tilt;
  TiltСomp = cosf(Tilt * TO_RAD);
}

void Engine::SetPowerPRY(const Vector3& PowerPRY, float Throt)
{
  float comp = OriPRT.Z;
  
  if(comp>0)
  {
    if (comp < TiltСomp) Throt /= TiltСomp;
    else Throt /= comp;
    if (Throt > 1.0f) Throt = 1.0f;
  }
  
  float pitch = Trimm(PowerPRY.X, 1.0f);
  float roll = Trimm(PowerPRY.Y, 1.0f);
  float yaw = Trimm(PowerPRY.Z, 1.0f);
  
  ThrustThrot = Throt;
  ThrustPitch = pitch;
  ThrustRoll = roll;
  ThrustYaw = yaw;
}

void Engine::GetQuadX(float Thrust[4])
{
  float throt = sqrtf(ThrustThrot);
  
  float pitch = ThrustPitch;
  float roll = ThrustRoll;
  float yaw = ThrustYaw;
  
  float max = (fabsf(pitch) + fabsf(roll)) * 2.0f;

  if (max > 1.0f)
  {
    pitch /= max;
    roll /= max;
  
    max = 1.0f;
  }
  
  float gain = 1.0f + fabsf(throt - 0.5f) * 2.0f;
  
  pitch *= gain;
  roll *= gain;
  yaw *= gain;
  
  yaw *= (1.0f - max);
  
  Thrust[0] = Thrust[1] = Thrust[2] = Thrust[3] = 0;
  
  Thrust[QuadSchemeX[QuadSchemeX[0]]] += pitch; Thrust[QuadSchemeX[QuadSchemeX[1]]] += pitch;
  Thrust[QuadSchemeX[QuadSchemeX[2]]] -= pitch; Thrust[QuadSchemeX[QuadSchemeX[3]]] -= pitch;

  Thrust[QuadSchemeX[QuadSchemeX[0]]] += roll; Thrust[QuadSchemeX[QuadSchemeX[1]]] -= roll;
  Thrust[QuadSchemeX[QuadSchemeX[2]]] += roll; Thrust[QuadSchemeX[QuadSchemeX[3]]] -= roll;

  Thrust[QuadSchemeX[QuadSchemeX[0]]] -= yaw; Thrust[QuadSchemeX[QuadSchemeX[1]]] += yaw;
  Thrust[QuadSchemeX[QuadSchemeX[2]]] += yaw; Thrust[QuadSchemeX[QuadSchemeX[3]]] -= yaw;

  for (int a = 0; a < 4; a++)
  {
    float& thrust=Thrust[a];
    thrust += throt;
    if (thrust > 1.0f) thrust = 1.0f;
    if (thrust < 0.0f) thrust = 0.0f;
  }
}

void Engine::GetQuadX2(float Thrust[4])
{
    float throt = sqrtf(fmaxf(0.0f, ThrustThrot));
    
    // 1. ПРИОРИТЕТ: НАКЛОНЫ (Pitch/Roll)
    // Считаем чистую разницу мощностей для осей
    float m[4];
    m[0] = -ThrustPitch - ThrustRoll; // FR
    m[1] = -ThrustPitch + ThrustRoll; // FL
    m[2] =  ThrustPitch - ThrustRoll; // BR
    m[3] =  ThrustPitch + ThrustRoll; // BL

    
    // Нормализуем наклоны, если они в сумме > 1.0 (чтобы не потерять управление)
    float max_tilt = 0.0f;
    for(int i=0; i<4; i++) max_tilt = fmaxf(max_tilt, fabsf(m[i]));
    
    if (max_tilt > 1.0f) 
    {
        for(int i=0; i<4; i++) m[i] /= max_tilt;
    }

    // 2. ПРИОРИТЕТ: ТЯГА (Throttle)
    // Добавляем газ к наклонам
    for(int i=0; i<4; i++) m[i] += throt;

    // Смещение (Anti-Clip): если из-за наклонов мотор > 1.0, 
    // опускаем всю "пачку", жертвуя общим газом ради стабилизации
    float overshoot = 0.0f;
    for(int i=0; i<4; i++) if(m[i] > 1.0f) overshoot = fmaxf(overshoot, m[i] - 1.0f);
    if (overshoot > 0.0f) {
        for(int i=0; i<4; i++) m[i] -= overshoot;
    }

    // 3. ПРИОРИТЕТ: ВРАЩЕНИЕ (Yaw)
    // Подмешиваем Yaw только в то "окно", которое осталось после Наклонов и Газа
    for(int i=0; i<4; i++) 
    {
        float yaw_val = 0;
        if (i == 1 || i == 2) yaw_val = ThrustYaw;  // FL, BR (+)
        else yaw_val = -ThrustYaw;                  // FR, BL (-)
        
        // Ограничиваем Yaw, чтобы он не вытолкнул мотор за 0.0 или 1.0
        float limit_up = 1.0f - m[i];
        float limit_dn = m[i] - 0.0f;
        float final_yaw = fminf(fmaxf(yaw_val, -limit_dn), limit_up);
        
        m[i] += final_yaw;
    }

    // Запись в итоговый массив через маппинг
    for (int i = 0; i < 4; i++) 
    {
        Thrust[QuadSchemeX[i]] = fminf(fmaxf(m[i], 0.0f), 1.0f);
    }
}
