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
  float comp = fabsf(OriPRT.Z);
  if (comp < TiltСomp) Throt /= TiltСomp;
  else Throt /= comp;
  if (Throt > 1.0f) Throt = 1.0f;
  
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
