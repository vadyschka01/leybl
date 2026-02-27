#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"

class Compass
{
public:
  Compass(const Record<Vector2>& RecordOF, const Record<Vector3>& RecordGyr, const Record<Quaternion>& RecordQua);

  float CourseSpeed = 2.0f;
  float MaxTimerGPS = 1.0f;

private:
  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  bool Active;

  float Timer = 0;

  

  float Past = 0.2f; // Время отставания между OF и GPS

  Average<Vector3> AveSpd;

  const Record<Vector2>& RecOF;
  const Record<Vector3>& RecGyr;
  const Record<Quaternion>& RecQua;

  struct { bool Ready; float Yaw; } Shift;

public:
  void UpdateAverageOF();

  void SetMagnet(const Vector3& Mag);
  void SetCourseGPS(const Vector2& SpeedGPS, float Time);

public:
  bool GetShift(float& Yaw);
};
