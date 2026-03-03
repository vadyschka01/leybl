#include <math.h>

#include "tick.h"
#include "uart.h"

#include "INS/geom/Vector.h"
#include "device/imu.h"
#include "device/mag.h"
#include "Filters.h"
#include "MathFunc.h"
#include "Autopilot.h"

MainOrintation DroneOrintation;

MainGoal DroneGoal;
Vector3 GoalPoint;

MainInertial DroneInertial;

DroneStatus MainDrone;

AutoPoint DroneNullPoint = { false, 0.0, 0.0, 0.0f }; // Тут лежат нулевые lat, lon, absAlt
bool ExternalGPSOff = false;
bool USE_GPS = false;

int StartIndexMission = 0;
int CurrentIndexMission = -1;
Vector3 MissionShiftData[3] = { {0.0f, 0.0f, 2.0f}, {0.0f, 5.0f, 0.0f}, {0.0f, -4.0f, 0.0f}};

Vector2 PointLand = {1488.0f, 1488.0f}; // Потом удалить

AutoPoint DroneHomePoint = { false, 0.0, 0.0, 0.0f }; // Тут лежат x,y точки дома по лнс

ExchangePoint ExPoint = {false, 0.0f, {0.0f, 0.0f, 0.0f}}; // буферная точка, используется если была получена не точка взлета

NAV_SYS_MODE ProtoNavSys = NAV_SYS_MODE::gps;
SYS_MODE ProtoSysMode = SYS_MODE::hold;
STATUS_MODE ProtoStatusMode = STATUS_MODE::on_ground;

AutoPilotSettings APSettings;

unsigned long TimeLeftInPoint = 0; // Время, когда дрон попал в точку и больше не двигался находясь в режиме миссии

static void AutoSendCallback(void* data, unsigned long size)
{
  UART2_Send(data, size);
  //LPUART1_Send(data, size);
}

static bool AutoRecvFunction()
{
  const unsigned long len = 128;

  char bytes[len];
  unsigned long s = UART2_Recv(bytes, len);
  //unsigned long s = LPUART1_Recv(bytes, len);
  
  for (unsigned long a = 0; a < s; a++) MainUpdate(bytes[a]);
  
  return false;
}

float CalcLandSpeed(float CurrentRange, float MinSpeed, float MaxSpeed,float HeightLandStop)
{
  float spd = HeightLandStop;
  if (CurrentRange < spd) spd = CurrentRange;
  spd = spd/HeightLandStop;
  spd = (spd*spd) * MaxSpeed;
  if (spd < MinSpeed) spd = MinSpeed;
  return spd;
}

static bool CheeckGround()
{
  //return false;
  if (MainDrone.StatusMode == STATUS_MODE::on_ground) return true;
  
  bool ground = false;
  
  static long TimeCheeckLand = 0;
  
  float ALT = DroneInertial.Pos.Z;
  if (MainVel.GroundMode) ALT -= DroneInertial.GroundShift;
  
  if(fabsf(DroneInertial.Acc.Z) < 0.05f && ALT < 0.10f)
  {
    if(TimeCheeckLand == 0) TimeCheeckLand = TICK_GetCount();
    if (TICK_GetCount() - TimeCheeckLand > 500)
    {
      ground = true;
      TimeCheeckLand = 0;
    }
  }
  else
  { 
    TimeCheeckLand = 0;
    ground = false;
  }
  
  //
  if(ground)
  {
    MainDrone.StatusMode = STATUS_MODE::on_ground;
    MainDrone.MainActive = false;
    MainDrone.engine = ENGINE_STATUS::disable;
    
    if (MainDrone.AutoPilotActive)
    {
      MainDrone.Mode = SYS_MODE::hold;
      MainDrone.AutoPilotActive = false;
    }
  }
  return ground;
}

static void CheeckCondition(Vector3 GoalDist, float dist) //Функция проверяет текущее состояние БПЛА
{
  if (MainDrone.AutoPilotActive)
  {
    if(MainDrone.Mode == SYS_MODE::hold || MainDrone.Mode == SYS_MODE::auto_mode || ((MainDrone.Mode == SYS_MODE::land) && (DroneGoal.Position.Z != APSettings.AltLand)) || ((MainDrone.Mode == SYS_MODE::go_home) && (DroneGoal.Position.Z != APSettings.AltLand)))
    {
      CheeckGround();
      if (MainDrone.MainActive)
      {
        float vector = DroneInertial.Pos.Z - DroneGoal.Position.Z;
        if(GoalDist.Z > APSettings.RadReachPoint)
        {
          if (vector < 0.0f) MainDrone.StatusMode = STATUS_MODE::taking_off;
          if (vector > 0.0f) MainDrone.StatusMode = STATUS_MODE::landing;
        }
        else MainDrone.StatusMode = STATUS_MODE::fly;
        
        if (MainDrone.Mode == SYS_MODE::auto_mode)
        {
          if (dist < APSettings.RadReachPoint)
          {
            if (TimeLeftInPoint == 0) TimeLeftInPoint = TICK_GetCount();
            if (TICK_GetCount() - TimeLeftInPoint > 1500)
            {
              if (MainDrone.AutoPilotActive) MainDrone.Mode = SYS_MODE::hold;
              TimeLeftInPoint = 0;
            }
          }
          else
          {
            TimeLeftInPoint = 0;
          }
        }
      }
    }
    if (((MainDrone.Mode == SYS_MODE::land) && (DroneGoal.Position.Z == APSettings.AltLand)) || ((MainDrone.Mode == SYS_MODE::go_home) && (DroneGoal.Position.Z == APSettings.AltLand)))
    {
      if (!CheeckGround())
      {
        MainDrone.StatusMode = STATUS_MODE::landing;
        DroneGoal.Speed = CalcLandSpeed(DroneInertial.Pos.Z);
      }
    }
  }
  else
  {
    CheeckGround();
    if(MainDrone.MainActive && DroneInertial.Pos.Z > 0.30f && fabsf(DroneInertial.Acc.Z) > 0.05f)
    {
      if(DroneInertial.Spd.Z < -0.1f) MainDrone.StatusMode = STATUS_MODE::landing;
      else if (DroneInertial.Spd.Z > 0.1f) MainDrone.StatusMode = STATUS_MODE::taking_off;
      else MainDrone.StatusMode = STATUS_MODE::fly;
    }
  }
  
  if (USE_GPS)
  {
    MainDrone.Nav = NAV_SYS_MODE::gps;
  }
  else
  {
    MainDrone.Nav = NAV_SYS_MODE::inertial;
  }
  
  if(!MainDrone.AutoPilotActive) MainDrone.PointBeginNeed = true;
}

EXEC_CODE_COMMAND CheeckStatusCommand(COMMANDS_ID comId) //Функция проверяет текущее состояние выполнения команды
{
  if ((comId == COMMANDS_ID::Land) || comId == COMMANDS_ID::GoHome)
  {
    if ((DroneInertial.Pos.Z < 0.15f) && (DroneGoal.Position.Z == APSettings.AltLand))
    {
      return EXEC_CODE_COMMAND::completed;
    }
    else
    {
      return EXEC_CODE_COMMAND::in_progress;
    }
  }
  else if (comId == COMMANDS_ID::ChangeNav)
  {
    if (ExternalGPSOff && (MainDrone.Nav == NAV_SYS_MODE::gps))
    {
      return EXEC_CODE_COMMAND::in_progress;
    }
    else if (ExternalGPSOff && (MainDrone.Nav == NAV_SYS_MODE::inertial))
    {
      return EXEC_CODE_COMMAND::completed;
    }
    else if (!ExternalGPSOff && (MainDrone.Nav == NAV_SYS_MODE::inertial))
    {
      return EXEC_CODE_COMMAND::in_progress;
    }
    else if (!ExternalGPSOff && (MainDrone.Nav == NAV_SYS_MODE::gps))
    {
      return EXEC_CODE_COMMAND::completed;
    }
  }
  else if (comId == COMMANDS_ID::SetCalibrationData)
  {
    switch(MainDrone.Sensor)
    {
      case SENSOR_ID::ACC:
      {
        if(!MainDrone.EEPROMSaved) return EXEC_CODE_COMMAND::in_progress;
        return EXEC_CODE_COMMAND::completed;
      }
      case SENSOR_ID::IMU_MAG:
      {
        if(!MainDrone.EEPROMSaved) return EXEC_CODE_COMMAND::in_progress;
        return EXEC_CODE_COMMAND::completed;
      }
      case SENSOR_ID::EXTERNAL_MAG:
      {
        if(!MainDrone.FLASHSaved) return EXEC_CODE_COMMAND::in_progress;
        return EXEC_CODE_COMMAND::completed;
      }
      case SENSOR_ID::LEVEL_HOR:
      {
        if(!MainDrone.FLASHSaved) return EXEC_CODE_COMMAND::in_progress;
        return EXEC_CODE_COMMAND::completed;
      }
    }
  }
  return EXEC_CODE_COMMAND::completed;
}

void CheeckStartCondition()
{
  bool ImuAccCalibNeed = CalibDataIMU.CalibAccNeed;
  bool MagCalibNeed = false;
  
  if (MainDrone.ExternMagInit) MagCalibNeed = ExternMagCalibData.CalibNeed;
  else MagCalibNeed = CalibDataIMU.CalibMagNeed;
  
  if (MainDrone.Mode == SYS_MODE::manual)
  {
    if (!MainDrone.MainActive)
    {
      MainDrone.AutoPilotActive = false;
      
      float thr_JOY = ((float)Joypad.Z)/2000.0f;
      
      if(MainDrone.ManualInputAllowed && (thr_JOY < 0.01f) && !ImuAccCalibNeed && !MagCalibNeed && MainDrone.IMUInit) 
      { 
        MainDrone.MainActive = true;
        MainDrone.engine = ENGINE_STATUS::enable;
      }
    }
  }
  else
  { 
    bool AutoPilotAllowed = MainDrone.GPSInit || ((MainDrone.OFInit && MainDrone.TOFInit));
    if (AutoPilotAllowed) MainDrone.AutoPilotActive = true;
    
    if (!MainDrone.MainActive && !ImuAccCalibNeed && !MagCalibNeed && MainDrone.IMUInit && AutoPilotAllowed)
    {
      MainDrone.MainActive = true;
      MainDrone.engine = ENGINE_STATUS::enable;
    }
  }
}

void AutoLand()
{
  PointLand = {DroneInertial.Pos.X, DroneInertial.Pos.Y};
  DroneGoal.Position = DroneInertial.Pos;
  if (MainVel.GroundMode) DroneGoal.Position.Z -= DroneInertial.GroundShift;
  DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
  DroneGoal.New = true;
  DroneGoal.Yaw = true;
  
  MainDrone.Mode = SYS_MODE::land;
  CheeckStartCondition();
}

void AutoGoHome() //Функция осуществлят немедленный возврат БПЛА (если он летел) в точку LaunchPoint. Сначала достижение точки. Затем посадка.
{
  DroneGoal.Position = { (float)DroneHomePoint.X, (float)DroneHomePoint.Y, DroneInertial.Pos.Z };
  DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
  DroneGoal.Yaw = true;
  DroneGoal.New = true;
  MainDrone.Mode = SYS_MODE::go_home;
  CheeckStartCondition();
}

void AutoSetSpeed(float speed)
{
  DroneGoal.Speed = speed;
}

void SaveHomePoint(float& x, float& y) // Функция запоминает точку взлета
{
  DroneHomePoint.X = x;
  DroneHomePoint.Y = y;
  DroneHomePoint.Z = 0.0f;
  DroneHomePoint.saved = true;
}

void SaveNullPoint(double& lat, double& lon, float& absAlt) // Функция запоминает нулевую точку привязки глобальных координат к локальным
{
  DroneNullPoint.X = lat;
  DroneNullPoint.Y = lon;
  DroneNullPoint.Z = absAlt;
  DroneNullPoint.saved = true;
}

void SetNextPoint(float x, float y, float z, float speed) // Функция осуществляет смену цели движения
{
  DroneGoal.Position = {x,y,z};
  AutoSetSpeed(speed);
  if (MainDrone.StatusMode == STATUS_MODE::on_ground)
  {
    Vector2 XY = DroneGoal.Position - DroneInertial.Pos;
    float dist = XY.Length();
    
    if (dist > APSettings.RadReachPoint)
    {
      DroneGoal.Position.X = DroneInertial.Pos.X;
      DroneGoal.Position.Y = DroneInertial.Pos.Y;
      ExPoint.New = true;
      ExPoint.Position = {x,y,z};
      ExPoint.Speed = speed;
    }
    DroneGoal.Speed = 2.0f;
  }
  DroneGoal.New = true;
  DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
  DroneGoal.Yaw = true;
}

void AutoGoToGlobal(double lat, double lon, float abs_alt, float speed) // Функция осуществляет смену цели перемещения дрона по глобальными координатам
{
  if (!DroneHomePoint.saved) SaveHomePoint(DroneInertial.Pos.X, DroneInertial.Pos.Y);
  float new_alt = abs_alt - DroneNullPoint.Z;
  if ((lat == 19991.1234) && (lon == 19991.1234))
  {
    DroneGoal.Position = DroneInertial.Pos;
    DroneGoal.Position.Z = new_alt;
    DroneGoal.New = true;
    DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
    DroneGoal.Yaw = true;
  }
  else
  {
    float x, y;
    GlobalToLocal(lat, lon, DroneNullPoint.X, DroneNullPoint.Y, x, y);
    SetNextPoint(y, x, new_alt, speed);
  }
  MainDrone.Mode = SYS_MODE::auto_mode;
  CheeckStartCondition();
  
}

void AutoGoToLocal(float x, float y, float z, float speed) // Функция осуществляет смену цели перемещения дрона по локальным координатам
{
  if (!DroneHomePoint.saved) SaveHomePoint(DroneInertial.Pos.X, DroneInertial.Pos.Y);
  if ((x == 19991.1234f) && (y == 19991.1234f))
  {
    DroneGoal.Position = DroneInertial.Pos;
    DroneGoal.Position.Z = z;
    DroneGoal.New = true;
    DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
    DroneGoal.Yaw = true;
  }
  else
  {
    SetNextPoint(x, y, z, speed);
  }
  MainDrone.Mode = SYS_MODE::auto_mode;
  CheeckStartCondition();
}

void InitAuto()
{
   MainInit(AutoSendCallback);
}

unsigned long AutoThread()
{
 
  AutoRecvFunction();
  
  if(MainDrone.NeedSendRawCalibData)
  {
    if (MainDrone.TimeLastSend == 0) MainDrone.TimeLastSend = TICK_GetCount();
    if (MainDrone.TimeStartSend == 0) MainDrone.TimeStartSend = TICK_GetCount();
    unsigned long TimeCurrent = TICK_GetCount();
    if (TimeCurrent - MainDrone.TimeLastSend > 100)
    {
      RawCalibData();
      MainDrone.TimeLastSend = 0;
    }
    if (TimeCurrent - MainDrone.TimeStartSend > 60000 * 5) // Если за 5 минут не откалибровали - перестать слать калибровочные данные
    {
      MainDrone.TimeStartSend = 0;
      MainDrone.TimeLastSend = 0;
      MainDrone.NeedSendRawCalibData = false;
    }
  }

  { // Управление автопилотом !!!
    
    // Авто полет по миссии
    if (Joypad.SWB > JOY_MID) 
    {
      MainDrone.MissionExecute = true;
      MainDrone.AutoPilotActive = true;
    }
    else if (Joypad.SWB < JOY_MID)
    { 
      MainDrone.MissionExecute = false;
      CurrentIndexMission = -1;
      StartIndexMission = 0;
    }
    Vector3 GoalDist;
    if (MainVel.GroundMode)
    {
      Vector3 pos = DroneInertial.Pos;
      pos.Z -= DroneInertial.GroundShift;
      GoalDist = DroneGoal.Position - pos;
    }
    else GoalDist = DroneGoal.Position - DroneInertial.Pos;
    Vector2 GoalDist2D = GoalDist;
    
    float dist = GoalDist.Length();
    float dist2D = GoalDist2D.Length();
    
    float altDist;
    if (MainVel.GroundMode) altDist = DroneGoal.Position.Z - (DroneInertial.Pos.Z - DroneInertial.GroundShift);
    else altDist = DroneGoal.Position.Z - DroneInertial.Pos.Z;
    
    bool InPoint = dist < APSettings.RadReachPoint ? true : false;

    CheeckCondition(GoalDist, dist);

    if (MainDrone.AutoPilotActive)
    {
      // Сюда можно вставлять какие-либо управляющие воздействия по условиям
      
      if (MainDrone.MissionExecute) //Автополет по миссии
      {
        if (CurrentIndexMission != StartIndexMission)
        {
          Vector3 Shift = MissionShiftData[StartIndexMission];
          Vector3 goal;
          
          if (MainVel.GroundMode) goal = { DroneInertial.Pos.X + Shift.X, DroneInertial.Pos.Y + Shift.Y, (DroneInertial.Pos.Z - DroneInertial.GroundShift) + Shift.Z };
          else goal = { DroneInertial.Pos.X + Shift.X, DroneInertial.Pos.Y + Shift.Y, DroneInertial.Pos.Z + Shift.Z };
          
          float spd = 1.0f;
          
          if (CurrentIndexMission == -1) spd = 2.0f;
          
          AutoGoToLocal(goal.X, goal.Y, goal.Z, spd);
          CurrentIndexMission = StartIndexMission;
          return 0;
        }
        if (InPoint && CurrentIndexMission == StartIndexMission)
        {
          if (StartIndexMission != 2) StartIndexMission += 1;
          else if (MainDrone.Mode != SYS_MODE::land)
          {
            AutoLand();
          }
        }
      }
      else
      {
        // Сначала взлет если была получена точка, но высоты мало и полученная точка не точка взлета
        if (ExPoint.New && altDist < APSettings.RadReachPoint)
        {
          ExPoint.New = false;
          AutoGoToLocal(ExPoint.Position.X, ExPoint.Position.Y, ExPoint.Position.Z, ExPoint.Speed);
        }
      }
      
      bool LandNeed = false;
      
      // Автоматическая посадка в режиме land
      if ((MainDrone.Mode == SYS_MODE::land) && DroneGoal.Position.Z != APSettings.AltLand && InPoint) LandNeed = true;

      // Автоматическая посадка в режиме go home после достижения домашней точки
      if ((MainDrone.Mode == SYS_MODE::go_home) && DroneGoal.Position.Z != APSettings.AltLand && (DroneGoal.Position.X == (float)DroneHomePoint.X) && InPoint) LandNeed = true;
      
      // Автоматическая посадка
      if (LandNeed)
      {
        if (TimeLeftInPoint == 0) TimeLeftInPoint = TICK_GetCount();
        if (TICK_GetCount() - TimeLeftInPoint > 1500)
        {
          DroneGoal.Position.Z = APSettings.AltLand;
          DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
          DroneGoal.New = true;
          DroneGoal.Yaw = true;
          TimeLeftInPoint = 0;
        }
      }
    }
  }

  return 0;
}
