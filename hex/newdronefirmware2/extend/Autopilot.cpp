#pragma once
#include <math.h>

#include "tick.h"
#include "uart.h"
#include "joy.h"
#include "crsf.h"

#include "INS/geom/Vector.h"
#include "device/interfaces/IIMU.h"
#include "device/mag.h"
#include "MathFunc.h"
#include "Filters.h"
#include "Autopilot.h"

MainOrintation DroneOrintation;

MainGoal DroneGoal;
Vector3 GoalPoint;

MainInertial DroneInertial;

DroneStatus MainDrone;

AutoPoint DroneNullPoint = {false, false, 0.0, 0.0, 0.0f }; // Тут лежат нулевые lat, lon, absAlt
bool ExternalGPSOff = false;
bool USE_GPS = false;

int CurrentIndexMission = -1;
Vector3 MissionShiftData[3] = { {0.0f, 0.0f, 1.0f}, {0.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};

Vector2 PointLand = {1488.0f, 1488.0f}; // Потом удалить

AutoPoint DroneHomePoint = {false, false, 0.0, 0.0, 0.0f }; // Тут лежат x,y точки дома по лнс

ExchangePoint ExPoint = {false, 0.0f, {0.0f, 0.0f, 0.0f}}; // буферная точка, используется если была получена не точка взлета

NAV_SYS_MODE ProtoNavSys = NAV_SYS_MODE::gps;
SYS_MODE ProtoSysMode = SYS_MODE::hold;
STATUS_MODE ProtoStatusMode = STATUS_MODE::on_ground;

AutoPilotSettings APSettings;

JOPAD_TYPE JOPAD_Type = JOPAD_TYPE::CRSF; // Выбрать пульт заранее !!!!!!!!!!!!!!!!!!!!!! ТЕПЕРЬ ЛЕЖИТ ТУТ, ПОТОМУ ЧТО МНЕ ТОЖЕ НАДО ЗНАТЬ, КАКОЙ ПУЛЬТ!!!1111!!!!!!!!!
unsigned long TELEM_RECV_SIZE = 128;
unsigned char TELEM_BUFFER[128];
unsigned char MAX_CRSF_PAYLOAD_SIZE = 56;
unsigned long TimeLeftInPoint = 0; // Время, когда дрон попал в точку и больше не двигался находясь в режиме миссии
unsigned long TimeSendMsg = 0;

static void AutoSendCallback(void* data, unsigned long size, int priority)
{
  PriorityQueue_Push(data, size, priority);
}

static unsigned int TelemQueueSize = 4;
TelemQueueObject TelemQueue[] = {
  {MESSAGES_ID::SysInfo, SysInfo, false},
  {MESSAGES_ID::BatteryInfo, BatteryInfo, false},
  {MESSAGES_ID::GpsInfo, GpsInfo, false},
  {MESSAGES_ID::InertialInfo, InertialInfo, false}
};

static void AutoSend(void* data, unsigned long size)
{
  unsigned char* pData = (unsigned char*)data;
  unsigned long offset = 0;
  while (offset < size)
  {
      unsigned long bytesToCopy = size - offset;
      if (bytesToCopy > MAX_CRSF_PAYLOAD_SIZE)
      {
          bytesToCopy = MAX_CRSF_PAYLOAD_SIZE;
      }
      if (JOPAD_Type == JOPAD_TYPE::CRSF)
      {
        CRSF_SendFGT(&pData[offset], bytesToCopy);
        TimeSendMsg = TICK_GetCount();
      }
      else
      {
        unsigned long SendCount = bytesToCopy;
        void* pSend = CRSF_GetPacket(&pData[offset], &SendCount);
        UART2_Send(pSend, SendCount);
        TimeSendMsg = TICK_GetCount();
      }
      offset += bytesToCopy; 
  }
}

static void AutoSendInLoop()
{
  PriorityMessage msg;
  if (PriorityQueue_Pop(&msg)) AutoSend(msg.data,msg.size); // Если в очереди что-то лежит - берем из нее
  else if(!MainDrone.NeedSendRawCalibData) // иначе отправляем сообщение телеметрии (если не выполняется калибровка)
  {
    for (int i = 0; i < TelemQueueSize; i++)
    {
      if (!TelemQueue[i].Sended)
      {
        TelemQueue[i].Send();
        TelemQueue[i].Sended = true;
        return;
      }
    }
    TelemQueue[0].Send();
    TelemQueue[0].Sended = true;
    for (int i = 1; i < TelemQueueSize; i++) TelemQueue[i].Sended = false;
  }
}

static bool AutoRecvFunction()
{
  if (JOPAD_Type == JOPAD_TYPE::CRSF)
  {
    if (TELEM_RECV_SIZE > 0)
    {
      for (unsigned long a = 0; a < TELEM_RECV_SIZE; a++)
      {
        MainUpdate(TELEM_BUFFER[a]);
        TELEM_BUFFER[a] = 0;
      }
    }
    return false;
  }
  else
  {
    unsigned char buf[128];
    unsigned long s = UART2_Recv(buf, 1);
    bool done = false;
    JOYPAD_Data d;
    for (long a = 0; a < s; a++) done = Parse(d, &TELEM_BUFFER, &TELEM_RECV_SIZE, buf[a]);
    if (done)
    {
      for (unsigned long a2 = 0; a2 < TELEM_RECV_SIZE; a2++)
      {
        MainUpdate(TELEM_BUFFER[a2]);
        TELEM_BUFFER[a2] = 0;
      }
    }
    //unsigned long s = LPUART1_Recv(bytes, len);
    return false;
  }
}

float CalcLandSpeed(float CurrentRange)
{
  float spd = APSettings.HeightLandStop;
  if (CurrentRange < spd) spd = CurrentRange;
  spd = spd/APSettings.HeightLandStop;
  spd = (spd*spd) * APSettings.MaxSpeedLand;
  
  const float fast_land = 0.2f;
  if (CurrentRange < fast_land)
  {
    spd = (1.0f-(CurrentRange/fast_land)) * 2.0f;
  }
  
  if (spd < APSettings.MinSpeedLand) spd = APSettings.MinSpeedLand;
  
  return spd;
}

static bool CheeckGround()
{
  //return false;  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  if (MainDrone.StatusMode == STATUS_MODE::on_ground) return true;
  
  bool ground = false;
  
  static long TimeCheeckLand = 0;
  
  float ALT = DroneInertial.Pos.Z - DroneInertial.GroundShift;
  
  if(fabsf(DroneInertial.Acc.Z) < 0.05f && ALT < 0.20f)
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
    /*MainDrone.MainActive = false;
    MainDrone.engine = ENGINE_STATUS::disable;
    
    if (MainDrone.AutoPilotActive)
    {
      MainDrone.Mode = SYS_MODE::hold;
      MainDrone.AutoPilotActive = false;
    }*/
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
      // Ee нужно вернуть после посадки
      if (MainVel.GroundLimit != MainVel.GroundLimitSettings) MainVel.GroundLimit = MainVel.GroundLimitSettings;
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
        float alt = DroneInertial.Pos.Z;
        if (MainVel.GroundMode) alt -= DroneInertial.GroundShift;
        MainVel.GroundSpeed = CalcLandSpeed(alt);
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
  // Если нельзя использовать GPS или нам нужно начать снижаться (в режме посадки или возврата домой)
  if (!USE_GPS || (((MainDrone.Mode == SYS_MODE::land) || (MainDrone.Mode == SYS_MODE::go_home)) && (MainDrone.StatusMode == STATUS_MODE::landing)))
  {
      MainVel.GroundMode = true;
      MainDrone.Nav = NAV_SYS_MODE::inertial;
      if ((MainVel.GroundMode) && (DroneGoal.Position.Z > MainDrone.MaxHeightGroundMode)) DroneGoal.Position.Z = MainDrone.MaxHeightGroundMode;
      if (MainVel.GroundMode) MainVel.GroundHeight = DroneGoal.Position.Z;
  }
  else
  {
    MainDrone.Nav = NAV_SYS_MODE::gps;
    MainVel.GroundMode = false;
  }
  
  float alt;
  if (MainVel.GroundMode) alt = DroneInertial.Pos.Z - DroneInertial.GroundShift;
  else alt = DroneInertial.Pos.Z;
  
  if (Joypad.SWC >= 1900 && alt >= 0.5f)
  {
    MainVel.LockAllRegInt(false);
    MainAtt.LockAllRegInt(false);
  }
  else if (Joypad.SWC < 1900 || alt < 0.5f)
  {
    MainVel.LockAllRegInt(true);
    MainAtt.LockAllRegInt(true);
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
  
  if (MainDrone.Mode == SYS_MODE::stabilized || MainDrone.Mode == SYS_MODE::acro)
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
  MainDrone.PointBeginNeed = true;
  DroneGoal.New = true;
  DroneGoal.Yaw = true;
  MainDrone.Mode = SYS_MODE::land;
  CheeckStartCondition();
}

void AutoGoHome() //Функция осуществлят немедленный возврат БПЛА (если он летел) в точку LaunchPoint. Сначала достижение точки. Затем посадка.
{
  DroneGoal.Position = { (float)DroneHomePoint.X, (float)DroneHomePoint.Y, DroneInertial.Pos.Z };
  if (MainVel.GroundMode) DroneGoal.Position.Z -= DroneInertial.GroundShift;
  MainDrone.PointBeginNeed = true;
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
  DroneGoal.PrevPosition = DroneGoal.Position;
  DroneGoal.Position = {x,y,z};
      
  AutoSetSpeed(speed);
  if (MainDrone.StatusMode == STATUS_MODE::on_ground)
  {
    Vector2 XY = DroneGoal.Position - DroneInertial.Pos;
    float dist = XY.Length();
    
    if (dist > APSettings.RadReachPoint)
    {
      ExPoint.Position = DroneGoal.Position;
      DroneGoal.Position.X = DroneInertial.Pos.X;
      DroneGoal.Position.Y = DroneInertial.Pos.Y;
      ExPoint.New = true;
      ExPoint.Speed = speed;
    }
    DroneGoal.Speed = 2.0f;
  }
  DroneGoal.New = true;
  DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
  DroneGoal.Yaw = true;
  MainDrone.PointBeginNeed = true;
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
   MainInit(AutoSendCallback, AutoSend);
}

unsigned long AutoThread()
{
  AutoRecvFunction();
  
  unsigned long tick = TICK_GetCount();
  
  int timer = 10;
  if (JOPAD_Type == JOPAD_TYPE::CRSF) timer = 50;
  
  if ((tick - TimeSendMsg >= timer) && (tick - LastTimeRecvMsg < 3000))
  {
    AutoSendInLoop();
  }
  
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
    
    Vector3 GoalDist;
    if (MainVel.GroundMode)
    {
      Vector3 pos = DroneInertial.Pos;
      pos.Z -= DroneInertial.GroundShift;
      GoalDist = DroneGoal.Position - pos;
    }
    else GoalDist = DroneGoal.Position - DroneInertial.Pos;
    Vector2 GoalDist2D = GoalDist;
    if (!ProtoMissionInfo.f_start) CurrentIndexMission = -1;
    float dist = GoalDist.Length();
    float dist2D = GoalDist2D.Length();
    
    float altDist;
    if (MainVel.GroundMode) altDist = DroneGoal.Position.Z - (DroneInertial.Pos.Z - DroneInertial.GroundShift);
    else altDist = DroneGoal.Position.Z - DroneInertial.Pos.Z;
    
    bool InPoint = false;
    //if (MainVel.GroundMode && dist2D < APSettings.RadReachPoint) InPoint = true;
    //if (!MainVel.GroundMode && dist < APSettings.RadReachPoint) InPoint = true;
    
    if (dist < APSettings.RadReachPoint) InPoint = true;
    
    CheeckCondition(GoalDist, dist);

    if (MainDrone.AutoPilotActive)
    {
      // Сюда можно вставлять какие-либо управляющие воздействия по условиям
      if (ProtoMissionInfo.f_start && !ProtoMissionInfo.f_pause) //Автополет по миссии
      {
        if (ProtoMissionInfo.current_item != CurrentIndexMission)
        {
          if (!DroneNullPoint.update) // Если нулевой точки нет за нее принимается первая точка ПЗ
          {
            DroneNullPoint.X = ProtoMissionData.items[0].param5;
            DroneNullPoint.Y = ProtoMissionData.items[0].param6;
            DroneNullPoint.Z = 0.0f;
          }
          MissionItemData point = ProtoMissionData.items[ProtoMissionInfo.current_item];
          AutoGoToGlobal(point.param5, point.param6, point.param7, point.param2);
          CurrentIndexMission = ProtoMissionInfo.current_item;
          return 0;
        }
        if (!ExPoint.New && InPoint && CurrentIndexMission == ProtoMissionInfo.current_item)
        {
          if (ProtoMissionInfo.current_item + 1 <= ProtoMissionInfo.total_items - 1) ProtoMissionInfo.current_item += 1;
          else
          {
            if (ProtoMissionData.items[ProtoMissionInfo.current_item].item_command == COMMANDS_ID::GoHome) AutoGoHome();
            if (ProtoMissionData.items[ProtoMissionInfo.current_item].item_command == COMMANDS_ID::Land) AutoLand();
            ProtoMissionInfo.f_start = false;
            ProtoMissionInfo.f_pause = false;
            ProtoMissionInfo.current_item = ProtoMissionInfo.total_items;
            CurrentIndexMission = -1;
          }
          
        }
      }
      // Сначала взлет если была получена точка, но высоты мало и полученная точка не точка взлета
      if (ExPoint.New && altDist < APSettings.RadReachPoint)
      {
        ExPoint.New = false;
        AutoGoToLocal(ExPoint.Position.X, ExPoint.Position.Y, ExPoint.Position.Z, ExPoint.Speed);
      }
      
      bool LandNeed = false;
      
      // Автоматическая посадка в режиме land
      if ((MainDrone.Mode == SYS_MODE::land) && DroneGoal.Position.Z != APSettings.AltLand && InPoint) LandNeed = true;

      // Автоматическая посадка в режиме go home после достижения домашней точки
      if ((MainDrone.Mode == SYS_MODE::go_home) && DroneGoal.Position.Z != APSettings.AltLand && InPoint) LandNeed = true;
      
      // Автоматическая посадка
      if (LandNeed)
      {
        if (TimeLeftInPoint == 0) TimeLeftInPoint = TICK_GetCount();
        if (TICK_GetCount() - TimeLeftInPoint > 1500)
        {
          DroneGoal.Position.Z = APSettings.AltLand;
          MainVel.GroundLimit = APSettings.AltLand;
          MainVel.GroundHeight = APSettings.AltLand;
          DroneGoal.PRY.Z = DroneOrintation.PRY.Z;
          MainDrone.PointBeginNeed = true;
          DroneGoal.New = true;
          DroneGoal.Yaw = true;
          TimeLeftInPoint = 0;
        }
      }
    }
  }

  return 0;
}
