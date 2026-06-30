#pragma once

#include "../INS/geom/Vector.h"
#include "INS/cont/Velocity.h"
#include "INS/cont/Attitude.h"
#include "Filters.h"
#include "Protocol.h"
#include "MathFunc.h"
#include "sbus.h"
#include "crsf.h"
#include "PriorityQueue.h"

void AutoLand();
void AutoGoHome();
void AutoGoToGlobal(double lat, double lon, float abs_alt, float speed);
void AutoGoToLocal(float x, float y, float z, float speed);
void AutoSetSpeed(float speed);
void SaveNullPoint(double& lat, double& lon, float& absAlt);
void CheeckStartCondition();
float CalcLandSpeed(float CurrentRange);

EXEC_CODE_COMMAND CheeckStatusCommand(COMMANDS_ID comId);

struct AutoPilotSettings
{
        float MinSpeedLand = 0.5f; // Минимальная скорость снижения
        float MaxSpeedLand = 3.0f; // Максимальная скорсоть снижения
        float HeightLandStop = 5.0f; // Высота, до которой будет осуществляться снижение с максимальной скоростью
	float RadReachPoint = 3.0f; // Радиус достижения точки
	float AltAutoGoHome = 2.0f; // Высота возврата домой по умолчанию в метрах (в случае посадки в другом месте)
        float AltLand = -5.0f;
};

struct AutoPoint
{
        bool update = false;
	bool saved;
	double X, Y;
	float Z;
};

struct ExchangePoint
{
	bool New;
        float Speed;
	Vector3 Position;
};

struct DroneStatus
{
  bool MainActive = false; // Индикатор снятия с охраны
  bool AutoPilotActive = false; // Индикатор актив. автопилота
  bool ManualInputAllowed = false; // Индикатор возможности ручного управления (безопасного)
  bool AltChangeAllowed = false; // Индикатор возможности смены целевой высоты в режиме удержания позиции
  bool PointBeginNeed = true; // Индикатор необходимости указания начальной точки
  bool MissionExecute = false; // Инидикатор выполняется ли миссия
  
  bool MotorMoutionTestEnable = false;
  unsigned char MotorTestId = 0;
  unsigned long TimeMotorTestEnable = 0;
  
  unsigned char BatteryPer = 0;
  unsigned char BatteryCriticalPer = 10;
  bool NeedSendRawCalibData = false;
  SENSOR_ID Sensor = SENSOR_ID::ACC;
  unsigned long TimeLastSend = 0;
  unsigned long TimeStartSend = 0;
  bool NeedSaveNewEEPROMData = false;
  bool EEPROMSaved = false;
  bool NeedSaveNewFLASHData = false;
  bool FLASHSaved = false;
  
  bool EEPROMInit = false;
  bool BarInit = false;
  bool IMUInit = false;
  bool GPSInit = false;
  bool TOFInit = false;
  bool OFInit = false;
  bool CamFlowInit = false;
  bool ExternMagInit = false;
  
  float AvgEngineThrust = 0.0f;
  float MaxHeightGroundMode = 500.0f;
  
  ENGINE_STATUS engine = ENGINE_STATUS::disable;
  SYS_MODE Mode = SYS_MODE::stabilized;
  STATUS_MODE StatusMode = STATUS_MODE::on_ground;
  NAV_SYS_MODE Nav = NAV_SYS_MODE::inertial;
};

struct TelemQueueObject
{
  MESSAGES_ID id;
  void (*Send)();
  bool Sended;
};

extern bool ExternalGPSOff;
extern bool USE_GPS;
extern bool AutoActive;
extern bool PortDisconect;
extern JOPAD_TYPE JOPAD_Type;
extern unsigned long TELEM_RECV_SIZE;
extern unsigned char TELEM_BUFFER[128];
extern Vector2 PointLand;

extern AutoPoint DroneNullPoint;
extern JOYPAD_Data Joypad;
enum class AutoPortState { Free, Conected, Disconect };

extern AutoPortState AutoState;

void InitAuto();
unsigned long AutoThread();

struct MainGoal
{
  bool New;
  bool Stop;
  bool Yaw;
  float Speed;
  Vector3 PRY, PrevPosition, Position, Route;
};

extern MainGoal DroneGoal;
extern Vector3 GoalPoint;
extern DroneStatus MainDrone;

struct MainInertial
{
  Vector3 Acc, Spd, Pos;
  float GroundShift;
  unsigned long Time;
};

extern MainInertial DroneInertial;

struct MainOrintation
{
  float Range;
  Vector3 PRY, Iner, Position;
  float Power_UL, Power_UR, Power_DL, Power_DR;
};

extern MainOrintation DroneOrintation;
extern Velocity MainVel;
extern Attitude MainAtt;
