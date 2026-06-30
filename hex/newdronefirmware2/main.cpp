#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32g4xx.h"

#include "gpio.h"
#include "led.h"
#include "tim.h"
#include "tick.h"
#include "i2c.h"
#include "uart.h"
#include "spi.h"
#include "pwm.h"
#include "flash.h"
#include "adc.h"
#include "dshot2.h"


#include "tof.h"
#include "flow.h"
#include "gps.h"
#include "camflow.h"

#include "device/interfaces/IBar.h"
#include "device/interfaces/IIMU.h"
#include "device/interfaces/IMag.h"
#include "device/interfaces/IEEPROM.h"

#include "sbus.h"
#include "crsf.h"

#include "INS/IRS.h"

#include "INS/device/GPS.h"
#include "INS/device/Barometer.h"
#include "INS/device/Range.h"
#include "INS/device/OpticalFlow.h"
#include "INS/device/CameraFlow.h"
#include "INS/device/Compass.h"

#include "INS/cont/Attitude.h"
#include "INS/cont/Velocity.h"
#include "INS/cont/Engine.h"

#include "Autopilot.h"
#include "Settings.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

constexpr long TIM_PRIORITY=2;

static Vector3 ANG_TEST;

IMU_Data DataIMU;
BAR_Data DataBAR;
MAG_Data DataMAG;

struct GPS_Data
{
  bool Init;
  bool Valid;
  float X, Y, Z;
  float AbsAlt;
  Point Zero;
  
  bool Update;
};

struct TOF_Data
{
  bool Valid;
  float Alpha;
  float Range;
  float Strength;
  
  bool Update;
};

struct FLOW_Data
{
  bool Valid;
  float Quality;
  Vector2 OF;
  
  bool Update;
};

struct CamFlow_Data
{
  bool Valid;
  Vector2 XY;
  float Yaw;
  unsigned long Time;
  unsigned long Period;
  
  Vector2 z;
  
  bool Update;
};

FLOW_Data DataFLOW;
TOF_Data DataTOF;
GPS_Data DataGPS;
CamFlow_Data DataCamFlow;

IRS MainIRS;

GPS MainGPS(MainIRS.RecordPosit, MainIRS.RecordSpeed);
Barometer MainBar(MainIRS.RecordPosit, MainIRS.RecordSpeed, MainIRS.RecordQuat);
Range MainLen(MainIRS.RecordSpeed, MainIRS.RecordQuat);
OpticalFlow MainOF(MainIRS.RecordGyro, MainIRS.RecordSpeed, MainIRS.RecordPosit, MainIRS.RecordQuat);

CameraFlow MainCam(MainIRS.RecordSpeed, MainIRS.RecordPosit, MainIRS.RecordQuat);

//Compass MainCompas(MainOF.RecordOF, MainIRS.RecordGyro, MainIRS.RecordQuat);
ControllerLED LedControl;
RPIController rpiControl;

Attitude MainAtt;
Velocity MainVel(MainIRS.OriQuat);
Engine MainEng(MainIRS.OriPRT);

IBar* BarObj;
IIMU* IMUObj;
IMag* MagObj;
IEEPROM* EEPROMObj;

JOYPAD_Data Joypad;
bool MainActive=false;
float BAT_VAL;

unsigned long TimeManualActive = 0;

ButterworthCascadeFilter ButterBaroFilt(IRS::Freq);

enum class MOTOR_TYPE {NO = 0, PWM = 1, DSHOT = 2};

MOTOR_TYPE MotorType = MOTOR_TYPE::DSHOT;

void SetMotorPower(unsigned long Count, unsigned short Pow[6], unsigned short Min, unsigned short Max)
{
  if(Max>999) Max=999;
  if(Min>Max) Min=Max;
  
  switch(MotorType)
  {
    case MOTOR_TYPE::PWM:
    {
      if(Count==4) 
      {
        unsigned short m[4]={Pow[0], Pow[1], Pow[2], Pow[3]};
        PWM_SetQuad(m, 1000+Min, 1000+Max);
      }
      if(Count==6) 
      {
        unsigned short m[6]={Pow[0], Pow[1], Pow[2], Pow[3], Pow[4], Pow[5]};
        PWM_SetHexa(m, 1000+Min, 1000+Max);
      }
      return;
    }
    case MOTOR_TYPE::DSHOT:
    {
      unsigned short cmd[6]={Pow[2], Pow[5], Pow[3], Pow[4], 0, 0}; 
      
      if(Count==6) { cmd[4]=Pow[0];  cmd[5]=Pow[1]; }
      
      for(int a=0; a<Count; a++) 
      {
        if(cmd[a]<Min) cmd[a]=Min;
        else if(cmd[a]>Max) cmd[a]=Max;
        if(cmd[a]>0) cmd[a]=48+cmd[a]*2;
      }
      
      DSHOT_SetCommand(cmd, true);
      return;
    }
  };
}

void SetMotorAll(unsigned long Count, unsigned short Pow)
{
  if(Pow>999) Pow=999;
  
  switch(MotorType)
  {
    case MOTOR_TYPE::PWM: 
    {
      Pow = Pow>0 ? (1000+Pow) : 900;
      PWM_SetAll(Pow); 
      return;
    }
    case MOTOR_TYPE::DSHOT:
    {
      if (Pow>0) Pow=Pow*2+48;
      unsigned short cmd[6]={Pow, Pow, Pow, Pow, Pow, Pow};
      DSHOT_SetCommand(cmd, true);
      return;
    }
  }
}

float GetCurrentTime()
{
  return ((float)TICK_GetCount())/1000.0f;
}

void UpdateRange(float range, bool valid) // Приоритет 4
{
  //return;
  
  if (!valid)
  {
    //MainIRS.SetGroundHeight(MainLen.MaxHeight, 0.0005f);
    
    MainLen.SetRange(nullptr);
    
    return;
  }
  
  float len = MainLen.SetRange(&range);
 
  if (len>0) 
  {
    float alpha=0.1f;
    
    if(len<MainLen.MaxHeight) alpha = 1.0f - len/MainLen.MaxHeight;

    if(alpha<0.1f) alpha = 0.1f;
    
    MainIRS.SetGroundHeight(len, alpha*alpha);
  }
  else 
  {
    MainIRS.SetGroundHeight(MainLen.MaxHeight, 0.0005f);
  }
  
  //return; (раскоменьти чтобы отключить коррекцию)
  // Коррекция скорости !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  float spd;
  if (MainLen.GetShift(spd))
  {
    MainIRS.UpdatePositionSpeed({0.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, spd });
  }
}

void UpdateFlow(Vector2& of, float range, bool valid, float alpha) // Приоритет 3
{
  return;
  
  if(!valid || USE_GPS)
  {
    MainOF.SetMove(0, 0, 0);
    return;
  }
  
  MainOF.SetMove(&of, range, alpha);
  
  //MainCompas.UpdateAverageOF();
  
  Vector2 pos, spd;
  if (MainOF.GetShift(pos, spd))
  {
    //if (DataGPS.Valid || !ExternalGPSOff) pos = {0.0f, 0.0f};
    MainIRS.UpdatePositionSpeed(pos, spd);
  }
}

void UpdateCam(Vector2& cam, float range, bool valid, unsigned long time)
{
  //return;
  
  if(!valid || USE_GPS)
  {
    MainCam.SetMove(0, 0, time);
    return;
  }
  
  MainCam.SetMove(&cam, range, time);
  
  Vector2 pos, spd;
  if (MainCam.GetShift(pos, spd))
  {
    //if (DataGPS.Valid || !ExternalGPSOff) pos = {0.0f, 0.0f};
    MainIRS.UpdatePositionSpeed(pos, spd);
  }
}

void UpdateBar(float pressure, float temp, bool Valid)  // Приоритет 2
{  
  //return;
  
  if(!Valid) 
  {
    MainBar.SetPressure(0, 0);
    return;
  }
  
  
  pressure = ButterBaroFilt.process(pressure);
  pressure = BaroFilt.update2(pressure);
  
  MainBar.SetPressure(&pressure, &temp);
  
  float pos, spd;
  if (MainBar.GetShift(pos, spd))
  {
    MainIRS.UpdatePositionSpeed({ 0.0f, 0.0f, pos }, { 0.0f, 0.0f, spd });
  }
}

void UpdateGPS(Vector3& local, float time, bool Valid)  // Приоритет 1
{
  //return;
  if((!Valid) || ExternalGPSOff)
  {
    MainGPS.SetPosition(0, time);
    //ShiftGPS.Active = false; // инидикатор ЖПС фе для протокола (можно на что-то другое заменить)
    return;
  }
  
  MainGPS.SetPosition(&local, time);
  MainBar.UpdateZeroHeight(local.Z, 0.002f);
  
  Vector3 pos, spd;
  if (MainGPS.GetShift(pos, spd))
  {
    MainIRS.UpdatePositionSpeed(pos, spd);
  }
  
  //Vector3 crs;
  //MainGPS.GetPositionSpeed(nullptr, &crs);
  //MainCompas.SetCourseGPS(crs, time);
  //ShiftGPS.Active = true; // индикатор ЖПС ок для протокола (можно на что-то другое заменить)
}

void AutoControl(Vector3 gyro)
{
  Vector3 pos, spd, acc; float gnd;
  MainIRS.GetInertial(&pos, &spd, &acc, &gnd);
  
  bool rev;
  Vector3 pry, ang;
  MainIRS.GetPitchRollYaw(pry, rev);
  MainAtt.UpdateRegPRY(MainIRS.OriQuat, gyro);
  
  float thr_JOY = ((float)Joypad.Z)/2000.0f;
  
  float fly;
  
  static float joy_yaw;
  
  //return; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  if (MainDrone.engine == ENGINE_STATUS::disable)
  {
    CalibDataIMU.AllowedCalib = true;
    MainIRS.Inertial.Pos = 0;
    MainIRS.Inertial.Spd = 0;
    DataCamFlow.z = 0;
    DroneNullPoint.saved = false;
    if (!MainDrone.MotorMoutionTestEnable) SetMotorAll(6, 0);
    else
    {
      unsigned short Power[6]={0,0,0,0,0,0};
      Power[MainDrone.MotorTestId] = 50;
      SetMotorPower(6, Power, 0, 100);
      
      if (MainDrone.TimeMotorTestEnable == 0) MainDrone.TimeMotorTestEnable = TICK_GetCount();
      if (TICK_GetCount() - MainDrone.TimeMotorTestEnable > 3000 && MainDrone.StatusMode == STATUS_MODE::on_ground)
      {
        MainDrone.TimeMotorTestEnable = 0;
        MainDrone.MotorMoutionTestEnable = false;
      }
    }
    MainDrone.MainActive = false;
    joy_yaw=pry.Z;
    return;
  }
  
  CalibDataIMU.AllowedCalib = false;
  
  if (((thr_JOY < 0.01f) || !MainDrone.MainActive ) && MainDrone.ManualInputAllowed)
  {
    SetMotorAll(6, 100);
    joy_yaw=pry.Z;
    if (!MainDrone.AutoPilotActive)
    {
      if (TimeManualActive == 0) TimeManualActive = TICK_GetCount();
      if (TICK_GetCount() - TimeManualActive > 3000 && MainDrone.StatusMode == STATUS_MODE::on_ground)
      {
        MainDrone.MainActive = false;
        MainDrone.engine = ENGINE_STATUS::disable;
        TimeManualActive = 0;
      }
    }
    return;
  }
  
  TimeManualActive = 0;
  
  joy_yaw+=(float)(Joypad.W - JOYPAD_MID)/440.0f;
  
  if (!MainDrone.AutoPilotActive)
  {
    MainDrone.AltChangeAllowed = false;
    
    Vector3 pry = { (float)(JOYPAD_MID - Joypad.Y)/22.0f, (float)(Joypad.X - JOYPAD_MID)/22.0f, joy_yaw};
    pry /= 57.2957795f;
    Quaternion q_pry = Quaternion::CreatePitchRollYaw(pry);
    
    MainAtt.SetThrot(thr_JOY);
    MainAtt.SetAnglePRY(q_pry);
    MainAtt.SetMode(false);
    fly = 0.0f;
    
    // Тест зависания;
    {
      DroneGoal.Position = pos;
      if (MainVel.GroundMode) DroneGoal.Position.Z -= MainIRS.GroundShift;
      DroneGoal.Speed = 3.0f;
      DroneGoal.New = true;
      DroneGoal.Yaw = true;
      DroneGoal.PRY.Z = pry.Z;
    }

    MainVel.GroundHeight = MainIRS.Inertial.Pos.Z - MainIRS.GroundShift;
  }
  else
  {
    float newY = (float)(Joypad.Y - JOYPAD_MID)/100000.0f;
    float newX = (float)(Joypad.X - JOYPAD_MID)/100000.0f;
    float newZ = 0.0f;
    
    //if (fabs(Joypad.Z - JOYPAD_MID) < 450) MainDrone.AltChangeAllowed = true;
    //if(fabs(Joypad.Z - JOYPAD_MID) > 500) newZ = (float)(Joypad.Z - JOYPAD_MID)/100000.0f;
    if (((newY != 0.0f) || (newX != 0.0f) || ((newZ != 0.0f) && MainDrone.AltChangeAllowed)) && MainDrone.ManualInputAllowed)
    {
      if (MainDrone.Mode != SYS_MODE::hold)
      {
        MainDrone.Mode = SYS_MODE::hold;
        if (ProtoMissionInfo.f_start)
        {
          ProtoMissionInfo.f_start = false;
          ProtoMissionInfo.f_pause = true;
        }
        DroneGoal.Position = pos;
      }
      
      Vector3 xyz = { newX, newY, newZ };
      
      xyz=MainIRS.OriQuat.RotateAroundZ(xyz);
      
      DroneGoal.Position += xyz;
      DroneGoal.New = true;
    }
    
    MainVel.SetManualYaw(DroneGoal.Yaw, &joy_yaw);
    if(DroneGoal.New)
    {
      Vector3* PointBegin = nullptr;
      if (MainDrone.PointBeginNeed)
      {
        if (ProtoMissionInfo.f_start)
        {
          PointBegin = &DroneGoal.PrevPosition;
          if (MainVel.GroundMode) PointBegin->Z -= DroneInertial.GroundShift;
        }
        else
        {
          PointBegin = &DroneInertial.Pos;
          if (MainVel.GroundMode) PointBegin->Z -= DroneInertial.GroundShift;
        }
        MainDrone.PointBeginNeed = false;
      }
      
      MainVel.SetSpeed(&DroneGoal.Speed, nullptr);
      
      MainVel.SetDistance(PointBegin, DroneGoal.Position);
      DroneGoal.New = false; // Не задавать если уже получена !!!!!!!!!!!!!!!!!
    }
    else MainVel.SetSpeed(&DroneGoal.Speed, nullptr);
    
    bool con = false;
    
    float throt;
    Quaternion q_pry;
    
    static float TEST_T;
    static Quaternion TEST_Q;
    
    //Vector3 corr = { 10.0f*TO_RAD, 10.0f*TO_RAD, 0.0f };
    //q_pry = Quaternion::CreatePitchRollYaw(corr) * q_pry;
    
    MainVel.GetThrotAnglePRY(throt, q_pry);
    
    TEST_T=throt;
    TEST_Q=q_pry;
    
    MainAtt.SetThrot(throt);
    MainAtt.SetAnglePRY(q_pry);
    MainAtt.SetMode(false);
    fly = 0.25f;
    
    if (DroneGoal.Stop)
    {
      DroneGoal.Stop = false;
      MainVel.Stop(nullptr);
    }
    Vector3 pos, spd, acc; float gnd;
    MainIRS.GetInertial(&pos, &spd, &acc, &gnd);
    MainVel.UpdateMoveSpeed(pos, spd, acc, gnd);
  }
  
  float throt;
  Vector3 pow;
  MainAtt.GetPowerPRY(&throt, &pow);
  MainEng.SetPowerPRY(pow, throt + fly);
  
  /*float thrust[4];
  MainEng.GetQuadX(thrust);
  float motor[4] = { thrust[MainEng.QuadSchemeX[0]], thrust[MainEng.QuadSchemeX[1]], thrust[MainEng.QuadSchemeX[2]], thrust[MainEng.QuadSchemeX[3]] };
  short power[4] = { (short)(motor[0]*1000.0f), (short)(motor[1]*1000.0f), (short)(motor[2]*1000.0f), (short)(motor[3]*1000.0f) };
  
  
  //power[0]=power[1]=power[2]=power[3]=0;
  
  static short testPower[4];
  testPower[0] = power[0];
  testPower[1] = power[1];
  testPower[2] = power[2];
  testPower[3] = power[3];
  
  PWM_SetQuad(power, 1100, 2000);
  //PWM_SetQuad(power, 900, 900);*/
  
  float thrust[6];
  MainEng.GetHex(thrust);
  
  unsigned short power[6] = { thrust[0]*1000.0f, thrust[1]*1000.0f, thrust[2]*1000.0f, thrust[3]*1000.0f, thrust[4]*1000.0f, thrust[5]*1000.0f };
  
  
  //power[0]=power[1]=power[2]=power[3]=power[4]=power[5]=Joypad.Z/2;
  
  static unsigned short testPower[6];
  testPower[0] = power[0];
  testPower[1] = power[1];
  testPower[2] = power[2];
  testPower[3] = power[3];
  testPower[4] = power[4];
  testPower[5] = power[5];
  
  SetMotorPower(6, power, 200, 1000);
}

float GPSNoiz = 0;
float GPSNoizPast = 0.5f;
unsigned long GPSNoizTime = 5000;
float GPSNoizDelta = 10.0f;

struct StructNoiseGPS
{
  static constexpr float Freq = 100;
  static constexpr float Past = 0.5f; // Максимальная запись прошлых данных
  static constexpr unsigned long Count = Freq * Past + 1;
  float Buffer[Count];
  Record<float> Rec;
} NoiseGPS;

void ReadStateINS()
{
  bool rev;
  static Vector3 pry;
  MainIRS.GetPitchRollYaw(pry, rev);
  DroneOrintation.PRY = pry;
  
  static Vector3 iner_acc, iner_spd, iner_pos;
  MainIRS.GetInertial(&iner_pos, &iner_spd, &iner_acc, nullptr);
  
  static GPS_BaseInfo gpsInfo;
  bool gps_valid = DataGPS.Valid;
  GPS_GetBaseInfo(gpsInfo);
  
  NoiseGPS.Rec.Add(gpsInfo.noise);
  static float TEST_GPS_PAST;
  TEST_GPS_PAST = NoiseGPS.Rec.Past(GPSNoizPast);
  GPSNoiz = gpsInfo.noise;//fabsf(gpsInfo.noise-NoiseGPS.Rec.Past(GPSNoizPast));
  
  //DroneOrintation.Iner = acc;
  DroneInertial.Spd = iner_spd;
  DroneInertial.Acc = iner_acc;
  DroneInertial.Pos = iner_pos;
  DroneInertial.GroundShift = MainIRS.GroundShift;
  
  float current_A = 0.0f;
  //
  { //SYS INFO
    ProtoDataSysInfo sys;
    sys.engineCount = sizeof(MainEng.QuadSchemeX); //
    sys.pressureBaro = (MainBar.Bar / 10.0f);
    sys.tempBaro = MainBar.Temp * 100.0f;
    //
    sys.engineStatus = MainDrone.engine;
    
    float thrust[sizeof(MainEng.QuadSchemeX)];
    MainEng.GetQuadX(thrust);
    
    float AvgEngineThrust = 0.0f;
    for (int i = 0; i < sys.engineCount; i++)
    {
      EnginePowerInfo eng;
      
      eng.current = (thrust[MainEng.QuadSchemeX[i]] * thrust[MainEng.QuadSchemeX[i]] * 20.0f) * 100.0f;
      eng.engineSpeed = thrust[MainEng.QuadSchemeX[i]] * 1000;
      eng.power = eng.engineSpeed / 10;
      eng.temp = 65.0f * 100.0f;
      eng.voltage = BAT_VAL * 100.0f;
      sys.enginePower[i] = eng;
      
      AvgEngineThrust += thrust[MainEng.QuadSchemeX[i]];
      current_A += eng.current;
    }
    ProtoDataWriteReadSysInfo(sys, false);
    
    if (AvgEngineThrust > 0.0f) MainDrone.AvgEngineThrust = AvgEngineThrust / sizeof(MainEng.QuadSchemeX);
    else MainDrone.AvgEngineThrust = 0.0f;
  }

  { //Battery
    // Нужен измеритель тока и температуры АКБ, чтобы сделать красивее.
    static float onePerVal = 100.0f/(BAT_MAX_VAL-BAT_MIN_VAL);

    ProtoDataBattery bat;
    bat.amperage = (0.5f + current_A) * 100.0f;
    bat.perCharge = 0;
    if (BAT_VAL > 1e-12f)
    {
      bat.voltage = BAT_VAL * 100.0f;
      if (BAT_VAL < BAT_MIN_VAL) bat.perCharge = 0;
      else bat.perCharge = onePerVal*(BAT_VAL - BAT_MIN_VAL);
      if (bat.perCharge > 100) bat.perCharge = 100;
    }
    MainDrone.BatteryPer = bat.perCharge;
    bat.temp = 36.6f * 100.0f;
    bat.totalPower = (bat.amperage * bat.voltage) * 1000.0f;
    bat.timeRemaining = (unsigned long)((BAT_CONS/(bat.amperage * 1000.0f)) * 0.7f * 3600);
    //-------------------------------------------------------------------------------
    bat.attackStatus = 5;
    bat.detectTime = 0;
    ProtoDataWriteReadBattery(bat, false);
  }

  { // GYR
    ProtoDataGyroInfo gyr;
    gyr.pitchGyroVel = DataMAG.RawX * 100.0f; //
    gyr.rollGyroVel = DataMAG.RawY * 100.0f; //
    gyr.yawGyroVel = DataMAG.RawZ * 100.0f; //
    ProtoDataWriteReadGyroInfo(gyr, false);
  }

  { // ACC
    ProtoDataAccelInfo acc;
    acc.aX = DroneInertial.Acc.X * 100.0f;//DataIMU.RawGyr.X;
    acc.aY = DroneInertial.Acc.Y * 100.0f;//DataIMU.RawGyr.Y;
    acc.aZ = DroneInertial.Acc.Z * 100.0f;//DataIMU.RawGyr.Z;
    acc.pitchAccelVel = DataIMU.RawAcc.X * 100.0f; //
    acc.rollAccelVel = DataIMU.RawAcc.Y * 100.0f; //
    acc.yawAccelVel = DataIMU.RawAcc.Z * 100.0f; //
    acc.tempAccel = 34.5f * 10.0f; //
    ProtoDataWriteReadAccelInfo(acc, false);
  }
  
  { // GPS
    ProtoDataGpsInfo gps;
    float null_lat, null_lon, null_abs_alt;
    float lat, lon, alt, r_alt, hdop, vdop, pdop, noise;
    unsigned char satVis, satUsed;
    if (gps_valid)
    {
      lat = gpsInfo.lat;
      lon = gpsInfo.lon;
      alt = gpsInfo.absAlt;
      r_alt = DataGPS.Z;
      hdop = gpsInfo.hdop;
      vdop = gpsInfo.vdop;
      pdop = gpsInfo.pdop;
      satVis = gpsInfo.satVisible;
      satUsed = gpsInfo.satUsed;
      noise = gpsInfo.noise;
    }
    else
    {
      lat = 0.0f;
      lon = 0.0f;
      alt = 0.0f;
      r_alt = 0.0f;
      hdop = 99.0f;
      pdop = 99.0f;
      vdop = 99.0f;
      satVis = 0;
      satUsed = 0;
      noise = 0;
    }
    gps.lat = lat * pow(10,7);
    gps.lon = lon * pow(10,7);
    gps.absAlt = alt;
    gps.null_lat = DroneNullPoint.X  * pow(10,7);
    gps.null_lon = DroneNullPoint.Y  * pow(10,7);
    gps.null_abs_alt = DroneNullPoint.Z;
    //gps.realAlt = r_alt;
    gps.realAlt = (DroneInertial.Pos.Z - DroneInertial.GroundShift);
    gps.hdop = hdop * 10.0f;
    gps.vdop = vdop * 10.0f;
    gps.pdop = pdop * 10.0f;
    gps.jamming = gpsInfo.jamming * 10.0f; // jamming indicator ublox.
    gps.noise = noise * 10.0f;
    gps.satVisible = satVis;
    gps.satUsed = satUsed;
    gps.speed = gpsInfo.speed * 100.0f;
    gps.fixType = gpsInfo.fixType;
    gps.timeUTC = gpsInfo.timeUTC * 1000;
    ProtoDataWriteReadGpsInfo(gps, false);
  }

  {// INNER
    ProtoDataInertialInfo inner;
    inner.x = DroneInertial.Pos.X;
    inner.y = DroneInertial.Pos.Y;
    inner.z = DroneInertial.Pos.Z;
    inner.baroAlt = MainBar.LastPos;
    inner.pitch = DroneOrintation.PRY.X * 100.0f;
    inner.roll = DroneOrintation.PRY.Y * 100.0f;
    inner.yaw = DroneOrintation.PRY.Z * 100.0f;
    inner.headingDeg = DroneOrintation.PRY.Z * 100.0f;
    //inner.speed = sqrtf(DroneInertial.Spd.X * DroneInertial.Spd.X + DroneInertial.Spd.Y * DroneInertial.Spd.Y);
    
    //inner.speed = (MainIRS.Inertial.Pos.Z - MainIRS.GroundShift) * 10.0f;//MainVel.TEST_SPEED;//DroneGoal.Position.Z-MainIRS.GroundShift;//DroneInertial.Spd.Z;

    inner.speed = fabsf(DroneGoal.Position.Z-(MainIRS.Inertial.Pos.Z - MainIRS.GroundShift))*10.0f;
    
    inner.pitchVel = (1000.f / DataCamFlow.Period) * 10.0f;//MainVel.PointEnd.X * 10.0f;//MainIRS.Inertial.Pos.Z - MainIRS.GroundShift;//DroneInertial.Spd.X;//MainVel.PointBegin.X;//PointLand.X; //MainBar.RegShift.Test;
    
    //inner.pitchVel = DroneGoal.Position.Z * 10.0f;
    
    //inner.yawVel = MainCam.ShiftSpd.X * 10.0f;//MainIRS.Inertial.Spd.Z;//DataGPS.Z;// / MainOF.RegShift.Accuracy;//MainIRS.Inertial.Spd.Z;//DroneInertial.Spd.Y;
    //inner.rollVel = MainCam.ShiftSpd.Y * 10.0f;//MainOF.RegShift.Test;//DroneInertial.Spd.Y;//MainVel.PointBegin.Y;//PointLand.Y; //DataGPS.X;//MainGPS.Shift.Dyn.Y / MainGPS.RegShift.Accuracy;//
    
    inner.yawVel = DataCamFlow.z.X * 10.0f;
    inner.rollVel = DataCamFlow.z.Y * 10.0f;
    
    //inner.rollVel = MainIRS.GroundShift * 10.0f;
    
    ProtoDataWriteReadInertialInfo(inner, false);
  }
  
  if(MainDrone.NeedSendRawCalibData)
  {// Raw calibration data
    ProtoDataRawCalib calibData;
    switch (MainDrone.Sensor)
    {
      case SENSOR_ID::ACC:
      {
        calibData.X = DataIMU.RawAcc.X;
        calibData.Y = DataIMU.RawAcc.Y;
        calibData.Z = DataIMU.RawAcc.Z;
        break;
      }
      case SENSOR_ID::IMU_MAG:
      {
        calibData.X = DataIMU.RawMag.X;
        calibData.Y = DataIMU.RawMag.Y;
        calibData.Z = DataIMU.RawMag.Z;
        break;
      }
      case SENSOR_ID::EXTERNAL_MAG:
      {
        calibData.X = DataMAG.RawX;
        calibData.Y = DataMAG.RawY;
        calibData.Z = DataMAG.RawZ;
        break;
      }
      case SENSOR_ID::LEVEL_HOR:
      {
        calibData.X = DroneOrintation.PRY.X;
        calibData.Y = DroneOrintation.PRY.Y;
        calibData.Z = DroneOrintation.PRY.Z;
        break;
      }
    }
    ProtoDataWriteReadRawCalib(calibData, false);
  }
  
  // Проверка надо ли калибровать датчики
  //if (!MainDrone.MainActive)
  //{
    //acc
    //if (CalibDataIMU.Data.Mag.
  //}
}

bool MagReady=false;

void DoneProcIMU(IMU_Data& Data)
{
  DataIMU = Data;
  MagReady=true;
}

void DoneProcBAR(bool Ready, BAR_Data& Data)
{
  if(Ready) DataBAR=Data;
}



void DoneProcMag(MAG_Data& Data)
{
  DataMAG = Data;
  MagReady=true;
}

void TimerUpdateSensor()
{
  if(MainDrone.IMUInit) IMUObj->GetAsync(DoneProcIMU);
  if(MainDrone.BarInit) BarObj->GetAsync(DoneProcBAR);
}

void TimerUpdateMain()
{
  float time = GetCurrentTime();
  
  Vector3 gyr = {(float)DataIMU.Gyr.X, (float)DataIMU.Gyr.Y, (float)DataIMU.Gyr.Z};
  Vector3 acc = {(float)DataIMU.Acc.X, (float)DataIMU.Acc.Y, (float)DataIMU.Acc.Z};
  
  Vector3 mag;
  if(MainDrone.ExternMagInit) mag = {(float)DataMAG.X, (float)DataMAG.Y, (float)DataMAG.Z};
  //else mag = {(float)DataIMU.Mag.X, (float)DataIMU.Mag.Y, (float)DataIMU.Mag.Z};
  
  MainIRS.UpdateGyro(gyr);
  MainIRS.UpdateAccel(acc);
  if(MagReady)
  {
    MagReady=false;
    MainIRS.UpdateMagnet(mag);
  }
  
  MainGPS.UpdateAverage(time);
  MainBar.UpdateAverage();
  MainOF.UpdateAverage(IRS::Period);
  MainLen.UpdateAverage();
  MainCam.UpdateAverage();
  
  if(DataCamFlow.Update)
  {
    DataCamFlow.Update=false;
    UpdateCam(DataCamFlow.XY, MainIRS.Inertial.Pos.Z - MainIRS.GroundShift, DataCamFlow.Valid, DataCamFlow.Time);
  }
  
  if(DataFLOW.Update)
  {
    DataFLOW.Update=false;
    UpdateFlow(DataFLOW.OF, MainIRS.Inertial.Pos.Z - MainIRS.GroundShift, DataFLOW.Valid, DataFLOW.Quality);
  }
  
  if(DataTOF.Update)
  {
    DataTOF.Update=false;
    UpdateRange(DataTOF.Range, DataTOF.Valid && DataTOF.Range>0.02f && DataTOF.Strength>200.0f);
  }
  
  UpdateBar(DataBAR.Pressure, DataBAR.Temp, DataBAR.Pressure>0.0f);
  
  if(DataGPS.Update)
  {
    DataGPS.Update=false;
    Vector3 gps = { DataGPS.X, DataGPS.Y, DataGPS.Z };
    UpdateGPS(gps, time, DataGPS.Valid);
  }
  
  Vector3 pos;
  MainIRS.RestoreAllShift(pos);
  //MainVel.PointBegin-=pos;
  
  AutoControl(gyr);
  
  ReadStateINS();
  
  if(MainDrone.ExternMagInit) MagObj->GetAsync(DoneProcMag);
}

void TimerUpdateOptics()
{
  { // Range
    float range, strength;
    bool ready = TOF_GetRange(range, strength);
    DataTOF.Valid = ready && (range<40.0f);
    if(DataTOF.Valid && !MainDrone.TOFInit) MainDrone.TOFInit = true; 
    DataTOF.Range = range;
    DataTOF.Strength=strength;
    DataTOF.Alpha = 1.0f;
    DataTOF.Update = ready;
  }
  
  { // Flow
    short dx, dy;
    unsigned char quality;
    bool done = FLOW_GetMotion(dx, dy, quality);
    DataFLOW.OF = {(float)dx, (float)dy};
    
    const float min = 20;
    DataFLOW.Valid = quality>min && done;
    DataFLOW.Quality = ((float)quality)/255.0f; // Качество поверхности
    DataFLOW.Update=true;
  }
}

void InitPDI()
{
  // Они плохо настроены, но летает и можно тестить остальное

  //                      Period     Limit     P      D       IL    IP     Lock I
  MainAtt.RegAngleP = { IRS::Period, 10.0f,  10.0f,  0.0f,  { 0.0f, 0.0f }, true };
  MainAtt.RegAngleR = { IRS::Period, 10.0f,  10.0f,  0.0f,  { 0.0f, 0.0f }, true };
  MainAtt.RegAngleY = { IRS::Period, 10.0f,  10.0f,  0.0f,  { 0.0f, 0.0f }, true };
                                                                                    
  MainAtt.RegSpeedP = { IRS::Period, 0.50f,  0.05f,  0.0f,  { 0.0f, 0.0f }, true};
  MainAtt.RegSpeedR = { IRS::Period, 0.50f,  0.05f,  0.0f,  { 0.0f, 0.0f }, true};
  MainAtt.RegSpeedY = { IRS::Period, 1.00f,  0.20f,  0.0f,  { 0.0f, 0.0f }, true};
                                            
  //                             (лимит наклонов)
  MainVel.RegSpeedX = { IRS::Period, 0.383f,  0.1f,   0.05f};
  MainVel.RegSpeedY = { IRS::Period, 0.383f,  0.1f,   0.05f};
  MainVel.RegSpeedT = { IRS::Period, 0.50f,   0.3f,   0.025f};
  
  
  // GPS              Restore    Accuracy    Power
  MainGPS.RegShift = { 30.0f,      2.0f,     4.0f };
  //                 Alpha    Power   Dynamic,   Distance
  MainGPS.RegSpd = { 0.3f,    2.0f,   0.1f,      10.0f };
  MainGPS.RegPos = { 0.3f,    2.0f,   0.1f,      10.0f };
  MainGPS.AlphaHeight = 0.01;
  

  // OF               Restore    Accuracy    Power
  MainOF.RegShift = { 50.0f,      0.5f,     2.0f };
  //                Alpha       Power      Dynamic,   Distance
  MainOF.RegSpd = { 0.5f,       2.0f,      0.25f,      1.0f };
  
  // CF               Restore    Accuracy    Power
  MainCam.RegShift = { 50.0f,      0.5f,     2.0f };
  //                Alpha       Power      Dynamic,   Distance
  MainCam.RegSpd = { 0.5f,       2.0f,      0.25f,      1.0f };
  
  
  // BAR              Restore    Accuracy    Power
  MainBar.RegShift = { 5.0f,       0.05f,      2.0f };
  //                 Alpha    Power    Dynamic,   Distance
  MainBar.RegSpd = { 0.1f,    2.0f,     0.02f,      3.0f };
  MainBar.RegPos = { 0.1f,    2.0f,     0.02f,      1.0f };
  
  // ToF              Restore    Accuracy    Power
  MainLen.RegShift = { 10.0f,       0.5f,      2.0f };
  //                 Alpha    Power    Dynamic,   Distance
  MainLen.RegSpd = { 1.0f,    2.0f,     0.1f,      2.0f };
}

void MEM_Save() // Записать всё во флеш 
{
  FLASH_Erase(63);
  
  unsigned char* mem = (unsigned char*)0x0801F800UL;
  
  unsigned long count=sizeof(Settings::MenuParameters)/sizeof(ParameterData*);
  bool FirstBytes = true;
  for(int a=0; a<count; a++)
  {
    const unsigned long max = 512;
    unsigned char buffer[max];
    unsigned long len = 0;
    
    if (FirstBytes) //Харним во флеше CRC дерева параметров
    {
      unsigned short crc = sizeof(Settings::MenuPIDParameters) + sizeof(Settings::MenuEngineParameters) + sizeof(Settings::MenuAutoPilotParameters) + sizeof(Settings::MenuSensorsParameters);
      memcpy(buffer, &crc, 2);
      len = 2;
      FirstBytes = false;
    }
    
    int sub=Settings::MenuParametersCount[a];
    for(int b=0; b<sub; b++)
    {
      const ParameterData& param = Settings::MenuParameters[a][b];
      
      unsigned long size=Settings::GetTypeSize(param.type);
      
      if(size) memcpy(buffer+len, param.value, size);
      len+=size;
    }
    
    if(len) 
    {
      FLASH_Write((unsigned long)mem, buffer, len);
      if(len & 7) len=(len & ~7)+8;
      mem+=len;
    }
  }
  MainDrone.FLASHSaved = true;
}

void MEM_Restore() // Восстановить всё из флеша
{
  unsigned char* mem = (unsigned char*)0x0801F800UL;
  
  unsigned long count=sizeof(Settings::MenuParameters)/sizeof(ParameterData*);
  
  bool FirstBytes = true;
  unsigned short crc = sizeof(Settings::MenuPIDParameters) + sizeof(Settings::MenuEngineParameters) + sizeof(Settings::MenuAutoPilotParameters) + sizeof(Settings::MenuSensorsParameters);
  unsigned short crs_in_mem = 0;
  memcpy(&crs_in_mem, mem, 2);
  
  if(crs_in_mem != crc) MEM_Save(); //Проверяем CRC дерева параметров
  
  for(int a=0; a<count; a++)
  {
    unsigned long len = 0;
    
    if(FirstBytes)
    {
      len = 2;
      FirstBytes = false;
    }
    
    int sub=Settings::MenuParametersCount[a];
    for(int b=0; b<sub; b++)
    {
      const ParameterData& param = Settings::MenuParameters[a][b];
      
      unsigned long size=Settings::GetTypeSize(param.type);
      
      if(size) memcpy(param.value, mem+len, size);
      len+=size;
    }
    
    if(len) 
    {
      if(len & 7) len=(len & ~7)+8;
      mem+=len;
    }
  }
  ExternMagCalibData.CalibNeed = false;
}

void DoneProcEEPROM(bool Ready, EEP_Data& Data)
{
  if(Ready)
  {
    // есть чтение
    CalibDataIMU.Data = *(XYZ_IMU_DATA*)(Data.Data);
    if (CalibDataIMU.Data.Writed != sizeof(XYZ_IMU_DATA))
    {
      CalibDataIMU.CalibAccNeed = true;
      CalibDataIMU.CalibMagNeed = true;
    }
    else
    {
      CalibDataIMU.CalibAccNeed = false;
      CalibDataIMU.CalibMagNeed = false;
    }
  }
  else
  {
    MainDrone.EEPROMSaved = true;
    // была запись
  }
}

void EEPROM_Save() // Пример сохранения
{
  CalibDataIMU.Data.Writed = sizeof(XYZ_IMU_DATA);
  if(MainDrone.EEPROMInit) EEPROMObj->SetAsunc(0, &CalibDataIMU.Data, sizeof(XYZ_IMU_DATA), DoneProcEEPROM);
}

void EEPROM_Restore() // Пример чтения
{
  if(MainDrone.EEPROMInit) EEPROMObj->GetAsunc(0, sizeof(XYZ_IMU_DATA), DoneProcEEPROM);
}

extern "C" void SystemClock_Config();

int main()
{
  SystemClock_Config(); // 170MHz
  LedControl.Processing(1);
  InitPDI();
  //MEM_Save();
  MEM_Restore();
  MainIRS.SetAccelShift(LevelHor.Pitch, LevelHor.Roll, 0.0f);
  MainIRS.SetNorthDeclination(0.0f);
 
  switch(MotorType)
  {
    case MOTOR_TYPE::PWM:
    {
      PWM_Init(100); // 100Hz
      PWM_SetAll(0);
      break;
    }
    case MOTOR_TYPE::DSHOT:
    {
      DSHOT_Init(600'000, false);
      break;
    }
  }
  
  /*while(1)
  {
    static unsigned short dshot_cmd[6]={0,0,0,0,0,0};
    DSHOT_SetCommand(dshot_cmd, true);
    for(volatile int a=0; a<100000; a++);
  }*/
                 
  GPIO_InitPin(GPIO_PIN_13 | GPIO_PORT_C | GPIO_OUTPUT | GPIO_SET); // POWER ON 3V3 (nPC13)
  for (volatile int i = 0; i < 1000000; i ++) {};
  GPIOC->BSRR = GPIO_BSRR_BR13;
  for (volatile int i = 0; i < 1000000; i ++) {};

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  
  TICK_Init();
  
  TIM7_Init(TIM_PRIORITY, 500, TimerUpdateSensor, TimerUpdateMain, TimerUpdateOptics);
  
  IMUObj = TryFindIMU(MainDrone.IMUInit);
  if (!MainDrone.IMUInit) LedControl.SetErrorBlink(true, 5);
  
  BarObj = TryFindBar(MainDrone.BarInit);
  
  if (MainDrone.IMUInit && !MainDrone.BarInit) LedControl.SetErrorBlink(true, 3);
  TOF_Init();
  MainDrone.OFInit = FLOW_Init();
  GPS_Init();
  
  MagObj = TryFindMag(MainDrone.ExternMagInit);
  
  ADC_Init();
 
  switch(JOPAD_Type)
  {
    case JOPAD_TYPE::SBUS: SBUS_Init(); UART2_Init(115200); break; // Телеметрия на UART2 
    case JOPAD_TYPE::CRSF: CRSF_Init(); CamFlow_Init(); break; //Телеметрия по CRSF, камера на UART2 
  }
  
  EEPROMObj = TryFindEEPROM(MainDrone.EEPROMInit);
  
  //LPUART1_Init(115200); // Autopilot
  
  EEPROM_Restore();
  
  TIM7_Enable();
  
  InitAuto();
  
  NoiseGPS.Rec.Init(NoiseGPS.Buffer,NoiseGPS.Freq, NoiseGPS.Past);
  
  while(true)
  {
    const unsigned long tick=TICK_GetCount();
    
    static float BatAlfa = 0.0001f;

    BAT_VAL = ((1.0f-BatAlfa) * BAT_VAL) + (BatAlfa * ((float)(ADC_GetVolt() / 1000.0f))); // Смотрим напряжение аккума (V)
    
    switch(JOPAD_Type)
    {
      case JOPAD_TYPE::SBUS: SBUS_Update(Joypad); break;
      case JOPAD_TYPE::CRSF: CRSF_Update(Joypad, &TELEM_BUFFER, &TELEM_RECV_SIZE); break;
    }
    
    TOF_Update3();
    
    if(GPS_Update())
    {
      //DroneNullPoint.X=47.32223892211914;
      //DroneNullPoint.Y=38.76748275756836;
      //DataGPS.Zero=gps_p;
      //DataGPS.Zero.Latitude=47.32223892211914;
      //DataGPS.Zero.Longitude=38.76748275756836;
      //DataGPS.Zero.Altitude=0;
      
      
      bool valid;
      Point gps_p;
      static float www_dx=0, www_dy=0, www_dz=0;
      if(GPS_GetCoordinates(gps_p, valid))
      {
        //valid=true;
        
        if(valid)
        {
          if (!MainDrone.GPSInit) MainDrone.GPSInit = true;
          if(!DroneNullPoint.saved) 
          {
            DroneNullPoint = {true, true, gps_p.Latitude, gps_p.Longitude, gps_p.Altitude}; // потом оптимизировать (две точки)
            DataGPS.Zero=gps_p;
          }
          GPS_LocalDistance(DataGPS.Zero, gps_p, www_dx, www_dy, www_dz);
          DataGPS.X=-www_dx;
          DataGPS.Y=-www_dy;
          DataGPS.Z=-www_dz;
          if (DataGPS.Z < 0.0f)
          {
            DroneNullPoint.Z = gps_p.Altitude;
            DataGPS.Zero.Altitude = gps_p.Altitude;
            DataGPS.Z = 0.0f;
          }
          DataGPS.AbsAlt=gps_p.Altitude;
        }
        
        static unsigned long timer_noiz=0;
        static bool fixed_noiz = false;
        
        if(fixed_noiz)
        {
          if(TICK_GetCount()>timer_noiz+GPSNoizTime) fixed_noiz=false;
        }
        
        if(GPSNoiz<GPSNoizDelta || fixed_noiz) 
        {
          valid = false;
          if (GPSNoiz<GPSNoizDelta) 
          {
            timer_noiz = TICK_GetCount();
            fixed_noiz = true;
          }
        }
        
        DataGPS.Valid=valid;
        DataGPS.Update=true;
        
        if (DataGPS.Valid && !ExternalGPSOff) USE_GPS = true;
        else USE_GPS = false;
      }
    }
    
    if(CamFlow_Update())
    {
      CamFlowData xyy;
      if (CamFlow_GetData(xyy))
      {
        // данные от камеры тут считаются актуальными и хорошими.
        DataCamFlow.Valid = true;
        DataCamFlow.Update = true;
        DataCamFlow.XY = {xyy.X, -xyy.Y};
        DataCamFlow.Yaw = xyy.YAW;
        
        DataCamFlow.z += DataCamFlow.XY * MainCam.Lens * DataTOF.Range;
        
        DataCamFlow.Period = TICK_GetCount() - DataCamFlow.Time;
        DataCamFlow.Time=TICK_GetCount();
        
        if (!MainDrone.CamFlowInit) MainDrone.CamFlowInit = true;
        
        //CamFlow_Send(MainIRS.Inertial.Pos.Z - MainIRS.GroundShift);
      }
    }
    if (MainDrone.CamFlowInit && TICK_GetCount() - DataCamFlow.Time > 1000) MainDrone.CamFlowInit = false; // Сбрасываем, если за 1 сек данных не было
    
    // Сохранение новых калибровок
    if (MainDrone.NeedSaveNewEEPROMData)
    {
      EEPROM_Save();
      MainDrone.NeedSaveNewEEPROMData = false;
    }
    // Сохранение новых параметров
    if (MainDrone.NeedSaveNewFLASHData)
    {
      MEM_Save();
      MainDrone.NeedSaveNewFLASHData = false;
      MainIRS.SetAccelShift(LevelHor.Pitch, LevelHor.Roll, 0.0f);
    }
    
    //Логическая проверка стиков пульта
    //SWA - ARM
    if ((Joypad.LastSWA != Joypad.SWA) && MainDrone.ManualInputAllowed)
    {
      Joypad.LastSWA = Joypad.SWA;
      if (Joypad.SWA > JOYPAD_MID)
      {
        //Вызов функции проверки можно ли включать моторы
        CheeckStartCondition();
      }
      else if ((Joypad.SWA < JOYPAD_MID))
      {
        MainDrone.engine = ENGINE_STATUS::disable;
        // Вырубаем моторы
      }
    }
    
    //SWC - GPS OFF
    //if ((Joypad.LastSWC != Joypad.SWC) && MainDrone.ManualInputAllowed)
    //{
    //  Joypad.LastSWC = Joypad.SWC;
    //  if (Joypad.SWC > JOYPAD_MID) ExternalGPSOff = true;
    //  else ExternalGPSOff = false;
    //}
    
    //SWD - Autopilot active
    if ((Joypad.LastSWD != Joypad.SWD) && MainDrone.ManualInputAllowed)
    {
      Joypad.LastSWD = Joypad.SWD;
      if (Joypad.SWD < JOYPAD_MID) 
      { 
        MainDrone.Mode = SYS_MODE::stabilized;
        MainDrone.AutoPilotActive = false;
      }
      else 
      {
        if (MainDrone.Mode == SYS_MODE::stabilized || MainDrone.Mode == SYS_MODE::acro) MainDrone.Mode = SYS_MODE::hold;
        MainDrone.AutoPilotActive = true;
      }
    }
    // Потеря доверия к пульту
    if (Joypad.FailSafe)
    {
      MainDrone.ManualInputAllowed = false;
    }
    //Восстановление доверия к пульту
    if (!Joypad.FailSafe && !MainDrone.ManualInputAllowed && Joypad.Active)
    {
      if (!MainDrone.MainActive)
      {
        MainDrone.ManualInputAllowed = true;
      }
      else
      {
        if ((Joypad.SWA > JOYPAD_MID) && (fabs(Joypad.Z - JOYPAD_MID) < 450.0f)) 
        {
          MainDrone.ManualInputAllowed = true;
        }
      }
    }
    // Автопосадка при потере пульта или при заряде АКб <= 20%
    if ((Joypad.FailSafe /*|| ExternalGPSOff*/ /*|| MainDrone.BatteryPer <= MainDrone.BatteryCriticalPer*/) && DroneGoal.Position.Z != APSettings.AltLand && MainDrone.MainActive)
    {
      if (ProtoMissionInfo.f_start)
      {
        ProtoMissionInfo.f_start = false;
        ProtoMissionInfo.f_pause = true;
      }
      AutoLand();
      //DroneGoal.Position.Z = APSettings.AltLand;
      //DroneGoal.New = true;
      //PointLand = {DroneInertial.Pos.X, DroneInertial.Pos.Y};
      //MainDrone.Mode = SYS_MODE::land;
      //MainDrone.AutoPilotActive = true;
    }
    
    rpiControl.Processing(Joypad.SWB);
    LedControl.Processing(tick);
    AutoThread();
    
    static long test_tick=0;
    test_tick++;
    
  }
}
//------------------------------------------------------------------------------
