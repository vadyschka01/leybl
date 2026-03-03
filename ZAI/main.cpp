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

#include "imu.h"
#include "bar.h"
#include "tof.h"
#include "flow.h"
#include "gps.h"
#include "mag.h"
#include "eep.h"

#include "sbus.h"

#include "INS/IRS.h"

#include "INS/device/GPS.h"
#include "INS/device/Barometer.h"
#include "INS/device/Range.h"
#include "INS/device/OpticalFlow.h"
#include "INS/device/Compass.h"

#include "INS/cont/Attitude.h"
#include "INS/cont/Velocity.h"
#include "INS/cont/Engine.h"

#include "Autopilot.h"
#include "Settings.h"

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

FLOW_Data DataFLOW;
TOF_Data DataTOF;
GPS_Data DataGPS;

IRS MainIRS;

GPS MainGPS(MainIRS.RecordPosit, MainIRS.RecordSpeed);
Barometer MainBar(MainIRS.RecordPosit, MainIRS.RecordSpeed, MainIRS.RecordQuat);
Range MainLen(MainIRS.RecordSpeed, MainIRS.RecordQuat);
OpticalFlow MainOF(MainIRS.RecordGyro, MainIRS.RecordSpeed, MainIRS.RecordPosit, MainIRS.RecordQuat);
//Compass MainCompas(MainOF.RecordOF, MainIRS.RecordGyro, MainIRS.RecordQuat);
ControllerLED LedControl;

Attitude MainAtt;
Velocity MainVel(MainIRS.OriQuat);
Engine MainEng(MainIRS.OriPRT);

SBUS_Data Joypad;
bool MainActive=false;
float BAT_VAL;

unsigned long TimeManualActive = 0;

ButterworthCascadeFilter ButterBaroFilt(IRS::Freq);

float GetCurrentTime()
{
  return ((float)TICK_GetCount())/1000.0f;
}

void UpdateRange(float range, bool Valid) // Приоритет 4
{
  //return;
  
  if (!Valid)
  {
    MainIRS.SetGroundHeight(nullptr, MainLen.MaxHeight, 0.0005f);
    
    MainLen.SetRange(nullptr);
    
    return;
  }
  
  float len = MainLen.SetRange(&range);
 
  
  static float alpha_len=1.0f;
  
  if (len>0) 
  {
    if(len<MainLen.MaxHeight) alpha_len = 1.0f - len/MainLen.MaxHeight;
    else alpha_len = 0.1f;
    
    MainIRS.SetGroundHeight(&len, MainLen.MaxHeight, alpha_len*alpha_len);
  }
  else 
  {
    MainIRS.SetGroundHeight(nullptr, MainLen.MaxHeight, 0.0005f);
    
    alpha_len -= 0.01f;
    
    if(alpha_len<0.1f) alpha_len=0.1f;
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
  //return;
  
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
  //MainVel.UpdateMoveSpeed(pos, spd, acc, gnd);
  
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
    if (!MainDrone.MotorMoutionTestEnable) PWM_SetAll(900);
    else
    {
      short Power[4];
      Power[0] = 0;
      Power[1] = 0;
      Power[2] = 0;
      Power[3] = 0;
      Power[MainDrone.MotorTestId] = 1050;
      PWM_SetQuad(Power, 900, 1100);
      
      if (MainDrone.TimeMotorTestEnable == 0) MainDrone.TimeMotorTestEnable = TICK_GetCount();
      if (TICK_GetCount() - MainDrone.TimeMotorTestEnable > 3000)
      {
        MainDrone.TimeMotorTestEnable = 0;
        MainDrone.MotorMoutionTestEnable = false;
      }
    }
    MainDrone.MainActive = false;
    joy_yaw=pry.Z;
    DroneNullPoint.saved = 0;
    return;
  }
  
  CalibDataIMU.AllowedCalib = false;
  
  if (((thr_JOY < 0.01f) || !MainDrone.MainActive ) && MainDrone.ManualInputAllowed)
  {
    PWM_SetAll(1050);
    joy_yaw=pry.Z;
    if (!MainDrone.AutoPilotActive)
    {
      if (TimeManualActive == 0) TimeManualActive = TICK_GetCount();
      if (TICK_GetCount() - TimeManualActive > 3000)
      {
        MainDrone.MainActive = false;
        MainDrone.engine = ENGINE_STATUS::disable;
        TimeManualActive = 0;
      }
    }
    return;
  }
  
  TimeManualActive = 0;
  
  if (!MainDrone.AutoPilotActive)
  {
    MainDrone.AltChangeAllowed = false;
    bool spd = false;
    
    joy_yaw+=(float)(Joypad.W - JOY_MID)/440.0f;
    
    
    Vector3 pry = { (float)(JOY_MID - Joypad.Y)/22.0f, (float)(Joypad.X - JOY_MID)/22.0f, joy_yaw};
    pry /= 57.2957795f;
    Quaternion q_pry = Quaternion::CreateYawPitchRoll(pry);
    MainAtt.SetAnglePRY(&thr_JOY, &q_pry, &spd);
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

  }
  else
  {
    float newY = (float)(Joypad.Y - JOY_MID)/100000.0f;
    float newX = (float)(Joypad.X - JOY_MID)/100000.0f;
    float newZ = 0.0f;
    
    //if (fabs(Joypad.Z - JOY_MID) < 450) MainDrone.AltChangeAllowed = true;
    //if(fabs(Joypad.Z - JOY_MID) > 500) newZ = (float)(Joypad.Z - JOY_MID)/100000.0f;
    if (((newY != 0.0f) || (newX != 0.0f) || ((newZ != 0.0f) && MainDrone.AltChangeAllowed)) && MainDrone.ManualInputAllowed)
    {
      if (MainDrone.Mode != SYS_MODE::hold)
      {
        MainDrone.Mode = SYS_MODE::hold;
        DroneGoal.Position = pos;
        if (MainVel.GroundMode) DroneGoal.Position.Z -= MainIRS.GroundShift;
      }
      DroneGoal.Position.Y += newY;
      DroneGoal.Position.X += newX;
      DroneGoal.Position.Z += newZ;
      DroneGoal.New = true;
    }
   
   //float yaw = DroneGoal.PRY.Z;
    float yaw = DroneGoal.PRY.Z;
    
   MainVel.SetManualYaw(DroneGoal.Yaw,&yaw);
   if(DroneGoal.New)
   {
     Vector3* PointBegin = nullptr;
     if (MainDrone.PointBeginNeed)
     {
       PointBegin = &DroneInertial.Pos;
       if (MainVel.GroundMode) PointBegin->Z -= DroneInertial.GroundShift; 
       MainDrone.PointBeginNeed = false;
     }
     MainVel.SetDistanceSpeed(PointBegin, &DroneGoal.Position, &DroneGoal.Speed);
     DroneGoal.New = false; // Не задавать если уже получена !!!!!!!!!!!!!!!!!
   }
   else MainVel.SetDistanceSpeed(nullptr, nullptr, &DroneGoal.Speed);
   
   bool con[3];
   float throt;
   MainVel.GetThrotAnglePRY(&throt, &ang, con);
   ANG_TEST = ang;
   
   ang.X /= 57.2957795f;
   ang.Y /= 57.2957795f;
   //ang /= 57.2957795f;
   
   Quaternion q_pry = Quaternion::CreateYawPitchRoll(ang);
   
   MainAtt.SetAnglePRY(&throt, &q_pry, con);
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
  
  float thrust[4];
  MainEng.GetQuadX(thrust);
  
  float motor[4] = { thrust[MainEng.QuadSchemeX[0]], thrust[MainEng.QuadSchemeX[1]], thrust[MainEng.QuadSchemeX[2]], thrust[MainEng.QuadSchemeX[3]] };
  
  short power[4] = { (short)(motor[0]*1000.0f), (short)(motor[1]*1000.0f), (short)(motor[2]*1000.0f), (short)(motor[3]*1000.0f) };
  
  
  //power[0]=power[1]=power[2]=power[3]=Joypad.Z/2;
  
  static short testPower[4];
  testPower[0] = power[0];
  testPower[1] = power[1];
  testPower[2] = power[2];
  testPower[3] = power[3];
  
  PWM_SetQuad(power, 1100, 2000);
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
    sys.pressureBaro = MainBar.Bar;
    sys.tempBaro = MainBar.Temp;

    //
    sys.engineStatus = MainDrone.engine;
    
    float thrust[sizeof(MainEng.QuadSchemeX)];
    MainEng.GetQuadX(thrust);
    
    float AvgEngineThrust = 0.0f;
    for (int i = 0; i < sys.engineCount; i++)
    {
      EnginePowerInfo eng;
      
      eng.current = thrust[MainEng.QuadSchemeX[i]] * thrust[MainEng.QuadSchemeX[i]] * 20.0f;
      eng.engineSpeed = thrust[MainEng.QuadSchemeX[i]] * 1000;
      eng.power = eng.engineSpeed / 10;
      eng.temp = 65.0f;
      eng.voltage = BAT_VAL;
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
    bat.voltage = BAT_VAL;
    bat.amperage = 0.5f + current_A;
    bat.perCharge = 0;
    if (bat.voltage > 1e-12f)
    {
      if (bat.voltage < BAT_MIN_VAL) bat.perCharge = 0;
      else bat.perCharge = onePerVal*(bat.voltage - BAT_MIN_VAL);
      if (bat.perCharge > 100) bat.perCharge = 100;
    }
    
    bat.temp = 36.6f;
    bat.totalPower = bat.amperage * bat.voltage;
    bat.timeRemaining = (unsigned long long)((BAT_CONS/(bat.amperage * 1000.0f)) * 0.7f * 3600);
    //-------------------------------------------------------------------------------
    ProtoDataWriteReadBattery(bat, false);
  }

  { // GYR
    ProtoDataGyroInfo gyr;
    gyr.pitchGyroVel = DataMAG.RawX; //
    gyr.rollGyroVel = DataMAG.RawY; //
    gyr.yawGyroVel = DataMAG.RawZ; //
    ProtoDataWriteReadGyroInfo(gyr, false);
  }

  { // ACC
    ProtoDataAccelInfo acc;
    acc.aX = DroneInertial.Acc.X;//DataIMU.RawGyr.X;
    acc.aY = DroneInertial.Acc.Y;//DataIMU.RawGyr.Y;
    acc.aZ = DroneInertial.Acc.Z;//DataIMU.RawGyr.Z;
    acc.pitchAccelVel = DataIMU.RawAcc.X; //
    acc.rollAccelVel = DataIMU.RawAcc.Y; //
    acc.yawAccelVel = DataIMU.RawAcc.Z; //
    acc.tempAccel = 34.5f; //
    ProtoDataWriteReadAccelInfo(acc, false);
  }
  
  { // GPS
    ProtoDataGpsInfo gps;
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
    gps.lat = lat;
    gps.lon = lon;
    gps.absAlt = alt;
    gps.realAlt = r_alt;
    gps.hdop = hdop;
    gps.vdop = vdop;
    gps.pdop = pdop;
    gps.jamming = gpsInfo.jamming; // jamming indicator ublox.
    gps.noise = noise;
    gps.satVisible = satVis;
    gps.satUsed = satUsed;
    gps.speed = gpsInfo.speed;
    gps.fixType = gpsInfo.fixType;
    gps.timeUTC = gpsInfo.timeUTC;
    ProtoDataWriteReadGpsInfo(gps, false);
  }

  {// INNER
    ProtoDataInertialInfo inner;
    inner.x = DroneInertial.Pos.X;
    inner.y = DroneInertial.Pos.Y;
    inner.z = DroneInertial.Pos.Z;
    inner.baroAlt = MainBar.LastPos;
    inner.pitch = DroneOrintation.PRY.X;
    inner.roll = DroneOrintation.PRY.Y;
    inner.yaw = DroneOrintation.PRY.Z;
    inner.headingDeg = DroneOrintation.PRY.Z;
    //inner.speed = sqrtf(DroneInertial.Spd.X * DroneInertial.Spd.X + DroneInertial.Spd.Y * DroneInertial.Spd.Y);
    
    inner.speed = MainIRS.GroundShift;//MainVel.TEST_SPEED;//DroneGoal.Position.Z-MainIRS.GroundShift;//DroneInertial.Spd.Z;

    inner.pitchVel = MainIRS.Inertial.Pos.Z - MainIRS.GroundShift;//DroneInertial.Spd.X;//MainVel.PointBegin.X;//PointLand.X; //MainBar.RegShift.Test;
    
    inner.yawVel = MainIRS.Inertial.Spd.Z;//DataGPS.Z;// / MainOF.RegShift.Accuracy;//MainIRS.Inertial.Spd.Z;//DroneInertial.Spd.Y;
    
    inner.rollVel = MainOF.RegShift.Test;//DroneInertial.Spd.Y;//MainVel.PointBegin.Y;//PointLand.Y; //DataGPS.X;//MainGPS.Shift.Dyn.Y / MainGPS.RegShift.Accuracy;//
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
  if(MainDrone.IMUInit) IMU_GetAsunc(DoneProcIMU);
  if(MainDrone.BarInit) BAR_GetAsunc(DoneProcBAR);
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
  
  if(MainDrone.ExternMagInit) MAG_GetAsunc(DoneProcMag);
}

void TimerUpdateOptics()
{
  { // Range
    float range, strength;
    DataTOF.Valid = TOF_GetRange(range, strength);
    if(DataTOF.Valid && !MainDrone.TOFInit) MainDrone.TOFInit = true; 
    DataTOF.Range = range;
    DataTOF.Strength=strength;
    DataTOF.Alpha = 1.0f;
    DataTOF.Update = true;
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

  //                      Period     Limit    P      D       IL    IP    ID    Lock I
  MainAtt.RegAngleP = { IRS::Period, 10.0f, 5.0f, 0.0f, { 0.0f, 0.0f, 0.0f }, true };
  MainAtt.RegAngleR = { IRS::Period, 10.0f, 5.0f, 0.0f, { 0.0f, 0.0f, 0.0f }, true };
  MainAtt.RegAngleY = { IRS::Period, 10.0f, 5.0f, 0.0f, { 0.0f, 0.0f, 0.0f }, true };
                                                                                      
  MainAtt.RegSpeedP = { IRS::Period, 0.50f, 0.05f, 0.0f, { 0.0f, 0.0f, 0.0f }, true};
  MainAtt.RegSpeedR = { IRS::Period, 0.50f, 0.05f, 0.0f, { 0.0f, 0.0f, 0.0f }, true};
  MainAtt.RegSpeedY = { IRS::Period, 1.00f, 0.10f, 0.0f, { 0.0f, 0.0f, 0.0f }, true};
                                                                                                                                           
  MainVel.RegSpeedX = { IRS::Period, 0.50f, 0.3f,  0.1f};
  MainVel.RegSpeedY = { IRS::Period, 0.50f, 0.3f,  0.1f};
  MainVel.RegSpeedT = { IRS::Period, 0.50f, 0.3f,  0.1f};
  
  
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
  EEP_SetAsunc(0, &CalibDataIMU.Data, sizeof(XYZ_IMU_DATA), DoneProcEEPROM);
}

void EEPROM_Restore() // Пример чтения
{
  EEP_GetAsunc(0, sizeof(XYZ_IMU_DATA), DoneProcEEPROM);
}

extern "C" void SystemClock_Config();

int main()
{
  SystemClock_Config(); // 170MHz

  InitPDI();
  //MEM_Save();
  //MEM_Restore();
  MainIRS.SetAccelShift(LevelHor.Pitch, LevelHor.Roll, 0.0f);
  
  PWM_Init(100); // 100Hz
  PWM_SetAll(900);
  
  GPIO_InitPin(GPIO_PIN_13 | GPIO_PORT_C | GPIO_OUTPUT | GPIO_SET); // POWER ON 3V3 (nPC13)
  for (volatile int i = 0; i < 1000000; i ++) {};
  GPIOC->BSRR = GPIO_BSRR_BR13;
  for (volatile int i = 0; i < 1000000; i ++) {};

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  
  TICK_Init();
  
  TIM6_Init(TIM_PRIORITY, 500, TimerUpdateSensor, TimerUpdateMain);
  TIM7_Init(TIM_PRIORITY, OpticalFlow::Freq, TimerUpdateOptics);
  
  MainDrone.IMUInit = IMU_Init();
  if (!MainDrone.IMUInit) LedControl.SetErrorBlink(true, 5);
  MainDrone.BarInit = BAR_Init();
  if (MainDrone.IMUInit && !MainDrone.BarInit) LedControl.SetErrorBlink(true, 3);
  TOF_Init();
  MainDrone.OFInit = FLOW_Init();
  GPS_Init();
  MainDrone.ExternMagInit = MAG_Init();
  ADC_Init();
  
  //LED_Init();
  SBUS_Init();
  MainDrone.EEPROMInit = EEP_Init();
  
  UART2_Init(115200); // Autopilot
  //LPUART1_Init(115200); // Autopilot
  
  EEPROM_Restore();
  
  TIM6_Enable();
  TIM7_Enable();
  
  InitAuto();
  
  NoiseGPS.Rec.Init(NoiseGPS.Buffer,NoiseGPS.Freq, NoiseGPS.Past);
  
  while(true)
  {
    const unsigned long tick=TICK_GetCount();

    BAT_VAL = (float)(ADC_GetVolt() / 1000.0f); // Смотрим напряжение аккума (V)
    
    SBUS_Update(Joypad);
    
    TOF_Update2();
    
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
            DroneNullPoint = {true, gps_p.Latitude, gps_p.Longitude, gps_p.Altitude}; // потом оптимизировать (две точки)
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
      if (Joypad.SWA > JOY_MID)
      {
        //Вызов функции проверки можно ли включать моторы
        CheeckStartCondition();
      }
      else if ((Joypad.SWA < JOY_MID))
      {
        MainDrone.engine = ENGINE_STATUS::disable;
        // Вырубаем моторы
      }
    }
    
    //SWC - GPS OFF
    if ((Joypad.LastSWC != Joypad.SWC) && MainDrone.ManualInputAllowed)
    {
      Joypad.LastSWC = Joypad.SWC;
      if (Joypad.SWC > JOY_MID) ExternalGPSOff = true;
      else ExternalGPSOff = false;
    }
    if (!USE_GPS) 
    {
      MainVel.GroundMode = true;
      MainVel.GroundSpeed = CalcLandSpeed(DroneInertial.Pos.Z);
    }
    else
    {
      MainVel.GroundMode = false;
    }
    
    //SWD - Autopilot active
    if ((Joypad.LastSWD != Joypad.SWD) && MainDrone.ManualInputAllowed)
    {
      Joypad.LastSWD = Joypad.SWD;
      if (Joypad.SWD < JOY_MID) 
      { 
        MainDrone.Mode = SYS_MODE::manual;
        MainDrone.AutoPilotActive = false;
      }
      else 
      {
        if (MainDrone.Mode == SYS_MODE::manual) MainDrone.Mode = SYS_MODE::hold;
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
        if ((Joypad.SWA > JOY_MID) && (fabs(Joypad.Z - JOY_MID) < 450.0f)) 
        {
          MainDrone.ManualInputAllowed = true;
        }
      }
    }
    // Автопосадка при потере пульта или при отключении GPS
    if ((Joypad.FailSafe /*|| ExternalGPSOff*/) && DroneGoal.Position.Z != APSettings.AltLand && MainDrone.MainActive)
    {
      DroneGoal.Position.Z = APSettings.AltLand;
      DroneGoal.New = true;
      PointLand = {DroneInertial.Pos.X, DroneInertial.Pos.Y};
      MainDrone.Mode = SYS_MODE::land;
      MainDrone.AutoPilotActive = true;
    }
    
    LedControl.Processing(tick);
    AutoThread();
    
    static long test_tick=0;
    test_tick++;
    
  }
}
//------------------------------------------------------------------------------
