#pragma once
#include "Protocol.h"
#include "SettingsTable.h"

#define BUILD_YEAR (((__DATE__ [7] - '0') * 1000) + ((__DATE__ [8] - '0') * 100) + ((__DATE__ [9] - '0') * 10) + (__DATE__ [10] - '0'))

#define BUILD_MONTH ( \
  __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
: __DATE__ [2] == 'b' ? 2 \
: __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
: __DATE__ [2] == 'y' ? 5 \
: __DATE__ [2] == 'l' ? 7 \
: __DATE__ [2] == 'g' ? 8 \
: __DATE__ [2] == 'p' ? 9 \
: __DATE__ [2] == 't' ? 10 \
: __DATE__ [2] == 'v' ? 11 \
: 12)

#define BUILD_DAY (((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10) + (__DATE__ [5] - '0'))

extern unsigned long FlashVersion;
extern unsigned long FirmwareVersion;
extern unsigned long UpdateVersion;

#pragma pack(push,1)

// Пока пусть тут полежат
static unsigned int BAT_CONS = 4400; //емкость батареи в mAh
static float BAT_MAX_VAL = 16.8f; // Макс. напряжение бат.
static float BAT_MIN_VAL = 13.2f; // Мин.
//

struct LevelHorData
{
  float Pitch;
  float Roll;
  float Yaw;
};
extern LevelHorData LevelHor;

class Settings
{
public:
  static constexpr const char* MainMenuNames[] { "Version", "PID", "Engine", "Sensors", "AutoPilot" };
public:
  static constexpr const ParameterData MenuVersion[]
  {
    {"Flash", &FlashVersion, DATA_TYPES::dt_unsigned_long},
    {"Firmware", &FirmwareVersion, DATA_TYPES::dt_unsigned_long},
    {"Update", &UpdateVersion, DATA_TYPES::dt_unsigned_long}
  };
  
  static constexpr const ParameterData MenuPIDParameters[] 
  {
    {"AngleP_Limit", &MainAtt.RegAngleP.Lim, DATA_TYPES::dt_float_32}, {"AngleP_P", &MainAtt.RegAngleP.Pro, DATA_TYPES::dt_float_32},
    {"AngleP_D", &MainAtt.RegAngleP.Der, DATA_TYPES::dt_float_32}, {"AngleP_IL", &MainAtt.RegAngleP.Int.Lim, DATA_TYPES::dt_float_32},
    {"AngleP_IP", &MainAtt.RegAngleP.Int.Pro, DATA_TYPES::dt_float_32}, {"AngleP_ILock", &MainAtt.RegAngleP.LockI, DATA_TYPES::dt_bool},

    {"AngleR_Limit", &MainAtt.RegAngleR.Lim, DATA_TYPES::dt_float_32}, {"AngleR_P", &MainAtt.RegAngleR.Pro, DATA_TYPES::dt_float_32},
    {"AngleR_D", &MainAtt.RegAngleR.Der, DATA_TYPES::dt_float_32}, {"AngleR_IL", &MainAtt.RegAngleR.Int.Lim, DATA_TYPES::dt_float_32},
    {"AngleR_IP", &MainAtt.RegAngleR.Int.Pro, DATA_TYPES::dt_float_32}, {"AngleR_ILock", &MainAtt.RegAngleR.LockI, DATA_TYPES::dt_bool},

    {"AngleY_Limit", &MainAtt.RegAngleY.Lim, DATA_TYPES::dt_float_32}, {"AngleY_P", &MainAtt.RegAngleY.Pro, DATA_TYPES::dt_float_32},
    {"AngleY_D", &MainAtt.RegAngleY.Der, DATA_TYPES::dt_float_32}, {"AngleY_IL", &MainAtt.RegAngleY.Int.Lim, DATA_TYPES::dt_float_32},
    {"AngleY_IP", &MainAtt.RegAngleY.Int.Pro, DATA_TYPES::dt_float_32}, {"AngleY_ILock", &MainAtt.RegAngleY.LockI, DATA_TYPES::dt_bool},

    {"Att_SpeedP_Limit", &MainAtt.RegSpeedP.Lim, DATA_TYPES::dt_float_32}, {"Att_SpeedP_P", &MainAtt.RegSpeedP.Pro, DATA_TYPES::dt_float_32},
    {"Att_SpeedP_D", &MainAtt.RegSpeedP.Der, DATA_TYPES::dt_float_32}, {"Att_SpeedP_IL", &MainAtt.RegSpeedP.Int.Lim, DATA_TYPES::dt_float_32},
    {"Att_SpeedP_IP", &MainAtt.RegSpeedP.Int.Pro, DATA_TYPES::dt_float_32}, {"Att_SpeedP_ILock", &MainAtt.RegSpeedP.LockI, DATA_TYPES::dt_bool},

    {"Att_SpeedR_Limit", &MainAtt.RegSpeedR.Lim, DATA_TYPES::dt_float_32}, {"Att_SpeedR_P", &MainAtt.RegSpeedR.Pro, DATA_TYPES::dt_float_32},
    {"Att_SpeedR_D", &MainAtt.RegSpeedR.Der, DATA_TYPES::dt_float_32}, {"Att_SpeedR_IL", &MainAtt.RegSpeedR.Int.Lim, DATA_TYPES::dt_float_32},
    {"Att_SpeedR_IP", &MainAtt.RegSpeedR.Int.Pro, DATA_TYPES::dt_float_32}, {"Att_SpeedR_ILock", &MainAtt.RegSpeedR.LockI, DATA_TYPES::dt_bool},

    {"Att_SpeedY_Limit", &MainAtt.RegSpeedY.Lim, DATA_TYPES::dt_float_32}, {"Att_SpeedY_P", &MainAtt.RegSpeedY.Pro, DATA_TYPES::dt_float_32},
    {"Att_SpeedY_D", &MainAtt.RegSpeedY.Der, DATA_TYPES::dt_float_32}, {"Att_SpeedY_IL", &MainAtt.RegSpeedY.Int.Lim, DATA_TYPES::dt_float_32},
    {"Att_SpeedY_IP", &MainAtt.RegSpeedY.Int.Pro, DATA_TYPES::dt_float_32}, {"Att_SpeedY_ILock", &MainAtt.RegSpeedY.LockI, DATA_TYPES::dt_bool},

    {"Vel_SpeedX_Limit", &MainVel.RegSpeedX.Lim, DATA_TYPES::dt_float_32}, {"Vel_SpeedX_P", &MainVel.RegSpeedX.Pro, DATA_TYPES::dt_float_32},
    {"Vel_SpeedX_D", &MainVel.RegSpeedX.Der, DATA_TYPES::dt_float_32}, {"Vel_SpeedX_IL", &MainVel.RegSpeedX.Int.Lim, DATA_TYPES::dt_float_32},
    {"Vel_SpeedX_IP", &MainVel.RegSpeedX.Int.Pro, DATA_TYPES::dt_float_32}, {"Vel_SpeedX_ILock", &MainVel.RegSpeedX.LockI, DATA_TYPES::dt_bool},

    {"Vel_SpeedY_Limit", &MainVel.RegSpeedY.Lim, DATA_TYPES::dt_float_32}, {"Vel_SpeedY_P", &MainVel.RegSpeedY.Pro, DATA_TYPES::dt_float_32},
    {"Vel_SpeedY_D", &MainVel.RegSpeedY.Der, DATA_TYPES::dt_float_32}, {"Vel_SpeedY_IL", &MainVel.RegSpeedY.Int.Lim, DATA_TYPES::dt_float_32},
    {"Vel_SpeedY_IP", &MainVel.RegSpeedY.Int.Pro, DATA_TYPES::dt_float_32}, {"Vel_SpeedY_ILock", &MainVel.RegSpeedY.LockI, DATA_TYPES::dt_bool},

    {"Vel_SpeedT_Limit", &MainVel.RegSpeedT.Lim, DATA_TYPES::dt_float_32}, {"Vel_SpeedT_P", &MainVel.RegSpeedT.Pro, DATA_TYPES::dt_float_32},
    {"Vel_SpeedT_D", &MainVel.RegSpeedT.Der, DATA_TYPES::dt_float_32}, {"Vel_SpeedT_IL", &MainVel.RegSpeedT.Int.Lim, DATA_TYPES::dt_float_32},
    {"Vel_SpeedT_IP", &MainVel.RegSpeedT.Int.Pro, DATA_TYPES::dt_float_32}, {"Vel_SpeedT_ILock", &MainVel.RegSpeedT.LockI, DATA_TYPES::dt_bool}
  };

  static constexpr const ParameterData MenuEngineParameters[] 
  { 
    {"Battery_mAh", &BAT_CONS, DATA_TYPES::dt_unsigned_int},
    {"Battery_min_v", &BAT_MIN_VAL, DATA_TYPES::dt_float_32},
    {"Battery_max_v", &BAT_MAX_VAL, DATA_TYPES::dt_float_32},
    {"Motor_UL", &MainEng.QuadSchemeX[0], DATA_TYPES::dt_unsigned_char},
    {"Motor_UR", &MainEng.QuadSchemeX[1], DATA_TYPES::dt_unsigned_char},
    {"Motor_DL", &MainEng.QuadSchemeX[2], DATA_TYPES::dt_unsigned_char},
    {"Motor_DR", &MainEng.QuadSchemeX[3], DATA_TYPES::dt_unsigned_char}
  };

  static constexpr const ParameterData MenuSensorsParameters[] 
  { 
    {"Bar_Gravity", &MainBar.Gravity, DATA_TYPES::dt_float_32},
    {"Ext_Mag_MX",&ExternMagCalibData.X[0], DATA_TYPES::dt_short },
    {"Ext_Mag_PX",&ExternMagCalibData.X[1], DATA_TYPES::dt_short },
    {"Ext_Mag_MY",&ExternMagCalibData.Y[0], DATA_TYPES::dt_short },
    {"Ext_Mag_PY",&ExternMagCalibData.Y[1], DATA_TYPES::dt_short },
    {"Ext_Mag_MZ",&ExternMagCalibData.Z[0], DATA_TYPES::dt_short },
    {"Ext_Mag_PZ",&ExternMagCalibData.Z[1], DATA_TYPES::dt_short },
    {"Level_Hor_Pitch", &LevelHor.Pitch, DATA_TYPES::dt_float_32},
    {"Level_Hor_Roll", &LevelHor.Roll, DATA_TYPES::dt_float_32},
    {"Level_Hor_Yaw", &LevelHor.Yaw, DATA_TYPES::dt_float_32},
    {"OF_Lens", &MainOF.Lens, DATA_TYPES::dt_float_32},
    {"CamFlow_Lens", &MainCam.Lens, DATA_TYPES::dt_float_32}
  };

  static constexpr const ParameterData MenuAutoPilotParameters[] 
  {
    {"Min_Alt_With_GPS", &MainVel.GroundLimit, DATA_TYPES::dt_float_32},
    {"Min_Speed_Land", &APSettings.MinSpeedLand, DATA_TYPES::dt_float_32},
    {"Max_Speed_Land", &APSettings.MaxSpeedLand, DATA_TYPES::dt_float_32},
    {"Height_MaxSpd_Land", &APSettings.HeightLandStop, DATA_TYPES::dt_float_32},
    {"Rad_Reach_Point", &APSettings.RadReachPoint, DATA_TYPES::dt_float_32},
    {"Alt_Auto_GH", &APSettings.AltAutoGoHome, DATA_TYPES::dt_float_32},
    {"Stop_Soft", &MainVel.StopSoft, DATA_TYPES::dt_bool},
    {"Min_Distance", &MainVel.MinDistance, DATA_TYPES::dt_float_32},
    {"Max_Distance", &MainVel.MaxDistance, DATA_TYPES::dt_float_32},
    {"Angle_Limit", &MainVel.AngleLimit, DATA_TYPES::dt_float_32}
  };

  static constexpr const ParameterData* MenuParameters[] 
  {
    MenuVersion, MenuPIDParameters, MenuEngineParameters, MenuSensorsParameters, MenuAutoPilotParameters
  };
public:
  static constexpr unsigned char MenuParametersCount[]  
  {
    sizeof(MenuVersion)/sizeof(ParameterData),
    sizeof(MenuPIDParameters)/sizeof(ParameterData),
    sizeof(MenuEngineParameters)/sizeof(ParameterData),
    sizeof(MenuSensorsParameters)/sizeof(ParameterData),
    sizeof(MenuAutoPilotParameters)/sizeof(ParameterData)
  };
  static const char* GetCategoryName(unsigned char id, unsigned char& size);
  static DATA_TYPES GetParameterDataType(unsigned char id_cat, unsigned char id_param);
  static const char* GetParameterName(unsigned char id_cat, unsigned char id_param, unsigned char& size);
  static unsigned char GetCountCategories();
  static unsigned char GetCountParameters(unsigned char id_cat);
  static unsigned char GetTypeSize(DATA_TYPES type);
  static unsigned char ReadOrWriteParameter(unsigned char id_cat, unsigned char id_param, unsigned char value[], bool read = true);
};
#pragma pack(pop)
// Sensors
// Baro Gravity = 0.1f
// OF Lens = 1.0f

// Engine
// Engine count = 4;
// Engine QuadSchemeX[] = 0,1,2,3
// Engine TiltСomp = 0.7f;
// Engine LimitThrot = 1.0f;
// Engine MinimumThrot = 0.75f;

// AutoPilot
// Velocity Freq = 100.0f
// Velocity AreaResist = 0.12f
// Velocity DynamicCoef = 8.5f
// Velocity MinDistance = 1.0f
// Velocity AngleLimit = 45.0f
// Velocity MoveSpeed = 1.0f