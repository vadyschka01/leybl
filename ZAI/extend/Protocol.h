#pragma once

#define MAX_LEN_MISSION_NAME 64
#define MAX_COUNT_MISSION_POINT 100

enum class MESSAGES_ID : unsigned char
{
  SysInfo = 1,
  GyroInfo = 2,
  AccelInfo = 3,
  GpsInfo = 4,
  InertialInfo = 5,
  BatteryInfo = 6,
  Ack = 7,
  StatusCommand = 8,
  StatusAllCommands = 9,
  Command = 10,
  RequestReg = 11,
  ResponseReg = 12,
  Auth = 13,
  OpenKey = 14,
  MissionCount = 15,
  MissionItem = 16,
  MissionItemAck = 17,
  MissionRequestItem = 18,
  RequestLastMessage = 19,
  ConnectionTest = 20,
  CountMenuCategories = 32,
  CategoriesMenu = 33,
  CategoriesParameters = 34,
  RawCalibData = 35
};
enum class COMMANDS_ID : unsigned char
{
  ChangeNav = 1,
  ChangeSpeed = 2,
  Land = 3,
  GoHome = 4,
  StopEngine = 5,
  StartEngine = 6,
  Pause = 7,
  Continue = 8,
  GoToGlobal = 9,
  GoToLocal = 10,
  SetParameter = 15,
  ProcessingCalibration = 16,
  SetCalibrationData = 17,
  MotorMoutionTest = 18
};
enum class ERROR_CODE_COMMAND : unsigned char
{
  No_error = 0,
  Speed_has_already_been_set = 1,
  Engines_are_already_running = 2,
  Engines_have_already_been_stopped = 3,
  UAV_is_already_on_the_ground = 4,
  UAV_is_already_landing = 5,
  UAV_is_already_moving = 6,
  Mission_has_not_been_launched = 7,
  Mission_is_already_on_pause = 8,
  Mission_was_not_on_pause = 9,
  Mission_was_not_launched = 10,
  This_NavSys_has_already_been_applied = 11,
  Lost_connection_with_controller = 12,
  The_command_is_canceled_when_a_new_one_is_received = 13,
  Error_reading_the_mission_file = 14,
  Error_validating_the_mission_file = 15,
  This_attack_event_already_exists = 16,
  Error_creating_the_configuration_file_moa = 17,
  Error_saving_the_configuration_file_moa = 18,
  Error_deleting_the_attack_event = 19,
  This_attack_event_cannot_be_deleted = 20,
  This_attack_event_does_not_exist = 21
};
enum class SENSOR_ID : unsigned char
{
  ACC = 0,
  IMU_MAG = 1,
  EXTERNAL_MAG = 2,
  LEVEL_HOR = 3
};
enum class ENGINE_STATUS : unsigned char
{
  disable = 0,
  enable = 1
};
enum class STATUS_MODE : unsigned char
{
  on_ground = 1,
  taking_off = 2,
  fly = 3,
  landing = 4
};
enum class SYS_MODE : unsigned char
{
  take_off = 1,
  land = 2,
  go_home = 3,
  auto_mode = 4,
  hold = 5,
  manual = 6
};
enum class NAV_SYS_MODE : unsigned char
{
  inertial = 1,
  gps = 2,
  Optical_flow = 3
};
enum class EXEC_CODE_COMMAND : unsigned char
{
  error = 0,
  in_progress = 1,
  completed = 2
};
enum class FEASIBILITY_CODE_COMMAND : unsigned char
{
  cannot_be_performed = 0,
  accepted = 1
};
enum class DATA_TYPES : unsigned char
{
  dt_char = 0,
  dt_unsigned_char = 1,
  dt_string = 2,
  dt_short = 3,
  dt_bool = 4,
  dt_unsigned_short = 5,
  dt_int = 6,
  dt_unsigned_int = 7,
  dt_long = 8,
  dt_unsigned_long = 9,
  dt_long_long = 10,
  dt_unsigned_long_long = 11,
  dt_float_32 = 12,
  dt_float_64 = 13,
  dt_unknown = 255
};

//extern CRITICAL_SECTION ProtoSection;

#pragma pack(push,1)

struct MissionItemData
{
  unsigned long long item_number;
  COMMANDS_ID item_command; // ID команды, может быть: движение к точке, возврат домой, посадка, смена скорости и др.
  char param1; // резерв
  float param2; // значение скорости, если в миссии есть команда смены скорости, для других команд - резерв
  char param3; // резерв
  double param4; // резерв
  double param5; // широта
  float param6; // долгота
  float param7; // абс. высота
};

struct MissionMetaData
{
  unsigned long long current_item;
  unsigned long long total_items;
  bool launch_confidentiality;
  bool mission_priority;
  bool fault_tolerence;
  bool f_start;
  bool f_pause;
  char mission_name[MAX_LEN_MISSION_NAME];
};
struct MissionItemsData
{
  MissionItemData items[MAX_COUNT_MISSION_POINT];
};

struct EnginePowerInfo
{
  unsigned char power; //Текущая нагрузка на мотор от 0 до 100%
  unsigned int engineSpeed;//Текущие оборты в секунду мотора
  float current; // Ток потребляемый мотором
  float voltage; // Напряжение на моторе
  float temp; // Температура мотора

  static const unsigned char MaxCount = 8;
};

struct ProtoDataBattery
{
  unsigned char perCharge;
  float voltage;
  float amperage;
  float temp;
  float totalPower;
  unsigned long long timeRemaining;
};
struct ProtoDataSysInfo
{
  SYS_MODE mode;
  STATUS_MODE statusMode;
  NAV_SYS_MODE navSys;
  ENGINE_STATUS engineStatus;
  unsigned char engineCount;
  float pressureBaro;
  float tempBaro;

  EnginePowerInfo enginePower[EnginePowerInfo::MaxCount];
};
struct ProtoDataGyroInfo
{
  float yawGyroVel;
  float pitchGyroVel;
  float rollGyroVel;
};
struct ProtoDataAccelInfo
{
  float yawAccelVel; 
  float pitchAccelVel; 
  float rollAccelVel; 
  float aX; 
  float aY; 
  float aZ; 
  float tempAccel; 
};
struct ProtoDataGpsInfo
{
  double lat;
  double lon;
  float absAlt;
  float realAlt;
  float hdop;
  float vdop;
  float pdop;
  float noise;
  float jamming;
  unsigned char satVisible;
  unsigned char satUsed;
  float speed;
  unsigned char fixType;
  unsigned long long timeUTC;
};
struct ProtoDataInertialInfo
{
  float x;
  float y;
  float z;
  float headingDeg;
  float speed;
  float roll;
  float pitch;
  float yaw;
  float rollVel;
  float pitchVel;
  float yawVel;
  float baroAlt;
};

struct ProtoDataRawCalib
{
  SENSOR_ID sensor;
  float X;
  float Y;
  float Z;
};

struct ParameterData
{
  const char* name;
  void* value;
  DATA_TYPES type;
};

#pragma pack(pop)

typedef void (*SendCallback)(void* data, unsigned long size);

void MainInit(SendCallback send);
bool MainUpdate(unsigned char byte);

void ProtoDataWriteReadBattery(ProtoDataBattery& data, bool read=true);
void ProtoDataWriteReadSysInfo(ProtoDataSysInfo& data, bool read = true);
//void ProtoDataWriteReadEngineInfo(EnginePowerInfo data[], unsigned char count, bool read = true);
void ProtoDataWriteReadGyroInfo(ProtoDataGyroInfo& data, bool read = true);
void ProtoDataWriteReadAccelInfo(ProtoDataAccelInfo& data, bool read = true);
void ProtoDataWriteReadGpsInfo(ProtoDataGpsInfo& data, bool read = true);
void ProtoDataWriteReadInertialInfo(ProtoDataInertialInfo& data, bool read = true);
void ProtoDataWriteReadRawCalib(ProtoDataRawCalib& data, bool read = true);

void RawCalibData();
