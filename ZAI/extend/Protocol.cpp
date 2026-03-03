#include <math.h>
#include <string.h>

#include "tick.h"

#include "Protocol.h"
#include "Messages.h"
#include "Autopilot.h"
#include "Settings.h"
#include "device/imu.h"

const unsigned char SRC_ID = 1;
const unsigned char DST_ID = 254;
bool GPS_HACK = false;

static void (*SendData)(void* data, unsigned long size) = 0;

void MainInit(SendCallback sendcall)
{
  SendData = sendcall;
}

enum class DataMode : unsigned char { Begin, Head, Data };

static unsigned char ProtoData[MaxLength];
static DataMode Mode = DataMode::Begin;
static unsigned long Length, Index;

static const unsigned long Wait = 100; // ms
static unsigned long Timer;

static void ProcessingMessage();

bool MainUpdate(unsigned char byte)
{
  const unsigned long tick = TICK_GetCount();

  //if ((tick - Timer > 100)) // Момент долгой обработки
  //{
  //  printf("------------> Warning: %d\n", tick - Timer);
  //}

  if ((Mode != DataMode::Begin) && (tick - Timer > Wait))
  {
    Mode = DataMode::Begin;
  }

  Timer = tick;

  switch (Mode)
  {
    case DataMode::Begin:
    {
      if (byte != Global_stx) return true;

      Index = 0;
      ProtoData[Index++] = byte;
      Length = sizeof(HeaderBegin);
      Mode = DataMode::Head;

      return true;
    }

    case DataMode::Head:
    {
      ProtoData[Index++] = byte;
      if (Index < Length) return true;

      HeaderBegin& begin = *(HeaderBegin*)ProtoData;

      if ((begin.len1 != begin.len2) || (begin.len1 > MaxLength)
        || (begin.len1 < sizeof(HeaderMessages)) || Index < sizeof(HeaderBegin))
      {
        Mode = DataMode::Begin;
        return true;
      }

      Mode = DataMode::Data;
      Length += begin.len1;

      return true;
    }

    case DataMode::Data:
    {
      ProtoData[Index++] = byte;
      if (Index < Length) return true;

      HeaderBegin& begin = *(HeaderBegin*)ProtoData;
      Mode = DataMode::Begin;
      if (!begin.CheckCRC())
      {
        return true;
      }
      ProcessingMessage();
      return true;
    }
  }

  return false;
}

void SendMes(HeaderBegin& begin)
{
  begin.SetCRC();
  SendData(&begin, begin.len1 + sizeof(HeaderBegin));
}

MissionMetaData ProtoMissionInfo;
MissionItemsData ProtoMissionData;

static ProtoDataBattery payloadBat;
static ProtoDataSysInfo payloadSysInfo;
static ProtoDataGyroInfo payloadGyroInfo;
static ProtoDataAccelInfo payloadAccelInfo;
static ProtoDataGpsInfo payloadGpsInfo;
static ProtoDataInertialInfo payloadInertialInfo;
static ProtoDataRawCalib payloadRawCalibData;

void ProtoDataWriteReadBattery(ProtoDataBattery& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadBat;
  else payloadBat = data;
  //LeaveCriticalSection(&ProtoSection);
};
void ProtoDataWriteReadSysInfo(ProtoDataSysInfo& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadSysInfo;
  else payloadSysInfo = data;
  //LeaveCriticalSection(&ProtoSection);
}

//void ProtoDataWriteReadEngineInfo(EnginePowerInfo data[], unsigned char count, bool read=true)
//{
//  // EnterCritSection();
//  if (read)
//  {
//    for (int i = 0; i < count; i++)
//    {
//      data[i] = payloadSysInfo.enginePower[i];
//    }
//  }
//  else
//  {
//    for (int i = 0; i < count; i++)
//    {
//      payloadSysInfo.enginePower[i] = data[i];
//    }
//  }
//  // LeaveCritSection();
//}

void ProtoDataWriteReadGyroInfo(ProtoDataGyroInfo& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadGyroInfo;
  else payloadGyroInfo = data;
  //LeaveCriticalSection(&ProtoSection);
}
void ProtoDataWriteReadAccelInfo(ProtoDataAccelInfo& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadAccelInfo;
  else payloadAccelInfo = data;
  //LeaveCriticalSection(&ProtoSection);
}
void ProtoDataWriteReadGpsInfo(ProtoDataGpsInfo& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadGpsInfo;
  else payloadGpsInfo = data;
  //LeaveCriticalSection(&ProtoSection);
}
void ProtoDataWriteReadInertialInfo(ProtoDataInertialInfo& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadInertialInfo;
  else payloadInertialInfo = data;
  //LeaveCriticalSection(&ProtoSection);
}
void ProtoDataWriteReadRawCalib(ProtoDataRawCalib& data, bool read)
{
  //EnterCriticalSection(&ProtoSection);
  if (read) data = payloadRawCalibData;
  else payloadRawCalibData = data;
  //LeaveCriticalSection(&ProtoSection);
}

bool HeaderBegin::CheckCRC()
{
  unsigned char* data = (unsigned char*)(this + 1);
  unsigned char test = 0;
  for (unsigned short a = 0; a < len1; a++) test ^= data[a];
  return crc == test;
}
void HeaderBegin::SetCRC()
{
  unsigned char* data = (unsigned char*)(this + 1);
  unsigned char test = 0;
  for (unsigned short a = 0; a < len1; a++) test ^= data[a];
  crc = test;
}
void BeginHeader::CreateMessageHeaders(unsigned long size, MESSAGES_ID msg_id)
{
  begin.len1 = begin.len2 = size - sizeof(HeaderBegin);
  begin.stx = Global_stx;
  header.len = size - sizeof(BeginHeader);
  header.msgId = msg_id;
  header.srcId = SRC_ID;
  header.dstId = DST_ID;

  header.timeusec = TICK_GetCount();
}

static void BatteryInfo()
{
  MessageBatteryInfo MsgBatteryInfo;
  MsgBatteryInfo.headers.CreateMessageHeaders(sizeof(MsgBatteryInfo), MESSAGES_ID::BatteryInfo);
  ProtoDataWriteReadBattery(MsgBatteryInfo.payload);
  SendMes(MsgBatteryInfo.headers.begin);
}
static void SysInfo()
{
  ProtoDataSysInfo data;
  ProtoDataWriteReadSysInfo(data);
  if (data.engineCount == 0) return; // отдать можем только после хотя бы 1 обновления

  unsigned int sub_size = sizeof(EnginePowerInfo) * (EnginePowerInfo::MaxCount - data.engineCount);

  MessageSysInfo MsgSysInfo;
  MsgSysInfo.headers.CreateMessageHeaders(sizeof(MsgSysInfo) - sub_size, MESSAGES_ID::SysInfo);
  MsgSysInfo.payload = data;

  // их меняет автопилот, у проткола и автопилота 1 нить, поэтому так
  MsgSysInfo.payload.mode = MainDrone.Mode;
  MsgSysInfo.payload.navSys = MainDrone.Nav;
  MsgSysInfo.payload.statusMode = MainDrone.StatusMode;

  SendMes(MsgSysInfo.headers.begin);
}
static void GyroInfo()
{
  MessageGyroInfo MsgGyroInfo;
  MsgGyroInfo.headers.CreateMessageHeaders(sizeof(MsgGyroInfo), MESSAGES_ID::GyroInfo);
  ProtoDataWriteReadGyroInfo(MsgGyroInfo.payload);
  SendMes(MsgGyroInfo.headers.begin);
}
static void AccelInfo()
{
  MessageAccelInfo MsgAccelInfo;
  MsgAccelInfo.headers.CreateMessageHeaders(sizeof(MsgAccelInfo), MESSAGES_ID::AccelInfo);
  ProtoDataWriteReadAccelInfo(MsgAccelInfo.payload);
  SendMes(MsgAccelInfo.headers.begin);
}
static void GpsInfo()
{
  MessageGpsInfo MsgGpsInfo;
  MsgGpsInfo.headers.CreateMessageHeaders(sizeof(MsgGpsInfo), MESSAGES_ID::GpsInfo);
  ProtoDataWriteReadGpsInfo(MsgGpsInfo.payload);
  SendMes(MsgGpsInfo.headers.begin);
}
static void InertialInfo()
{
  MessageInertialInfo MsgInertialInfo;
  MsgInertialInfo.headers.CreateMessageHeaders(sizeof(MsgInertialInfo), MESSAGES_ID::InertialInfo);
  ProtoDataWriteReadInertialInfo(MsgInertialInfo.payload);
  SendMes(MsgInertialInfo.headers.begin);
}
void RawCalibData()
{
  MessageRawCalibData MsgRawCalib;
  MsgRawCalib.headers.CreateMessageHeaders(sizeof(MsgRawCalib), MESSAGES_ID::RawCalibData);
  ProtoDataWriteReadRawCalib(MsgRawCalib.payload);
  SendMes(MsgRawCalib.headers.begin);
}
static void SendCommandAck(COMMANDS_ID Id, ERROR_CODE_COMMAND ErrorCode, FEASIBILITY_CODE_COMMAND FeasCode)
{
  MessageCommandAck msg;
  msg.headers.CreateMessageHeaders(sizeof(msg), MESSAGES_ID::Ack);

  msg.commandId = Id;
  msg.errorCode = ErrorCode;
  msg.status = FeasCode;
  //printf("MessageCommandACK: Send\n");
  SendMes(msg.headers.begin);
}
static void ProcessingCommandLand()
{
  // Выполняет команду...
  if (MainDrone.StatusMode == STATUS_MODE::on_ground)
  {
    SendCommandAck(COMMANDS_ID::Land, ERROR_CODE_COMMAND::UAV_is_already_on_the_ground, FEASIBILITY_CODE_COMMAND::cannot_be_performed);
  }
  else if (MainDrone.StatusMode == STATUS_MODE::landing)
  {
    SendCommandAck(COMMANDS_ID::Land, ERROR_CODE_COMMAND::UAV_is_already_landing, FEASIBILITY_CODE_COMMAND::cannot_be_performed);
  }
  else if ((MainDrone.StatusMode == STATUS_MODE::fly) || MainDrone.StatusMode == STATUS_MODE::taking_off)
  {
    AutoLand();
    SendCommandAck(COMMANDS_ID::Land, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  }

}
static void ProcessingCommandPause()
{
  // Выполняет команду...

  SendCommandAck(COMMANDS_ID::Pause, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandGoHome()
{
  // Выполняет команду...
  AutoGoHome();

  SendCommandAck(COMMANDS_ID::GoHome, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandContinue()
{
  // Выполняет команду...

  SendCommandAck(COMMANDS_ID::Continue, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandStartEngine()
{
  // Выполняет команду...

  SendCommandAck(COMMANDS_ID::StartEngine, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandStopEngine()
{
  // Выполняет команду...

  SendCommandAck(COMMANDS_ID::StopEngine, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandChangeSpeed(MessageCommandChangeSpeed* msg)
{
  // Выполняет команду...
  AutoSetSpeed(msg->speed);
  SendCommandAck(COMMANDS_ID::ChangeSpeed, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandChangeNav(MessageCommandChangeNav* msg)
{
  // Выполняет команду...
  if (msg->nav_id == NAV_SYS_MODE::inertial) ExternalGPSOff = true;
  if (msg->nav_id == NAV_SYS_MODE::gps) ExternalGPSOff = false;

  SendCommandAck(COMMANDS_ID::ChangeNav, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
}
static void ProcessingCommandGoToGlobal(MessageCommandGoToGlobal* msg)
{
  // Выполняет команду...
  AutoGoToGlobal(msg->lat, msg->lon, msg->abs_alt, msg->speed);
  SendCommandAck(COMMANDS_ID::GoToGlobal, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandGoToLocal(MessageCommandGoToLocal* msg)
{
  // Выполняет команду...
  AutoGoToLocal(msg->x, msg->y, msg->z, msg->speed);
  SendCommandAck(COMMANDS_ID::GoToLocal, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);

}
static void ProcessingCommandSetParameter(MessageCommandSetParameter* msg)
{
  unsigned char cat_id = msg->cat_id; // ID категории
  unsigned char param_id = msg->param_id; // ID параметра
  if (cat_id == 255 && param_id == 255)
  {
    MainDrone.NeedSaveNewFLASHData = true;
    SendCommandAck(COMMANDS_ID::SetParameter, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  }
  else
  {
    DATA_TYPES param_type = msg->param_type_code; // Тип данных
    unsigned char param_len = msg->param_len; // Размер массива данных
    msg->param_data; // Массив байт параметра
    Settings::ReadOrWriteParameter(cat_id, param_id, msg->param_data, false);
    SendCommandAck(COMMANDS_ID::SetParameter, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  }
}

static void ProcessingCommandProcessingCalibration(MessageCommandProcessingCalibration* msg)
{
  MainDrone.TimeStartSend = 0;
  MainDrone.TimeLastSend = 0;
  MainDrone.NeedSendRawCalibData = false;
  
  if (!(msg ->start))
  {
    SendCommandAck(COMMANDS_ID::ProcessingCalibration, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
    return;
  }
  if(!MainDrone.MainActive)
  {
    MainDrone.NeedSendRawCalibData = true;
    MainDrone.Sensor = msg->sensor_id;
    SendCommandAck(COMMANDS_ID::ProcessingCalibration, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  }
  else
  {
    SendCommandAck(COMMANDS_ID::ProcessingCalibration, ERROR_CODE_COMMAND::UAV_is_already_moving, FEASIBILITY_CODE_COMMAND::cannot_be_performed);
  }
}

static void ProcessingCommandSetCalibrationData(MessageCommandSetCalibrationData* msg)
{
  MainDrone.TimeStartSend = 0;
  MainDrone.TimeLastSend = 0;
  MainDrone.NeedSendRawCalibData = false;
  MainDrone.EEPROMSaved = false;
  MainDrone.FLASHSaved = false;
  SendCommandAck(COMMANDS_ID::SetCalibrationData, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  MainDrone.Sensor = msg->sensor_id;
  switch(msg->sensor_id)
  {
    case SENSOR_ID::ACC:
    {
      CalibDataIMU.Data.Acc.X[0] = (short)msg->X[0];
      CalibDataIMU.Data.Acc.X[1] = (short)msg->X[1];
      CalibDataIMU.Data.Acc.Y[0] = (short)msg->Y[0];
      CalibDataIMU.Data.Acc.Y[1] = (short)msg->Y[1];
      CalibDataIMU.Data.Acc.Z[0] = (short)msg->Z[0];
      CalibDataIMU.Data.Acc.Z[1] = (short)msg->Z[1];
      MainDrone.NeedSaveNewEEPROMData = true;
      return;
    }
    case SENSOR_ID::IMU_MAG:
    {
      CalibDataIMU.Data.Mag.X[0] = (short)msg->X[0];
      CalibDataIMU.Data.Mag.X[1] = (short)msg->X[1];
      CalibDataIMU.Data.Mag.Y[0] = (short)msg->Y[0];
      CalibDataIMU.Data.Mag.Y[1] = (short)msg->Y[1];
      CalibDataIMU.Data.Mag.Z[0] = (short)msg->Z[0];
      CalibDataIMU.Data.Mag.Z[1] = (short)msg->Z[1];
      MainDrone.NeedSaveNewEEPROMData = true;
      return;
    }
    case SENSOR_ID::EXTERNAL_MAG:
    {
      ExternMagCalibData.X[0] = (short)msg->X[0];
      ExternMagCalibData.X[1] = (short)msg->X[1];
      ExternMagCalibData.Y[0] = (short)msg->Y[0];
      ExternMagCalibData.Y[1] = (short)msg->Y[1];
      ExternMagCalibData.Z[0] = (short)msg->Z[0];
      ExternMagCalibData.Z[1] = (short)msg->Z[1];
      MainDrone.NeedSaveNewFLASHData = true;
      return;
    }
    case SENSOR_ID::LEVEL_HOR:
    {
      LevelHor.Pitch = msg->X[0];
      LevelHor.Roll = msg->X[1];
      LevelHor.Yaw = msg->Y[0];
      MainDrone.NeedSaveNewFLASHData = true;
      return;
    }
  }
}

static void ProcessingCommandMotorMoutionTest(MessageCommandMotorMoutionTest* msg)
{
  if(MainDrone.engine == ENGINE_STATUS::enable || MainDrone.MotorMoutionTestEnable) SendCommandAck(COMMANDS_ID::MotorMoutionTest, ERROR_CODE_COMMAND::Engines_are_already_running, FEASIBILITY_CODE_COMMAND::cannot_be_performed);
  else
  {
    MainDrone.MotorTestId = msg -> motor_id;
    MainDrone.MotorMoutionTestEnable = true;
    SendCommandAck(COMMANDS_ID::MotorMoutionTest, ERROR_CODE_COMMAND::No_error, FEASIBILITY_CODE_COMMAND::accepted);
  }
  return;
}

static void ProcessingCommand(MessageCommand* msg) 
{
  switch (msg->commandId)
  {
  case COMMANDS_ID::Land: ProcessingCommandLand(); return;
  case COMMANDS_ID::Pause: ProcessingCommandPause(); return;
  case COMMANDS_ID::GoHome: ProcessingCommandGoHome(); return;
  case COMMANDS_ID::Continue: ProcessingCommandContinue(); return;
  case COMMANDS_ID::StartEngine: ProcessingCommandStartEngine(); return;
  case COMMANDS_ID::StopEngine: ProcessingCommandStopEngine(); return;
  case COMMANDS_ID::ChangeSpeed: ProcessingCommandChangeSpeed((MessageCommandChangeSpeed*)ProtoData); return;
  case COMMANDS_ID::ChangeNav: ProcessingCommandChangeNav((MessageCommandChangeNav*)ProtoData); return;
  case COMMANDS_ID::GoToGlobal: ProcessingCommandGoToGlobal((MessageCommandGoToGlobal*)ProtoData); return;
  case COMMANDS_ID::GoToLocal: ProcessingCommandGoToLocal((MessageCommandGoToLocal*)ProtoData); return;
  case COMMANDS_ID::SetParameter: ProcessingCommandSetParameter((MessageCommandSetParameter*)ProtoData); return;
  case COMMANDS_ID::ProcessingCalibration: ProcessingCommandProcessingCalibration((MessageCommandProcessingCalibration*)ProtoData); return;
  case COMMANDS_ID::SetCalibrationData: ProcessingCommandSetCalibrationData((MessageCommandSetCalibrationData*)ProtoData); return;
  case COMMANDS_ID::MotorMoutionTest: ProcessingCommandMotorMoutionTest((MessageCommandMotorMoutionTest*)ProtoData); return;
  }
}
static void ConnectionTest()
{
  BeginHeader msg;
  msg.CreateMessageHeaders(sizeof(msg), MESSAGES_ID::ConnectionTest);
  SendMes(msg.begin);
}
static void SendStatusCommand(MessageStatusCommand* request)
{
  COMMANDS_ID comId = request->commandId;

  MessageStatusCommand msg;
  msg.headers.CreateMessageHeaders(sizeof(msg), MESSAGES_ID::StatusCommand);

  msg.commandId = comId;
  msg.executionCode = CheeckStatusCommand(comId);
  msg.errorCode = ERROR_CODE_COMMAND::No_error;
  SendMes(msg.headers.begin);
}
static void CountMenuCategories()
{
  unsigned char count_categories = Settings::GetCountCategories();
  MessageCountMenuCategories msg;
  unsigned long size = sizeof(msg) + count_categories * sizeof(unsigned char);

  msg.headers.CreateMessageHeaders(size, MESSAGES_ID::CountMenuCategories);

  msg.countCategories = count_categories;
  for (int i = 0; i < msg.countCategories; i++) {
    msg.data[i] = strlen(Settings::MainMenuNames[i]);
  }
  SendMes(msg.headers.begin);
}
static void MenuCategories(MessageMenuCategories* request)
{
  unsigned char req_count = request->countCategories;
  unsigned char total_len_names = 0;
  for (int i = 0; i < req_count; i++)
  {
    unsigned char cat_id = (request->categories)[i + i];
    unsigned char cat_name_len;
    Settings::GetCategoryName(cat_id, cat_name_len);
    total_len_names += cat_name_len;
  }

  MessageMenuCategories msg;
  unsigned long size = sizeof(msg) + req_count * sizeof(MenuCategori) + total_len_names * sizeof(unsigned char);
  msg.headers.CreateMessageHeaders(size, MESSAGES_ID::CategoriesMenu);

  msg.countCategories = req_count;
  MenuCategori* menu = (MenuCategori*)msg.categories;
  for (int i = 0; i < req_count; i++)
  {
    unsigned char cat_id = (request->categories)[i + i];
    menu->id = cat_id;
    menu->count_param = Settings::GetCountParameters(cat_id);
    const char* name = Settings::GetCategoryName(cat_id, menu->len_name);
    memcpy(menu->name, name, menu->len_name);
    menu = (MenuCategori*)(((char*)menu) + sizeof(MenuCategori) + menu->len_name);
  }
  SendMes(msg.headers.begin);
}
static void CategoriesParameters(MessageCategoriesParameters* request)
{
  MessageCategoriesParameters msg;

  const unsigned char cat_id = request->cat_id; // id категории для поиска параметра
  const unsigned char param_id = request->param_id; // id параметра для поиска параметра

  unsigned char parambuffer[64];
  msg.param_len = Settings::ReadOrWriteParameter(cat_id, param_id, parambuffer);
  const char* name = Settings::GetParameterName(cat_id, param_id, msg.param_name_len);

  unsigned long size = sizeof(msg) + msg.param_name_len + msg.param_len;
  msg.headers.CreateMessageHeaders(size, MESSAGES_ID::CategoriesParameters);
  msg.cat_id = cat_id;
  msg.param_id = param_id;
  msg.param_type_code = Settings::GetParameterDataType(cat_id, param_id);
  memcpy(msg.param_data, name, msg.param_name_len);
  memcpy(&msg.param_data[msg.param_name_len], &parambuffer, msg.param_len);

  SendMes(msg.headers.begin);
}
static void MissionCount(MessageMissionCount* request)
{
  ProtoMissionInfo.current_item = 0;
  ProtoMissionInfo.f_pause = false;
  ProtoMissionInfo.f_start = false;
  ProtoMissionInfo.total_items = request->count_items;
  ProtoMissionInfo.launch_confidentiality = request->launch_confidentiality;
  ProtoMissionInfo.mission_priority = request->mission_priority;
  ProtoMissionInfo.fault_tolerence = request->fault_tolerence;
  //memcpy(ProtoMissionInfo.mission_name, request->mission_name, sizeof(MessageMissionCount) - 11);
  //ProtoMissionInfo.mission_name[sizeof(MessageMissionCount) - 11] = '\0';
  ProtoMissionInfo.mission_name[0] = '\0';

  MessageMissionRequestItem msg;
  msg.headers.CreateMessageHeaders(sizeof(msg), MESSAGES_ID::MissionRequestItem);

  msg.item_number = 0;

  SendMes(msg.headers.begin);
}
static void MissionItem(MessageMissionItem* request)
{
  MissionItemData itemData = (MissionItemData)request->payload;
  ProtoMissionData.items[itemData.item_number].item_number = itemData.item_number;
  ProtoMissionData.items[itemData.item_number].item_command = itemData.item_command;
  ProtoMissionData.items[itemData.item_number].param1 = itemData.param1;
  ProtoMissionData.items[itemData.item_number].param2 = itemData.param2;
  ProtoMissionData.items[itemData.item_number].param3 = itemData.param3;
  ProtoMissionData.items[itemData.item_number].param4 = itemData.param4;
  ProtoMissionData.items[itemData.item_number].param5 = itemData.param5;
  ProtoMissionData.items[itemData.item_number].param6 = itemData.param6;
  ProtoMissionData.items[itemData.item_number].param7 = itemData.param7;

  MessageMissionItemAck msg1;
  msg1.headers.CreateMessageHeaders(sizeof(msg1), MESSAGES_ID::MissionItemAck);

  msg1.item_number = itemData.item_number;
  msg1.status = 1;

  SendMes(msg1.headers.begin);

  if (itemData.item_number == ProtoMissionInfo.total_items - 1)
  {
    // Получен послeдний элемент миссии. Что-то надо делать?
    unsigned char l = 0;
    for (int i = 0; i < ProtoMissionInfo.total_items; i++)
    {
      MissionItemData item = ProtoMissionData.items[i];
    }
  }
  else
  {
    MessageMissionRequestItem msg;
    msg.headers.CreateMessageHeaders(sizeof(msg), MESSAGES_ID::MissionRequestItem);

    msg.item_number = itemData.item_number + 1;

    SendMes(msg.headers.begin);
  }
}
static void ProcessingMessage()
{
  BeginHeader* headers = (BeginHeader*)(ProtoData);
  switch (headers->header.msgId)
  {
  case MESSAGES_ID::BatteryInfo: BatteryInfo(); return;
  case MESSAGES_ID::GyroInfo: GyroInfo(); return;
  case MESSAGES_ID::AccelInfo: AccelInfo(); return;
  case MESSAGES_ID::GpsInfo: GpsInfo(); return;
  case MESSAGES_ID::InertialInfo: InertialInfo(); return;
  case MESSAGES_ID::SysInfo: SysInfo(); return;
  case MESSAGES_ID::Command: ProcessingCommand((MessageCommand*)ProtoData); return;
  case MESSAGES_ID::ConnectionTest: ConnectionTest(); return;
  case MESSAGES_ID::StatusCommand: SendStatusCommand((MessageStatusCommand*)ProtoData); return;
  case MESSAGES_ID::CountMenuCategories: CountMenuCategories(); return;
  case MESSAGES_ID::CategoriesMenu: MenuCategories((MessageMenuCategories*)ProtoData); return;
  case MESSAGES_ID::CategoriesParameters: CategoriesParameters((MessageCategoriesParameters*)ProtoData); return;
  case MESSAGES_ID::MissionCount: MissionCount((MessageMissionCount*)ProtoData); return;
  case MESSAGES_ID::MissionItem: MissionItem((MessageMissionItem*)ProtoData); return;
  }
}