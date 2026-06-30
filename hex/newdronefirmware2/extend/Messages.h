#pragma once

#include "Protocol.h"
#define MAX_PACKET_LEN 44

#pragma pack(push,1)

const unsigned short MaxLength = 1024;
const unsigned char Global_stx = 0xAA;

//struct HeaderBegin
//{
//  unsigned char stx;
//  unsigned short len1; //Len header + payload (message)  ПОТОМ ДОЛЖНО БЫТЬ РАЗМЕРОМ ВСЕГО СООБЩЕНИЯ BEGIN+HEADER+PAYLOAD
//  unsigned short len2; //Len header + payload (message)
//  unsigned char crc; // CRC MESSAGE
  //---
//  bool CheckCRC();
//  void SetCRC();
//};

struct HeaderMessages
{
  unsigned char stx;
  MESSAGES_ID msgId;
  unsigned char Length;
  //unsigned char srcId;
  //unsigned char dstId;
  //unsigned char len;
  unsigned long timeusec;
};
struct BeginHeader
{
  HeaderMessages header;
  void CreateMessageHeaders(unsigned long size, MESSAGES_ID msg_id);
};

struct MessageBatteryInfo
{
  BeginHeader headers;

  ProtoDataBattery payload;
};
struct MessageSysInfo
{
  BeginHeader headers;

  ProtoDataSysInfo payload;

};
struct MessageGyroInfo
{
  BeginHeader headers;

  ProtoDataGyroInfo payload;
};
struct MessageAccelInfo
{
  BeginHeader headers;

  ProtoDataAccelInfo payload;
};
struct MessageGpsInfo
{
  BeginHeader headers;

  ProtoDataGpsInfo payload;
};
struct MessageInertialInfo
{
  BeginHeader headers;

  ProtoDataInertialInfo payload;
};
struct MessageRawCalibData
{
  BeginHeader headers;
  
  ProtoDataRawCalib payload;
};
struct MessageMissionProgress
{
  BeginHeader headers;

  unsigned long long current_item;
  unsigned long long total_items;
  bool f_start;
  bool f_pause;
};
struct MessageCommandAck
{
  BeginHeader headers;

  COMMANDS_ID commandId;
  FEASIBILITY_CODE_COMMAND status; // 0 - не может быть выполнена (указывается код ошибки), 1 - принята
  ERROR_CODE_COMMAND errorCode;
};
struct MessageStatusCommand
{
  BeginHeader headers;

  COMMANDS_ID commandId;
  EXEC_CODE_COMMAND executionCode; // 0 - ошибка при выполнении ,1 - выполняется ,2 - выполнена успешно.
  ERROR_CODE_COMMAND errorCode;
};

struct MessageCommand
{
  BeginHeader headers;
  COMMANDS_ID commandId;
};
struct MessageCommandChangeSpeed
{
  MessageCommand commandInfo;

  float speed;
};
struct MessageCommandChangeNav
{
  MessageCommand commandInfo;

  NAV_SYS_MODE nav_id; // Inertial - 1, InfoGPS = 2, Oprical_Flow = 3
};
struct MessageCommandGoToGlobal
{
  MessageCommand commandInfo;

  double lat;
  double lon;
  float abs_alt;
  float speed;
};
struct MessageCommandGoToLocal
{
  MessageCommand commandInfo;

  float x;
  float y;
  float z;
  float speed;
};
struct MessageCommandSetParameter
{
  MessageCommand commandInfo;

  unsigned char cat_id;
  unsigned char param_id;
  DATA_TYPES param_type_code;
  unsigned char param_len;
  unsigned char param_data[MAX_PACKET_LEN];
};
struct MessageCommandProcessingCalibration
{
  MessageCommand commandInfo;
 
  SENSOR_ID sensor_id;
  bool start;
};
struct MessageCommandSetCalibrationData
{
  MessageCommand commandInfo;
  
  SENSOR_ID sensor_id;
  float X[2];
  float Y[2];
  float Z[2];
};
struct MessageCommandMotorMoutionTest
{
  MessageCommand commandInfo;
  
  unsigned char motor_id;
};
struct MessageCountMenuCategories
{
  BeginHeader headers;

  unsigned char countCategories;
  unsigned char data[MAX_PACKET_LEN];
};
struct MenuCategori
{
  unsigned char id;
  unsigned char len_name;
  unsigned char count_param;
  char name[0];
};
struct MessageMenuCategories
{
  BeginHeader headers;

  unsigned char countCategories;
  unsigned char categories[MAX_PACKET_LEN];
};
struct MessageCategoriesParameters
{
  BeginHeader headers;

  unsigned char cat_id;
  unsigned char param_id;
  unsigned char param_name_len;
  unsigned char param_len;
  DATA_TYPES param_type_code;
  char param_data[MAX_PACKET_LEN];
};

struct MessageMissionCount
{
  BeginHeader headers;

  unsigned long long count_items;
  bool launch_confidentiality;
  bool mission_priority;
  bool fault_tolerence;
  unsigned char mission_name_len;
  char mission_name[MAX_PACKET_LEN];
};
struct MessageMissionItem
{
  BeginHeader headers;

  MissionItemData payload;
};
struct MessageMissionRequestItem
{
  BeginHeader headers;

  unsigned long long item_number;
};
struct MessageMissionItemAck
{
  BeginHeader headers;

  unsigned long long item_number;
  bool status; // 0 - не получен. 1 - получен.
};
struct MessageDeviceStatuses
{
  BeginHeader headers;
  
  bool ImuCalib;
  bool MagCalib;
  bool GpsInit;
  bool LidarInit;
  bool OpticalFlowInit;
  bool CamFlowInit;
};

#pragma pack(pop)