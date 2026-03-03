#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "gpio.h"

#include "uart.h"
#include "tick.h"

#include "gps.h"

void GPS_LocalDistance(Point p1, Point p2, float& dx, float& dy, float& dz) // light formula
{
  const float pi = 3.14159265359f;
  const float er = 6371000.0f; // Radius of the earth in m

  float lat = p1.Latitude - p2.Latitude;
  float lon = p1.Longitude - p2.Longitude;

  float y = er/360.0f * (lat*pi*2.0f); // lat
  dy = y;

  float l = ((float)(p1.Latitude + p2.Latitude)) / 2.0f;
  float r = cosf(l*pi/180.0f) * er;
  float x = r/360.0f * (lon*pi*2.0f); // long
  dx = x;
  dz = p1.Altitude - p2.Altitude;
}
//------------------------------------------------------------------------------

float GPS_GlobalDistance(Point p1, Point p2) // Haversine formula
{
  const float pi = 3.14159265359f;
  const float er = 6371000.0f; // Radius of the earth in m

  float dLat = ((float)(p2.Latitude - p1.Latitude))*pi/180.0f;
  float dLon = ((float)(p2.Longitude - p1.Longitude))*pi/180.0f;

  float lat1 = p1.Latitude, lat2 = p2.Latitude;

  float sdlat=sinf(dLat/2.0f);
  float sdlon=sinf(dLon/2.0f);
  
  float a = (sdlat*sdlat) + cosf((lat1)*pi/180.0f) * cosf((lat2)*pi/180.0f) * (sdlon*sdlon);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f-a));
  float d = er * c; // Distance in m
  return d;
}
//------------------------------------------------------------------------------

struct GPS_Head
{
  static const unsigned long Size = 7;
  const char* NMEA;
};

struct GPS_GNRMC // $GNRMC,hhmmss.sss,A,ggmm.mm,P,gggmm.mm,J,v.v,b.b,ddmmyy,x.x,n,m*hh/r/n
{
  static constexpr GPS_Head Head = {"$GNRMC,"}; // Recommended minimum specific Transit data
  //---
  char* Time;      // UTC of position fix
  char* Valid;     // Data status (V=navigation receiver warning)
  char* Latitude;  // Latitude of fix
  char* Pole;      // N or S
  char* Longitude; // Longitude of fix
  char* J;         // E or W
  char* HorSpeed;  // Speed over ground in knots
  char* Angle;     // Track made good in degrees True
  char* Date;      // UT date
  char* Mag;       // Magnetic variation degrees (Easterly var. subtracts from true course)
  char* MagAng;    // Magnetic declination direction, E (east) or W (west)
  char* Mode;      // Mode indication (A=autonomous positioning, D=differential, E=estimation, N=invalid data)
  //---
  // Checksum
};

struct GPS_GNGGA // $GNGGA,hhmmss.ss,ggmm.mm,a,gggmm.mm,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh/r/n
{
  static constexpr GPS_Head Head = {"$GNGGA,"}; // Global Positioning System Fix Data
  //---
  char* Time;          // UTC of position
  char* Latitude;      // latitude of position
  char* Pole;          // N or S
  char* Longitude;     // Longitude of position
  char* J;             // E or W
  char* Quality;       // GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
  char* Satellites;    // Number of satellites in use [not those in view]
  char* HDOP;          // Horizontal dilution of position
  char* Altitude;      // Antenna altitude above/below mean sea level (geoid)
  char* UnitsAlt;      // Meters  (Antenna height unit)
  char* Geoidal;       // Geoidal separation (Diff. between WGS-84 earth ellipsoid andmean sea level.  -=geoid is below WGS-84 ellipsoid)
  char* UnitsGeoidal;  // Meters  (Units of geoidal separation)
  char* LastUpdate;    // Age in seconds since last update from diff. reference station
  char* StationID;     // Diff. reference station ID#
  //---
  // Checksum
};

struct GPS_GNGSA // $GNGSA,mode,fix,PRN(11),PDOP,HDOP,VDOP,*hh/r/n
{
  static constexpr GPS_Head Head = {"$GNGSA,"}; // Global Positioning System Fix Data
  //---
  char* Mode;    // M = Manual, A = Automatic
  char* FixType; // 1 = not available, 2 = 2D, 3 = 3D
  char* ID_1;    // 01 to 32 for GPS, 33 to 64 for SBAS, 64+ for GLONASS
  char* ID_2;
  char* ID_3;
  char* ID_4;
  char* ID_5;
  char* ID_6;
  char* ID_7;
  char* ID_8;
  char* ID_9;
  char* ID_10;
  char* ID_11;
  char* ID_12;
  char* PDOP;
  char* HDOP;
  char* VDOP;
  //---
  // Checksum
};

struct GPS_GPGSV // $GPGSV,count,index,visible,[ID,elevation,azimuth,SNR]*hh/r/n
{
  static constexpr GPS_Head Head = {"$GPGSV,"}; // Global Positioning System Fix Data
  //---
  char* Count;    // M = Manual, A = Automatic
  char* Index;   // 1 = not available, 2 = 2D, 3 = 3D
  char* Visible; // 01 to 32 for GPS, 33 to 64 for SBAS, 64+ for GLONASS
  
  char* ID_1;
  char* Elevation_1;
  char* Azimuth_1;
  char* SNR_1;
  
  char* ID_2;
  char* Elevation_2;
  char* Azimuth_2;
  char* SNR_2;
  
  char* ID_3;
  char* Elevation_3;
  char* Azimuth_3;
  char* SNR_3;
  
  char* ID_4;
  char* Elevation_4;
  char* Azimuth_4;
  char* SNR_4;
  
  //---
  // Checksum
};

struct GPS_GLGSV // $GLGSV,count,index,visible,[ID,elevation,azimuth,SNR]*hh/r/n
{
  static constexpr GPS_Head Head = {"$GLGSV,"}; // Global Positioning System Fix Data
  //---
  char* Count;    // M = Manual, A = Automatic
  char* Index;   // 1 = not available, 2 = 2D, 3 = 3D
  char* Visible; // 01 to 32 for GPS, 33 to 64 for SBAS, 64+ for GLONASS
  
  char* ID_1;
  char* Elevation_1;
  char* Azimuth_1;
  char* SNR_1;
  
  char* ID_2;
  char* Elevation_2;
  char* Azimuth_2;
  char* SNR_2;
  
  char* ID_3;
  char* Elevation_3;
  char* Azimuth_3;
  char* SNR_3;
  
  char* ID_4;
  char* Elevation_4;
  char* Azimuth_4;
  char* SNR_4;
  
  //---
  // Checksum
};

enum class GPS_PROTO {GNRMC, GNGGA, GNGSA, GPGSV, GLGSV, COUNT};
static const long GPS_ProtCount = (long)GPS_PROTO::COUNT;
static GPS_Head GPS_Protocols[GPS_ProtCount] = { GPS_GNRMC::Head, GPS_GNGGA::Head, GPS_GNGSA::Head, GPS_GPGSV::Head, GPS_GLGSV::Head };

struct GPS_Info
{
  GPS_PROTO Type;

  static const unsigned long Length = 256;
  char Data[Length];

  static const unsigned long Count = 32;
  char* Param[Count];

  long Begin = 0;
  long Index = 0;
  long Size = 0;

  unsigned char CRC8;
};


static GPS_Info Info;

void (*GPS_CallBack)(GPS_PROTO Proto, void* Data)=0;

//------------------------------------------------------------------------------------------------------------------------------
/*static unsigned long UARTt_Recv(void* Data, unsigned long Size, bool WaitAll=false) // For Check ONLY !!!!!!!!!!!!!!!!
{
  static int begin=0;
  
  const char test[]="$GNGGA,122013.10,4719.45110,N,03846.54105,E,1,12,0.66,35.9,M,17.6,M,,*7A\r\n";
  unsigned long size=strlen(test);
  
  memcpy(Data, test, Size<size ? Size : size);
  //memcpy(Data, test+begin, 1);
  
  begin++;
  
  if(begin>=size) begin=0;
  
  //return 1;
  return size;
}*/
//------------------------------------------------------------------------------------------------------------------------------

bool GPS_Update()
{
  char* head = Info.Data;

  Info.Size += UART1_Recv(head + Info.Size, Info.Length - Info.Size);

  long size = Info.Size - Info.Index;

  while (size)
  {
    char* data = head + Info.Index;
  
    if (Info.Index && *data == '$')
    {
      memcpy(head, data, size);
      Info.Index = 0;
      Info.Begin = 0;
      Info.Size = size;
      data = head;
    }
  
    if (!Info.Begin)
    {
      if (*head != '$')
      {
        Info.Index++;
        size--;
        continue;
      }
      //---
      if (Info.Size < GPS_Head::Size)  return true;
      //---
      Info.Type = GPS_PROTO::COUNT;
      for (long a = 0; a < GPS_ProtCount; a++)
      {
        const char* nmea = GPS_Protocols[a].NMEA;
        if (memcmp(head, nmea, GPS_Head::Size) == 0)
        {
          Info.CRC8 = 0x0E;
          for (long a = 0; nmea[a]; a++) Info.CRC8 ^= nmea[a];
          Info.Type = (GPS_PROTO)a;
          break;
        }
      }
      //---
      if (Info.Type == GPS_PROTO::COUNT)
      {
        Info.Index++;
        *head = '#';
        size--;
        continue;
      }
      //---
      Info.Index = GPS_Head::Size;
      size = Info.Size - GPS_Head::Size;
      //---
      Info.Param[Info.Begin++] = head + Info.Index;
      //---
      continue;
    }
    //---
    if (Info.Size > Info.Length)
    {
      Info.Begin = 0;
      Info.Index = 0;
      Info.Size = 0;
      return true;
    }
    //---
    if (Info.Begin >= Info.Count)
    {
      if (*data == '\n')
      {
        *data = '\0';
        size--;
        char* end;
        unsigned char crc8 = strtol(data-3, &end, 16);
        if ((end == data - 1) && (crc8 == Info.CRC8))
        {
          if (GPS_CallBack) GPS_CallBack(Info.Type, Info.Param);
          //---
          if (size) memcpy(Info.Data, data + 1, size);
        }
        //---
        Info.Begin = 0;
        Info.Index = 0;
        Info.Size = size;
        //---
        continue;
      }
    }
    else
    {
      Info.CRC8 ^= *data;
      //---
      if (*data == ',')
      {
        *data = '\0';
        Info.Param[Info.Begin++] = ++data;
      }
      else if (*data == '*')
      {
        *data++ = '\0';
        while (Info.Begin < Info.Count)
        {
          Info.Param[Info.Begin++] = nullptr;
        }
      }
    }
    //---
    Info.Index++;
    size--;
  }
  
  if (!Info.Begin && Info.Index)
  {
    Info.Index = 0;
    Info.Size = 0;
  }
  
  return true;
}
//------------------------------------------------------------------------------

static unsigned char GPS_VisibleGPS=0;
static unsigned char GPS_VisibleGLO=0;

static unsigned char Base_Noise[256];

Point GetCoordinates_Coord;
bool GetCoordinates_Valid;
bool GetCoordinates_Update;
bool GetBaseInfo_Update;

Point Base_BeginXYZ;

static float Base_Alt=0, Base_Geo=0;
static unsigned char Base_Used, Base_Fix;

static GPS_BaseInfo GPS_BaseInfoData;

static void GPS_CallBack_GSV(GPS_PROTO Proto, void* Data)
{
  if(Proto==GPS_PROTO::GNGGA)
  {
    GPS_GNGGA* data=(GPS_GNGGA*)Data;
    
    GetCoordinates_Valid = (data->Quality[0] && (data->Quality[0]=='1' || data->Quality[0]=='2'));
    
    float geo=0;
    if(data->Geoidal[0] && GetCoordinates_Valid) geo=strtof(data->Geoidal, 0);
    
    if(data->Altitude[0]) GetCoordinates_Coord.Altitude=strtof(data->Altitude, 0);
    //---
    Base_Geo=GetCoordinates_Coord.Altitude+geo;
    
    
    Base_Alt=Base_Geo-Base_BeginXYZ.Altitude;
    //---
    GetCoordinates_Coord.Altitude+=geo;
    
    if(data->Latitude[0] && GetCoordinates_Valid)
    {
      GetCoordinates_Coord.Latitude=(data->Latitude[0]-'0')*10 + data->Latitude[1]-'0';
      GetCoordinates_Coord.Latitude+=strtod(data->Latitude+2, 0)/60;
    }
    //---
    if(data->Longitude[0] && GetCoordinates_Valid)
    {
      GetCoordinates_Coord.Longitude=(data->Longitude[0]-'0')*100 + (data->Longitude[1]-'0')*10 + data->Longitude[2]-'0';
      GetCoordinates_Coord.Longitude+=strtod(data->Longitude+3, 0)/60;
    }
    
    if(data->Satellites[0] && GetCoordinates_Valid) Base_Used=strtoul(data->Satellites, 0, 10);
    
    if(data->Quality[0] && GetCoordinates_Valid) Base_Fix=strtoul(data->Quality, 0, 10);
    
    GetBaseInfo_Update = true;
    GetCoordinates_Update=true;
  }
  
  if(Proto==GPS_PROTO::GPGSV)
  {
    GPS_GPGSV* data=(GPS_GPGSV*)Data;
    GPS_VisibleGPS = strtoul(data->Visible, 0, 10);
    
    if(!data->Index || *data->Index=='1') memset(Base_Noise, 0, sizeof(Base_Noise));
    
    long id;
    
    if(data->ID_1)
    {
      id=strtoul(data->ID_1, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_1, 0, 10);
    }
    if(data->ID_2)
    {
      id=strtoul(data->ID_2, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_2, 0, 10);
    }
    if(data->ID_3)
    {
      id=strtoul(data->ID_3, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_3, 0, 10);
    }
    if(data->ID_4)
    {
      id=strtoul(data->ID_4, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_4, 0, 10);
    }
  }
  else if(Proto==GPS_PROTO::GLGSV)
  {
    GPS_GLGSV* data=(GPS_GLGSV*)Data;
    GPS_VisibleGLO = strtoul(data->Visible, 0, 10);
    
    long id;
    
    if(data->ID_1)
    {
      id=strtoul(data->ID_1, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_1, 0, 10);
    }
    if(data->ID_2)
    {
      id=strtoul(data->ID_2, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_2, 0, 10);
    }
    if(data->ID_3)
    {
      id=strtoul(data->ID_3, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_3, 0, 10);
    }
    if(data->ID_4)
    {
      id=strtoul(data->ID_4, 0, 10);
      if(id<256) Base_Noise[id]=strtoul(data->SNR_4, 0, 10);
    }
    
    if(*data->Count==*data->Index)
    {
      float noise=0, count=0;
      for(long a=0; a<256; a++)
      {
        if(!Base_Noise[a]) continue;
        noise+=Base_Noise[a];
        count++;  
      }
      
      if(count) GPS_BaseInfoData.noise=noise/count;
      else GPS_BaseInfoData.noise=0;
      
      GPS_BaseInfoData.satVisible=GPS_VisibleGPS+GPS_VisibleGLO;
    }
  }
  else if(Proto==GPS_PROTO::GNRMC)
  {
    GPS_GNRMC* data=(GPS_GNRMC*)Data;
    
    GPS_BaseInfoData.lat=GetCoordinates_Coord.Latitude;
    GPS_BaseInfoData.lon=GetCoordinates_Coord.Longitude;
    
    if(data->HorSpeed[0]) GPS_BaseInfoData.speed=strtof(data->HorSpeed, 0);
    
    if(data->Time[0]) GPS_BaseInfoData.timeUTC=strtof(data->Time, 0);
    
    float dx, dy, dz;
    GPS_LocalDistance(Base_BeginXYZ, {GPS_BaseInfoData.lat, GPS_BaseInfoData.lon}, dx, dy, dz);
                      
    GPS_BaseInfoData.LocalXYZ[0]=dx;
    GPS_BaseInfoData.LocalXYZ[1]=-dy;
    GPS_BaseInfoData.LocalXYZ[2]=Base_Geo-Base_BeginXYZ.Altitude;
  }
  else if(Proto==GPS_PROTO::GNGSA)
  {
    GPS_GNGSA* data=(GPS_GNGSA*)Data;
    
    if(data->HDOP[0]) GPS_BaseInfoData.hdop=strtof(data->HDOP, 0);
    if(data->VDOP[0]) GPS_BaseInfoData.vdop=strtof(data->VDOP, 0);
    if(data->PDOP[0]) GPS_BaseInfoData.pdop=strtof(data->PDOP, 0);
  }
}
//------------------------------------------------------------------------------

void GPS_Init()
{
  UART1_Init(460800);
  GPS_CallBack=GPS_CallBack_GSV;
  memset(Base_Noise, 0, sizeof(Base_Noise));
}

bool GPS_GetCoordinates(Point& Coord, bool& Valid)
{
  if(!GetCoordinates_Update) return false;
  
  GetCoordinates_Update=false;
  
  Coord=GetCoordinates_Coord;
  Valid=GetCoordinates_Valid;
  
  return true;
}
//------------------------------------------------------------------------------

bool GPS_GetBaseInfo(GPS_BaseInfo& Info)
{
  if (!GetBaseInfo_Update) return false;
  GetBaseInfo_Update = false;
  
  Info=GPS_BaseInfoData;
  
  Info.realAlt=Base_Alt;
  Info.absAlt=Base_Geo;
  Info.satUsed=Base_Used;
  Info.fixType=Base_Fix;
  
  return true;
}
//------------------------------------------------------------------------------
