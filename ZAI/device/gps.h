#pragma once

struct Point { double Latitude; double Longitude; float Altitude; };

extern Point Base_BeginXYZ;

void GPS_LocalDistance(Point p1, Point p2, float& dx, float& dy, float& dz); // light formula
float GPS_GlobalDistance(Point p1, Point p2); // Haversine formula

void GPS_Init();
bool GPS_Update();
bool GPS_GetCoordinates(Point& Coord, bool& Valid);

struct GPS_BaseInfo
{
  float LocalXYZ[3];
  float lat; 
  float lon; 
  float absAlt; 
  float realAlt; 
  float hdop; 
  float vdop; 
  float pdop; 
  float noise; 
  float jamming; //
  unsigned char satVisible; 
  unsigned char satUsed; 
  float speed;  
  unsigned char fixType; // NO_GPS - 0, NO_FIX - 1, 2D_FIX - 2, 3D_FIX - 3, DGPS - 4, RTK_FLOAT - 5, RTK_FIXED - 6, STATIC - 7, PPP - 8
  unsigned long long timeUTC;
};

bool GPS_GetBaseInfo(GPS_BaseInfo& Info);
