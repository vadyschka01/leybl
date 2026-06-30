#pragma once
#include "device/mags/IST8310.h"
#include "Med.h"

static void (*MAG_DoneProc)(MAG_Data& Data);

static MAG_Data MagResult;

#pragma pack(push,1)
struct MAG_DataBuffer { char s1; short mx; short my; short mz; };
#pragma pack(pop)

static bool Wait = false;

short MagMedBuf[3][5], MagMedTmp[3][5];

MED_Data16 MagMed[3] = { { 5, 0, MagMedBuf[0], MagMedTmp[0]}, { 5, 0, MagMedBuf[1], MagMedTmp[1]}, { 5, 0, MagMedBuf[2], MagMedTmp[2]} };

static void MAG_CallbackStart(unsigned char Address, const unsigned char* Data, unsigned char Size)
{

}

static void MAG_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  MAG_DataBuffer& data = *(MAG_DataBuffer*)Data;

  static MAG_DataBuffer testMag = *(MAG_DataBuffer*)Data;
  testMag = *(MAG_DataBuffer*)Data;

  if (data.s1 & 1)
  {
    data.mx = MED_Update(data.mx, MagMed[0]);
    data.my = MED_Update(data.my, MagMed[1]);
    data.mz = MED_Update(data.mz, MagMed[2]);

    MagResult.X = -Normalize(data.mx, 100.0f, ExternMagCalibData.X[0], ExternMagCalibData.X[1]);
    MagResult.Y = -Normalize(data.my, 100.0f, ExternMagCalibData.Y[0], ExternMagCalibData.Y[1]);
    MagResult.Z = Normalize(data.mz, 100.0f, ExternMagCalibData.Z[0], ExternMagCalibData.Z[1]);

    MagResult.RawX = data.mx;
    MagResult.RawY = data.my;
    MagResult.RawZ = data.mz;

    Wait = false;
  }

  MAG_DoneProc(MagResult);
}

static unsigned char MAG_BufferStart[4], MAG_BufferDevice[8];

static I2C_Request MAG_Start = { &MAG_CallbackStart, MAG_BufferStart, sizeof(MAG_BufferStart), 0 };

static I2C_Request MAG_Request = { &MAG_CallbackProc, MAG_BufferDevice, sizeof(MAG_BufferDevice), 0 };

IST8310::IST8310()
{

}

IST8310::~IST8310()
{

}

void IST8310::Init()
{
  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); } // ѕодождем готовности устройства к работе

  // Reset
  MAG_SetReg(Mag_Addr, 11U, 1U); // CNTL2 <- SRST

  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }

  // Config
  MAG_SetReg(Mag_Addr, 11U, 8U);        // CNTL2 <- DREN
  MAG_SetReg(Mag_Addr, 65U, 8U | 1U);   // AVGCNTL <- AVG_Y_AvgBy2 | AVG_XZ_AvgBy2
  MAG_SetReg(Mag_Addr, 66U, 192U);      // PDCNTL <- PULSE_DUR_Normal
}

bool IST8310::Get(MAG_Data& Data)
{
  if (!Wait) MAG_SetReg(Mag_Addr, 10U, 1U); // CNTL1 <- MODE_SingleMeasurement
  Wait = true;

  MAG_DataBuffer data;

  I2C2_Write(Mag_Addr, Mag_Read_Reg);
  I2C2_Read(Mag_Addr, &data, sizeof(MAG_DataBuffer));
  I2C2_Stop();

  if (!(data.s1 & 1)) return false;

  Data.X = data.mx;
  Data.Y = data.my;
  Data.Z = data.mz;

  Wait = false;

  return true;
}

void IST8310::GetAsync(void (*DoneProc)(MAG_Data& Data))
{
  if (!Wait)
  {
    unsigned char reg[] = { 10U, 1U };
    I2C2_Trans(&MAG_Start, Mag_Addr, reg, 2, 0);
    Wait = true;
    return;
  }

  MAG_DoneProc = DoneProc;
  I2C2_Trans(&MAG_Request, Mag_Addr, &Mag_Read_Reg, 1, sizeof(MAG_DataBuffer));
}