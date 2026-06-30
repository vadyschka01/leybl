#include <device/bars/BMP280.h>
#include <math.h>

#define NORMAL_MODE    0x03
#define FORCED_MODE    0x02

#define STANDBY_500US  0x00
#define STANDBY_10MS   0x06
#define STANDBY_20MS   0x07
#define STANDBY_6250US 0x01
#define STANDBY_125MS  0x02
#define STANDBY_250MS  0x03
#define STANDBY_500MS  0x04
#define STANDBY_1000MS 0x05

#define MODULE_DISABLE	0x00
#define OVERSAMPLING_1  0x01
#define OVERSAMPLING_2  0x02
#define OVERSAMPLING_4  0x03
#define OVERSAMPLING_8  0x04
#define OVERSAMPLING_16 0x05

#define FILTER_DISABLE  0x00
#define FILTER_COEF_2   0x01
#define FILTER_COEF_4   0x02
#define FILTER_COEF_8   0x03
#define FILTER_COEF_16  0x04

static BMP280_CalibrationData CalibrationData;

static void CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  BAR_Data result;

  const unsigned char* buff = Data;

  long temp_raw = 0;
  unsigned long press_raw = 0;
  press_raw = (((unsigned long)buff[0]) << 16) | (((unsigned long)buff[1]) << 8) | ((unsigned long)buff[2]);
  temp_raw = (((unsigned long)buff[3]) << 16) | (((unsigned long)buff[4]) << 8) | ((unsigned long)buff[5]);

  if (press_raw == 0x800000 || temp_raw == 0x800000) { BAR_DoneProc(false, result); return; }

  temp_raw >>= 4;
  long value_t1 = ((((temp_raw >> 3) - ((long)CalibrationData._T1 << 1))) * ((long)CalibrationData._T2)) >> 11;
  long value_t2 = (((((temp_raw >> 4) - ((long)CalibrationData._T1)) * ((temp_raw >> 4) - ((long)CalibrationData._T1))) >> 12) * ((long)CalibrationData._T3)) >> 14;

  long temp = value_t1 + value_t2;

  long T = (temp * 5 + 128) >> 8;
  result.Temp = T / 100;

  press_raw >>= 4;
  long long value_1 = ((long long)temp) - 128000;
  long long value_2 = value_1 * value_1 * (long long)CalibrationData._P6;
  value_2 = value_2 + ((value_1 * (long long)CalibrationData._P5) << 17);
  value_2 = value_2 + (((long long)CalibrationData._P4) << 35);
  value_1 = ((value_1 * value_1 * (long long)CalibrationData._P3) >> 8) + ((value_1 * (long long)CalibrationData._P2) << 12);
  value_1 = (((((long long)1) << 47) + value_1)) * ((long long)CalibrationData._P1) >> 33;

  if (!value_1) { BAR_DoneProc(false, result); return; }

  long long p = 1048576 - press_raw;
  p = (((p << 31) - value_2) * 3125) / value_1;
  value_1 = (((long long)CalibrationData._P9) * (p >> 13) * (p >> 13)) >> 25;
  value_2 = (((long long)CalibrationData._P8) * p) >> 19;
  p = ((p + value_1 + value_2) >> 8) + (((long long)CalibrationData._P7) << 4);

  result.Pressure = p / 256.0f;

  BAR_DoneProc(true, result);
}
static unsigned char BAR_Buffer[32];
static I2C_Request BAR_Device = { &CallbackProc, BAR_Buffer, sizeof(BAR_Buffer), 0 };

BMP280::BMP280()
{

}

BMP280::~BMP280()
{
  
}

void BMP280::GetCalibration()
{
  I2C1_Write(Addr, 0x88);
  I2C1_Read(Addr, &CalibrationData, sizeof(CalibrationData));
  I2C1_Stop();
}

void BMP280::Init()
{
  BAR_SetReg(Addr, 0xE0, 0xB6); // RESET

  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }

  GetCalibration();

  BAR_SetReg(Addr, 0xF4, ((OVERSAMPLING_1 << 5) | (OVERSAMPLING_4 << 2) | NORMAL_MODE));
  BAR_SetReg(Addr, 0xF5, ((STANDBY_500US << 5) | (FILTER_DISABLE << 2)));
}

float BMP280::GetAltitude(float zero, float press, float temp)
{
  return (8.314f * (273.15f + temp) / (0.0289647f * 9.81f)) * logf(zero / press);
}

void BMP280::GetAsync(void (*DoneProc)(bool Ready, BAR_Data& Data))
{
  BAR_DoneProc = DoneProc;
  I2C1_Trans(&BAR_Device, Addr, &RegRead, 1, 6);
}

long BMP280::GetData(long* Temp)
{
  unsigned char buff[6];
  I2C1_Write(Addr, RegRead);
  I2C1_Read(Addr, buff, sizeof(buff));
  I2C1_Stop();

  long temp_raw = 0;
  unsigned long press_raw = 0;
  press_raw = (((unsigned long)buff[0]) << 16) | (((unsigned long)buff[1]) << 8) | ((unsigned long)buff[2]);
  temp_raw = (((unsigned long)buff[3]) << 16) | (((unsigned long)buff[4]) << 8) | ((unsigned long)buff[5]);

  if (press_raw == 0x800000 || temp_raw == 0x800000) return 0;

  temp_raw >>= 4;
  long value_t1 = ((((temp_raw >> 3) - ((long)CalibrationData._T1 << 1))) * ((long)CalibrationData._T2)) >> 11;
  long value_t2 = (((((temp_raw >> 4) - ((long)CalibrationData._T1)) * ((temp_raw >> 4) - ((long)CalibrationData._T1))) >> 12) * ((long)CalibrationData._T3)) >> 14;

  long temp = value_t1 + value_t2;

  if (Temp)
  {
    long T = (temp * 5 + 128) >> 8;
    *Temp = T / 100;
  }

  press_raw >>= 4;
  long long value_1 = ((long long)temp) - 128000;
  long long value_2 = value_1 * value_1 * (long long)CalibrationData._P6;
  value_2 = value_2 + ((value_1 * (long long)CalibrationData._P5) << 17);
  value_2 = value_2 + (((long long)CalibrationData._P4) << 35);
  value_1 = ((value_1 * value_1 * (long long)CalibrationData._P3) >> 8) + ((value_1 * (long long)CalibrationData._P2) << 12);
  value_1 = (((((long long)1) << 47) + value_1)) * ((long long)CalibrationData._P1) >> 33;

  if (!value_1) return 0;

  long long p = 1048576 - press_raw;
  p = (((p << 31) - value_2) * 3125) / value_1;
  value_1 = (((long long)CalibrationData._P9) * (p >> 13) * (p >> 13)) >> 25;
  value_2 = (((long long)CalibrationData._P8) * p) >> 19;
  p = ((p + value_1 + value_2) >> 8) + (((long long)CalibrationData._P7) << 4);

  return p / 256;
}