#pragma once

struct I2C_Request
{
  void (*CallbackProc)(unsigned char Address, const unsigned char* Data, unsigned char Size);
  unsigned char* Buffer;
  unsigned char Size;
  
  unsigned char Address;
  unsigned char Write;
  unsigned char Read;
  
  I2C_Request* Next;
};

void I2C2_Init();
bool I2C2_Trans(I2C_Request* Request, unsigned char Address, const void* Data, unsigned char SizeWrite, unsigned char SizeRead);
void I2C2_Write(unsigned char Address, unsigned char Data);
void I2C2_Write(unsigned char Address, const void* Data, unsigned char Size);
void I2C2_Read(unsigned char Address, void* Data, unsigned char Size);
bool I2C2_CheckDevice(unsigned char Address);
bool I2C2_CheckDeviceWhoAmI(unsigned char Address, unsigned char WhoAmI_Reg, unsigned char Expected_ID);
void I2C2_Stop();

void I2C1_Init();
bool I2C1_Trans(I2C_Request* Request, unsigned char Address, const void* Data, unsigned char SizeWrite, unsigned char SizeRead);
void I2C1_Write(unsigned char Address, unsigned char Data);
void I2C1_Write(unsigned char Address, const void* Data, unsigned char Size);
void I2C1_Write2(unsigned char Address, const void* Data1, unsigned char Size1, const void* Data2, unsigned char Size2);
void I2C1_Read(unsigned char Address, void* Data, unsigned char Size);
bool I2C1_CheckDevice(unsigned char Address);
bool I2C1_CheckDeviceWhoAmI(unsigned char Address, unsigned char WhoAmI_Reg, unsigned char Expected_ID);
void I2C1_Stop();
