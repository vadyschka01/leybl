#pragma once
#include "INS/geom/Vector.h"
#include <math.h>

const static int SIZE_ROW = 5;

enum PARAMETERS : unsigned char
{
    POSITION_DIST = 0,
    ABS_ALT,
    REAL_ALT,
    HDOP,
    VDOP,
    PDOP,
    NOISE,
    SAT_VISIBLE,
    SAT_USED,
    COUNT // Всегда последний элемент = количеству
};

enum STATUS_ATTACK : unsigned char
{
  GPS_JAMMING = 0,
  GPS_SPOOFING_ASYNC,
  GPS_SPOOFING_SYNC,
  GPS_ANOMALY,
  WEATHER_ANOMALY,
  NORMAL,
  UNKNOWN,
  COUNT_ATTACK
};

enum EntropyLevels
{
  Level1 = 0,
  Level2,
  Level3,
  Level4,
  LevelsCount
};

struct Data_Analyser
{
  float values[PARAMETERS::COUNT];
};

struct ParametersRows
{
  Data_Analyser BufferOldValue[SIZE_ROW];
  Data_Analyser BufferNewValue[SIZE_ROW];
  bool needSlice = true;
  void Slice(unsigned char& StepSize, unsigned char* indx);
};

struct NormalRows
{
  float data[PARAMETERS::COUNT][SIZE_ROW];
};

#pragma pack(push,1)

struct EntropyMetrics
{
    float OldToNew; // KL(P || Q)
    float NewToOld; // KL(Q || P)
    float Sum;      // OldToNew + NewToOld
    unsigned char DetectedLevel = 0;
    unsigned long TimeDetect = 0;
};

struct AllEntropyMetrics 
{
  EntropyMetrics data[PARAMETERS::COUNT];
};

struct LevelsEntropy
{
  float values[EntropyLevels::LevelsCount];
};

struct CorrelationRule
{
  float level;
  bool general;
};

struct CorrelatetionRules
{
  CorrelationRule value[STATUS_ATTACK::COUNT_ATTACK][PARAMETERS::COUNT];
};

#pragma pack(pop)

class Analyser
{
public:
  Analyser();
  
  STATUS_ATTACK lastStatus = STATUS_ATTACK::NORMAL;
  unsigned long TimeDetect = 0;
  
  ParametersRows RawDataRows;
  
  NormalRows OldNormalRows;
  NormalRows NewNormalRow;
  
  LevelsEntropy Levels;
  
  AllEntropyMetrics EntropyStorage;
  
  CorrelatetionRules Rules;
  
  bool update = false;
  
  STATUS_ATTACK process(Data_Analyser* data, unsigned long Time);
  
private:
  
  unsigned char indOldRow = 0;
  unsigned char indNewRow = 0;
  unsigned char StepSize = 1;
  
  unsigned int TimeLiveLevelDetect = 60; // Время жизни метки детектирования в секундах
  
  bool firstNormalize = true;
  
  void CalcPoissonPMFForArray(float* input, float* output, unsigned char size);
  
  void NormalizeData(Data_Analyser* dataArray, NormalRows* normalRow);
  
  float CalculateKL(float* p, float* q, unsigned char size);
  
  float CalculateJSD(float* p, float* q, unsigned char size);
  
  void CalculateFieldMetrics(float* p, float* q, unsigned char size, EntropyMetrics* out, unsigned long* Time);
  
  void CalculateAllEntropyMetrics(NormalRows* oldProb, NormalRows* newProb, AllEntropyMetrics* outMetrics, unsigned long* Time);
  
  float LnFactorial(float k);
  
  void InitRules();
};
