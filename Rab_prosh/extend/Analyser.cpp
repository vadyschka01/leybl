#include "Analyser.h"

Analyser::Analyser()
{
  memset(&OldNormalRows, 0, sizeof(NormalRows));
  memset(&EntropyStorage, 0, sizeof(AllEntropyMetrics));
  
  Levels = {0.0f, 0.04f, 0.1f, 1.0f};
  InitRules();
};

void Analyser::InitRules()
{
  { // GPS_JAMMING
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::POSITION_DIST] = {Levels.values[EntropyLevels::Level3], false};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::ABS_ALT] = {Levels.values[EntropyLevels::Level4], false};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::REAL_ALT] = {Levels.values[EntropyLevels::Level4], false};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_JAMMING][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level4], true};
  }
  { // GPS_SPOOFING_ASYNC
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::POSITION_DIST] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::ABS_ALT] = {Levels.values[EntropyLevels::Level3], false};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::REAL_ALT] = {Levels.values[EntropyLevels::Level3], false};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level4], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_ASYNC][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level4], true};
  }
  { // GPS_SPOOFING_SYNC
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::POSITION_DIST] = {Levels.values[EntropyLevels::Level2], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::ABS_ALT] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::REAL_ALT] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level3], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level2], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level2], true};
    Rules.value[STATUS_ATTACK::GPS_SPOOFING_SYNC][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level2], true};
  }
  { // GPS_ANOMALY
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::POSITION_DIST] = {-1.0f, false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::ABS_ALT] = {-1.0f, false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::REAL_ALT] = {-1.0f, false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level2], true};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::GPS_ANOMALY][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level2], false};
  }
  { // WEATHER_ANOMALY
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::POSITION_DIST] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::ABS_ALT] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::REAL_ALT] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level2], false};
    Rules.value[STATUS_ATTACK::WEATHER_ANOMALY][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level2], false};
  }
  { // NORMAL
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::POSITION_DIST] = {Levels.values[EntropyLevels::Level1], false};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::ABS_ALT] = {Levels.values[EntropyLevels::Level1], false};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::REAL_ALT] = {Levels.values[EntropyLevels::Level1], false};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::HDOP] = {Levels.values[EntropyLevels::Level1], true};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::VDOP] = {Levels.values[EntropyLevels::Level1], true};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::PDOP] = {Levels.values[EntropyLevels::Level1], true};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::NOISE] = {Levels.values[EntropyLevels::Level1], true};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::SAT_USED] = {Levels.values[EntropyLevels::Level1], true};
    Rules.value[STATUS_ATTACK::NORMAL][PARAMETERS::SAT_VISIBLE] = {Levels.values[EntropyLevels::Level1], true};
  }
  
};

void ParametersRows::Slice(unsigned char& StepSize, unsigned char* indx)
{
  needSlice = false;
  if (SIZE_ROW < StepSize) return; 
  if (SIZE_ROW == StepSize) 
  {
    for (unsigned char i = 0; i < SIZE_ROW; i++) BufferNewValue[i] = BufferOldValue[i]; 
    *indx = 0;
    return;
  }
  unsigned char elementsToKeep = SIZE_ROW - StepSize;
  if (elementsToKeep > 0) memcpy(&BufferNewValue[0], &BufferOldValue[StepSize], sizeof(Data_Analyser) * elementsToKeep);
  *indx = elementsToKeep;
  
};

STATUS_ATTACK Analyser::process(Data_Analyser* data, unsigned long Time)
{
  if (!update) return lastStatus;

  if (indOldRow < SIZE_ROW) 
  {
    memcpy(&RawDataRows.BufferOldValue[indOldRow++], data, sizeof(Data_Analyser));
    
    if (indOldRow == SIZE_ROW) NormalizeData(RawDataRows.BufferOldValue, &OldNormalRows);
    update = false;
    return lastStatus;
  }

  if (RawDataRows.needSlice) RawDataRows.Slice(StepSize, &indNewRow);

  if (indNewRow < SIZE_ROW)
  {
    memcpy(&RawDataRows.BufferNewValue[indNewRow++], data, sizeof(Data_Analyser));
    if (indNewRow < SIZE_ROW)
    {
      update = false;
      return lastStatus;
    }
  }
  NormalizeData(RawDataRows.BufferNewValue, &NewNormalRow);
  
  // Считаем разницу между Old и New
  CalculateAllEntropyMetrics(&OldNormalRows, &NewNormalRow, &EntropyStorage, &Time);

  for (unsigned char i = 0; i < STATUS_ATTACK::COUNT_ATTACK; i++)
  {
    if (i == STATUS_ATTACK::UNKNOWN)
    {
      lastStatus = STATUS_ATTACK::UNKNOWN;
      TimeDetect = Time;
      break;
    }
    unsigned char GeneralCount = 0;
    unsigned char ParamCount = 0;
    unsigned char FindParam = 0;
    unsigned char FindGeneralParam = 0;
    
    for (unsigned char j = 0; j < PARAMETERS::COUNT; j++)
    {
      CorrelationRule rule = Rules.value[i][j];
      if (rule.level != -1.0f)
      {
        ParamCount++;
        if (rule.general) GeneralCount++;
        if (EntropyStorage.data[j].DetectedLevel >= rule.level)
        {
          if (rule.general) FindGeneralParam++;
          else FindParam++;
        }
      }
    }
    
    float prob = 0.0f;
    prob =  ((float)FindGeneralParam/GeneralCount) * 0.9f + ((float)FindParam/ParamCount) * 0.1f;
    
    if (prob > 0.5f)
    {
      lastStatus = (STATUS_ATTACK)i;
      TimeDetect = Time;
      break;
    }
  }
  
  memcpy(RawDataRows.BufferOldValue, RawDataRows.BufferNewValue, sizeof(Data_Analyser) * SIZE_ROW);
  memcpy(&OldNormalRows, &NewNormalRow, sizeof(NormalRows));
  
  RawDataRows.needSlice = true;
  update = false;
  return lastStatus;
};

void Analyser::CalcPoissonPMFForArray(float* Input, float* Output, unsigned char Size)
{
  if (Size == 0) return;

    float minVal = Input[0];
    float sumInput = 0.0f;
    for (unsigned char i = 0; i < Size; i++)
    {
      if (Input[i] < minVal) minVal = Input[i];
      sumInput += Input[i];
    }

    float shift = (minVal < 0.0f) ? fabsf(minVal) : 0.0f;
    float lambda = (sumInput / (float)Size) + shift;
    if (lambda < 1e-9f) lambda = 1e-9f;
    float logLambda = logf(lambda);

    float epsilon = 1e-10f;
    float pmfSum = 0.0f;

    for (unsigned char i = 0; i < Size; i++) 
    {
      int k = (int)roundf(Input[i] + shift);
      if (k < 0) k = 0;
      // ln(P) = k*ln(L) - L - ln(Gamma(k+1))
      float logProb = (float)k * logLambda - lambda - lgammaf((float)k + 1.0f);
      Output[i] = expf(logProb);
      pmfSum += (Output[i] + epsilon);
    }
    // Финальная нормализация ряда
    for (unsigned char i = 0; i < Size; i++) Output[i] = (Output[i] + epsilon) / pmfSum;
};

float Analyser::LnFactorial(float k) 
{
  if (k <= 1) return 0.0f;
  float res = 0.0f;
  for (int i = 2; i <= k; i++) res += logf((float)i);
  return res;
}

void Analyser::NormalizeData(Data_Analyser* dataArray, NormalRows* normalRow)
{
  float tempBuffer[SIZE_ROW];

  for (unsigned char f = 0; f < PARAMETERS::COUNT; f++) 
  {
    for (unsigned char r = 0; r < SIZE_ROW; r++) tempBuffer[r] = dataArray[r].values[f];
    CalcPoissonPMFForArray(tempBuffer, normalRow->data[f], SIZE_ROW);
  }

};

float Analyser::CalculateKL(float* p, float* q, unsigned char size)
{
  float sum = 0.0f;
  const float eps = 1e-12f;
  for (unsigned char i = 0; i < size; i++) 
  {
    if (p[i] <= eps) continue;
    float qi = (q[i] <= eps) ? eps : q[i];
    sum += p[i] * logf(p[i] / qi);
  }
  return (sum < 0.0f) ? 0.0f : sum;
}

float Analyser::CalculateJSD(float* p, float* q, unsigned char size)
{
  float m[SIZE_ROW];
  for (unsigned char i = 0; i < size; i++)  m[i] = 0.5f * (p[i] + q[i]);

  // JSD = 0.5 * KL(P||M) + 0.5 * KL(Q||M)
  return 0.5f * CalculateKL(p, m, size) + 0.5f * CalculateKL(q, m, size);
}

void Analyser::CalculateFieldMetrics(float* p, float* q, unsigned char size, EntropyMetrics* out, unsigned long* Time)
{    
    float OldToNew = CalculateKL(p, q, size);
    float NewToOld = CalculateKL(q, p, size);
    float NewSumEnt = OldToNew + NewToOld;
    
    unsigned char curLevel = out->DetectedLevel;
    unsigned long curTimeDetect = out->TimeDetect;
    
    unsigned char newLevel;
    for (unsigned char i = 0; i < EntropyLevels::LevelsCount; i++) 
    {
      if (NewSumEnt >= Levels.values[i]) newLevel = i;
    }
    if (newLevel >= out->DetectedLevel)
    {
      out->DetectedLevel = newLevel;
      out->Sum = NewSumEnt;
      out->OldToNew = OldToNew;
      out->NewToOld = NewToOld;
      out->TimeDetect = *Time;
    }
    else
    {
      if ((*Time - out->TimeDetect) > TimeLiveLevelDetect * 1000)
      {
        out->DetectedLevel = newLevel;
        out->Sum = NewSumEnt;
        out->OldToNew = OldToNew;
        out->NewToOld = NewToOld;
        out->TimeDetect = *Time;
      }
    }
}

void Analyser::CalculateAllEntropyMetrics(NormalRows* oldProb, NormalRows* newProb, AllEntropyMetrics* outMetrics, unsigned long* Time)
{
  for (unsigned char f = 0; f < PARAMETERS::COUNT; f++) 
  {
    CalculateFieldMetrics(oldProb->data[f], newProb->data[f], SIZE_ROW, &outMetrics->data[f], Time);
  }
}