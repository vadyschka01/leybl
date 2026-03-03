#include <string.h>

#include "Settings.h"

LevelHorData LevelHor;

const char* Settings::GetCategoryName(unsigned char id, unsigned char& size)
{
  const char* name = MainMenuNames[id];
  size = strlen(name);
  return name;
}

DATA_TYPES Settings::GetParameterDataType(unsigned char id_cat, unsigned char id_param)
{
  ParameterData param = MenuParameters[id_cat][id_param];
  return param.type;
}

const char* Settings::GetParameterName(unsigned char id_cat, unsigned char id_param, unsigned char& size)
{
  ParameterData param = MenuParameters[id_cat][id_param];
  size = strlen(param.name);
  return param.name;
}

unsigned char Settings::GetCountCategories()
{
  return sizeof(MainMenuNames) / sizeof(char*);
}

unsigned char Settings::GetCountParameters(unsigned char id_cat)
{
  return MenuParametersCount[id_cat];
}

unsigned char Settings::GetTypeSize(DATA_TYPES type)
{
  switch (type)
  {
    case DATA_TYPES::dt_char: return sizeof(char);
    case DATA_TYPES::dt_unsigned_char: return sizeof(unsigned char);
    case DATA_TYPES::dt_short: return sizeof(short);
    case DATA_TYPES::dt_unsigned_short: return sizeof(unsigned short);
    case DATA_TYPES::dt_int: return sizeof(int);
    case DATA_TYPES::dt_unsigned_int: return sizeof(unsigned int);
    case DATA_TYPES::dt_long: return sizeof(long);
    case DATA_TYPES::dt_unsigned_long: return sizeof(unsigned long);
    case DATA_TYPES::dt_long_long: return sizeof(long long);
    case DATA_TYPES::dt_unsigned_long_long: return sizeof(unsigned long long);
    case DATA_TYPES::dt_float_32: return sizeof(float);
    case DATA_TYPES::dt_float_64: return sizeof(double);
    case DATA_TYPES::dt_bool: return sizeof(bool);
    default: return 0;
  }
}

unsigned char Settings::ReadOrWriteParameter(unsigned char id_cat, unsigned char id_param, unsigned char value[], bool read)
{
  ParameterData param = MenuParameters[id_cat][id_param];
  unsigned char size = 0;
  switch (param.type)
  {
    case DATA_TYPES::dt_char:
    {
      size = sizeof(char);
      break;
    }
    case DATA_TYPES::dt_unsigned_char:
    {
      size = sizeof(unsigned char);
      break;
    }
    case DATA_TYPES::dt_short:
    {
      size = sizeof(short);
      break;
    }
    case DATA_TYPES::dt_unsigned_short:
    {
      size = sizeof(unsigned short);
      break;
    }
    case DATA_TYPES::dt_int:
    {
      size = sizeof(int);
      break;
    }
    case DATA_TYPES::dt_unsigned_int:
    {
      size = sizeof(unsigned int);
      break;
    }
    case DATA_TYPES::dt_long:
    {
      size = sizeof(long);
      break;
    }
    case DATA_TYPES::dt_unsigned_long:
    {
      size = sizeof(unsigned long);
      break;
    }
    case DATA_TYPES::dt_long_long:
    {
      size = sizeof(long long);
      break;
    }
    case DATA_TYPES::dt_unsigned_long_long:
    {
      size = sizeof(unsigned long long);
      break;
    }
    case DATA_TYPES::dt_float_32:
    {
      size = sizeof(float);
      break;
    }
    case DATA_TYPES::dt_float_64:
    {
      size = sizeof(double);
      break;
    }
    case DATA_TYPES::dt_bool:
    {
      size = sizeof(bool);
      break;
    }
    case DATA_TYPES::dt_string:
    {
      // Будут ли такие вообще? Как обработать запись..
      if (read) size = strlen((const char*)param.value);
      else size = 0;
      break;
    }
    case DATA_TYPES::dt_unknown:
    {
      size = 0;
      break;
    }
  }
  if (size == 0) return 0;
  if (read) memcpy(value, param.value, size);
  else memcpy(param.value, value, size);
  return size;
}
