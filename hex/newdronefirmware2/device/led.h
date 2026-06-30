#pragma once

class ControllerLED
{
public:
  ControllerLED();
  
  void Processing(unsigned long tick);
  
  void SetErrorBlink(bool error, unsigned short CountBlink = 10);
  
private:
  bool SetIn = false;
  bool LedSet = false;
  bool ErrorBlink = false;
  bool EndSycle = false;
  unsigned short Current = 1;
  unsigned short CountBlink = 0;
  float TimeChangeLed = 100;
  unsigned short TimeSycle = 3000;
  unsigned long Timer = 0;
};


class RPIController
{
public:
  RPIController();
  
  void Processing(short StickVal);
  
private:
  bool Enable = false;
};