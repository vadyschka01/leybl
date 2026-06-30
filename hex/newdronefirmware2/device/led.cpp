#include "stm32g4xx.h"

#include "gpio.h"
#include "joy.h"
#include "led.h"

ControllerLED::ControllerLED()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  GPIO_InitPin(GPIO_PIN_15 | GPIO_PORT_A | GPIO_INPUT);
  SetIn = true;
}

void ControllerLED::Processing(unsigned long tick)
{
  if(ErrorBlink)
  {
    if(EndSycle)
    {
      if (Timer == 0) Timer = tick;
      if (tick - Timer > TimeSycle)
      {
        EndSycle = false;
        Timer = 0;
      }
    }
    else
    {
      if (Timer == 0) Timer = tick;
      if (tick - Timer > TimeChangeLed)
      {
        GPIO_InitPin(GPIO_PIN_15 | GPIO_PORT_A | GPIO_OUTPUT);
        if(SetIn) SetIn = false;
        GPIOA->BSRR = GPIO_BSRR_BR_15;
        if (tick - Timer > TimeChangeLed + 500)
        {
          Timer = tick;
          GPIO_InitPin(GPIO_PIN_15 | GPIO_PORT_A | GPIO_INPUT);
          if (!SetIn) SetIn = true;
          Current += 1;
          if (Current == CountBlink)
          {
            Current = 0;
            EndSycle = true;
            Timer = 0;
          }
        }
      }
    }
  }
  else
  {
    if (SetIn)
    {
      GPIO_InitPin(GPIO_PIN_15 | GPIO_PORT_A | GPIO_OUTPUT);
      SetIn = false;
    }
    if (tick - Timer > TimeChangeLed)
    {
      if (LedSet) 
      {
        LedSet = false;
        GPIOA->BSRR = GPIO_BSRR_BS_15;
      }
      else 
      {
        LedSet = true;
        GPIOA->BSRR = GPIO_BSRR_BR_15;
      }
      Timer = tick;
    }
  }
}

void ControllerLED::SetErrorBlink(bool error, unsigned short countBlink)
{
  ErrorBlink = error;
  CountBlink = countBlink;
  TimeChangeLed = 1000.0f/CountBlink;
}
//------------------------------------------------------------------------------


RPIController::RPIController()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIO_InitPin(GPIO_PIN_15 | GPIO_PORT_B | GPIO_OUTPUT);
};

void RPIController::Processing(short StickVal)
{
  bool en = false;
  if (StickVal > JOYPAD_MID) en = true;
  
  if (en != Enable)
  {
    GPIOB->BSRR = en ? GPIO_BSRR_BS_15 : GPIO_BSRR_BR_15;
    Enable = en;
  }
};