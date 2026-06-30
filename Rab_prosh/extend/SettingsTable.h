#pragma once

#include "INS/cont/Attitude.h"
#include "INS/cont/Velocity.h"
#include "INS/device/Barometer.h"
#include "INS/device/Compass.h"
#include "INS/device/OpticalFlow.h"
#include "INS/cont/Engine.h"
#include "device/interfaces/IMag.h"
#include "INS/device/CameraFlow.h"

#include "Autopilot.h"

extern AutoPilotSettings APSettings;
extern Attitude MainAtt;
extern Velocity MainVel;
extern OpticalFlow MainOF;
extern Engine MainEng;
extern CameraFlow MainCam;
//extern Compass MainCompas;
extern Barometer MainBar;