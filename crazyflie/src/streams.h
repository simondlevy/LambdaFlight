#pragma once

#include <datatypes.h>
#include <math3d.h>

#ifndef _MAIN
#define EXTERN extern
#else
#define EXTERN
#endif

EXTERN axis3_t           stream_gyro;
EXTERN bool              stream_inFlyingMode;
EXTERN demands_t         stream_openLoopDemands;
EXTERN bool              stream_resetPids;
EXTERN vehicleState_t    stream_vehicleState;

void setStateIsInBounds(const bool inBounds);
void setState(const vehicleState_t & state);
