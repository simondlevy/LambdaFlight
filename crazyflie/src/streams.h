#pragma once

#include <datatypes.h>
#include <math3d.h>

#ifndef _MAIN
#define EXTERN extern
#else
#define EXTERN
#endif

EXTERN demands_t stream_openLoopDemands;
EXTERN vehicleState_t stream_vehicleState;
EXTERN bool stream_inFlyingMode;
EXTERN bool stream_resetPids;

EXTERN ekfAction_e stream_ekfAction;

