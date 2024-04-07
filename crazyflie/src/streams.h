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
EXTERN uint32_t stream_nowMsec;
EXTERN uint32_t stream_nextPredictionMsec;
EXTERN bool stream_isFlying;
EXTERN Axis3f stream_gyro;
EXTERN Axis3f stream_accel;
EXTERN flowMeasurement_t stream_flow; 
EXTERN rangeMeasurement_t stream_range;

void setStateIsInBounds(const bool inBounds);
void setState(const vehicleState_t & state);
