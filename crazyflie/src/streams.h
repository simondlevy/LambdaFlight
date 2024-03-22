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

EXTERN quat_t stream_quat;
EXTERN axes_t stream_gyro;

EXTERN int8_t stream_ekfMode;
EXTERN uint32_t stream_ekfNowMsec;
EXTERN uint32_t stream_ekfNextPredictionMsec;
EXTERN bool stream_ekfIsFlying;
EXTERN measurement_t stream_ekfMeasurement;
