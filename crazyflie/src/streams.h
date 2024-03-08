#pragma once

#include <datatypes.h>

#ifndef _MAIN
#define EXTERN extern
#else
#define EXTERN
#endif

EXTERN demands_t stream_openLoopDemands;
EXTERN vehicleState_t stream_vehicleState;
EXTERN bool stream_inHoverMode;
EXTERN bool stream_resetPids;

EXTERN kalmanMode_e kalmanMode;
EXTERN uint32_t kalmanNowMsec;
EXTERN uint32_t kalmanNextPredictionMsec;
EXTERN bool kalmanIsFlying;
EXTERN measurement_t kalmanMeasurement;
