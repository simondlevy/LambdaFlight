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

EXTERN ekfMode_e stream_ekfMode;
EXTERN uint32_t stream_ekfNowMsec;
EXTERN uint32_t stream_ekfNextPredictionMsec;
EXTERN bool stream_ekfIsFlying;
EXTERN measurement_t stream_ekfMeasurement;
