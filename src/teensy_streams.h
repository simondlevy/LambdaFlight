#pragma once

#include <datatypes.h>
#include <math3d.h>

#ifndef _MAIN
#define EXTERN extern
#else
#define EXTERN
#endif

EXTERN uint32_t stream_now_msec;
EXTERN float stream_dt;
EXTERN float stream_channel1_raw;
EXTERN float stream_channel2_raw;
EXTERN float stream_channel3_raw;
EXTERN float stream_channel4_raw;
EXTERN float stream_channel5_raw;
EXTERN bool stream_radio_failsafe;
EXTERN float stream_quat_w;
EXTERN float stream_quat_x;
EXTERN float stream_quat_y;
EXTERN float stream_quat_z;
EXTERN float stream_gyro_x;
EXTERN float stream_gyro_y;
EXTERN float stream_gyro_z;
EXTERN float stream_accel_x;
EXTERN float stream_accel_y;
EXTERN float stream_accel_z;

EXTERN ekfAction_e       stream_ekfAction;
EXTERN flowMeasurement_t stream_flow; 
EXTERN bool              stream_inFlyingMode;
EXTERN bool              stream_isFlying;
EXTERN uint32_t          stream_nextPredictionMsec;
EXTERN uint32_t          stream_nowMsec;
EXTERN demands_t         stream_openLoopDemands;
EXTERN float             stream_rangefinder_distance;
EXTERN bool              stream_resetPids;
EXTERN vehicleState_t    stream_vehicleState;

void setStateIsInBounds(const bool inBounds);
void setState(const vehicleState_t & state);
