#pragma once

#include <datatypes.h>

#ifndef _MAIN
#define EXTERN extern
#else
#define EXTERN
#endif

EXTERN float       stream_accel_x;
EXTERN float       stream_accel_y;
EXTERN float       stream_accel_z;
EXTERN float       stream_dt;
EXTERN ekfAction_e stream_ekf_action;
EXTERN float       stream_gyro_x;
EXTERN float       stream_gyro_y;
EXTERN float       stream_gyro_z;
EXTERN bool        stream_radio_failsafe;
EXTERN float       stream_channel1_raw;
EXTERN float       stream_channel2_raw;
EXTERN float       stream_channel3_raw;
EXTERN float       stream_channel4_raw;
EXTERN float       stream_channel5_raw;
EXTERN bool        stream_is_flying;
EXTERN uint32_t    stream_next_prediction_msec;
EXTERN uint32_t    stream_now_msec;
EXTERN float       stream_quat_w;
EXTERN float       stream_quat_x;
EXTERN float       stream_quat_y;
EXTERN float       stream_quat_z;
EXTERN float       stream_rangefinder_distance;


