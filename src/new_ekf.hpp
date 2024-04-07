#pragma once

#include <string.h>

#include "math3d.h"
#include "datatypes.h"
#include "linalg.h"

// Quaternion used for initial orientation
static const float QW_INIT = 1;
static const float QX_INIT = 0;
static const float QY_INIT = 0;
static const float QZ_INIT = 0;

// Initial variances, uncertain of position, but know we're
// stationary and roughly flat
static const float STDEV_INITIAL_POSITION_Z = 1;
static const float STDEV_INITIAL_VELOCITY = 0.01;
static const float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
static const float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

static const float PROC_NOISE_ACC_XY = 0.5;
static const float PROC_NOISE_ACC_Z = 1.0;
static const float PROC_NOISE_VEL = 0;
static const float PROC_NOISE_POS = 0;
static const float PROC_NOISE_ATT = 0;
static const float MEAS_NOISE_GYRO_ROLL_PITCH = 0.1; // radians per second
static const float MEAS_NOISE_GYRO_ROLL_YAW = 0.1;   // radians per second

static const float MSS_TO_GS = 9.81;

//We do get the measurements in 10x the motion pixels (experimentally measured)
static const float FLOW_RESOLUTION = 0.1;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
static const float MAX_COVARIANCE = 100;
static const float MIN_COVARIANCE = 1e-6;

// The bounds on states, these shouldn't be hit...
static const float MAX_POSITION = 100; //meters
static const float MAX_VELOCITY = 10; //meters per second

// Small number epsilon, to prevent dividing by zero
static const float EPS = 1e-6f;

// the reversion of pitch and roll to zero
static const float ROLLPITCH_ZERO_REVERSION = 0.001;

typedef enum {

    EKF_INIT,
    EKF_PREDICT,
    EKF_FINALIZE,
    EKF_GET_STATE,
    EKF_UPDATE_WITH_GYRO,
    EKF_UPDATE_WITH_ACCEL,
    EKF_UPDATE_WITH_FLOW,
    EKF_UPDATE_WITH_RANGE 

} new_ekfAction_e;

// Indexes to access the state
enum {

    KC_STATE_Z,
    KC_STATE_DX,
    KC_STATE_DY,
    KC_STATE_DZ,
    KC_STATE_E0,
    KC_STATE_E1,
    KC_STATE_E2,
    KC_STATE_DIM
};

typedef struct {

    // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
    // to allow easy normalization (in comparison to a rotation matrix),
    // while also being robust against singularities (in comparison to euler angles)
    float w;
    float x;
    float y;
    float z;

} newquat_t;

typedef struct {

    float z;
    float dx;
    float dy;
    float dz;
    float e0;
    float e1;
    float e2;

} ekfState_t;

typedef struct {

    Axis3f sum;
    uint32_t count;
    float conversionFactor;

    Axis3f subSample;

} axis3fSubSampler_t;

typedef struct {

    newquat_t quat;

    Axis3f _gyroLatest;

    axis3fSubSampler_t accSubSampler;
    axis3fSubSampler_t gyroSubSampler;

    ekfState_t ekfState;

    Axis3f r;

    float p[KC_STATE_DIM][KC_STATE_DIM];

    bool isUpdated;

    uint32_t lastPredictionMs;

    uint32_t lastProcessNoiseUpdateMs;

} ekf_t;

static void ekf_init(ekf_t & ekf, const uint32_t nowMsec)
{
    ekf.ekfState.z = 0;
    ekf.ekfState.dx = 0;
    ekf.ekfState.dy = 0;
    ekf.ekfState.dz = 0;
    ekf.ekfState.e0 = 0;
    ekf.ekfState.e1 = 0;
    ekf.ekfState.e2 = 0;

    ekf.quat.w = QW_INIT;
    ekf.quat.x = QX_INIT;
    ekf.quat.y = QY_INIT;
    ekf.quat.z = QZ_INIT;

    ekf.r.x = 0;
    ekf.r.y = 0;
    ekf.r.z = 1;

    memset(&ekf.p, 0, sizeof(ekf.p));

    ekf.p[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);
    ekf.p[KC_STATE_DX][KC_STATE_DX] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_DY][KC_STATE_DY] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_DZ][KC_STATE_DZ] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_E1][KC_STATE_E1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
    ekf.p[KC_STATE_E2][KC_STATE_E2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);

    ekf.isUpdated = false;
    ekf.lastPredictionMs = nowMsec;
    ekf.lastProcessNoiseUpdateMs = nowMsec;
}

static void new_ekf_step(
        const new_ekfAction_e action,
        const uint32_t nowMsec)
{
    static ekf_t _ekf;

    ekf_t ekf = {};
    ekf_init(ekf, nowMsec);

    (void)action;

    (void)_ekf;
}
