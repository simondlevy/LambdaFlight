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

    Axis3f subSample;

} axisSubSampler_t;

typedef struct {

    newquat_t quat;

    Axis3f _gyroLatest;

    axisSubSampler_t accSubSampler;
    axisSubSampler_t gyroSubSampler;

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

static Axis3f* subSamplerFinalize(
        axisSubSampler_t* subSampler,
        const float conversionFactor)
{
    const auto count  = subSampler->count; 
    const auto isCountNonzero = count > 0;

    subSampler->subSample.x = isCountNonzero ? 
        subSampler->sum.x * conversionFactor / count :
        subSampler->subSample.x;

    subSampler->subSample.y = isCountNonzero ?
        subSampler->sum.y * conversionFactor / count :
        subSampler->subSample.y;

    subSampler->subSample.z = isCountNonzero ?
        subSampler->sum.z * conversionFactor / count :
        subSampler->subSample.z;

    // Reset
    subSampler->count = 0;
    subSampler->sum = (Axis3f){.axis={0}};

    return &subSampler->subSample;
}


static void ekf_predict(ekf_t & ekf, const bool isFlying) 
{
    subSamplerFinalize(&ekf.gyroSubSampler, DEGREES_TO_RADIANS);

#if 0
    const float dt = (nowMsec - ekf.lastPredictionMs) / 1000.0f;

    const auto dt2 = dt * dt;

    axis3fSubSamplerFinalize(&_accSubSampler, shouldPredict);
    const Axis3f * acc = &_accSubSampler.subSample; 

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = ekf.ekfState.dx * dt + isFlying ? 0 : acc->x * dt2 / 2;
    const auto dy = ekf.ekfState.dy * dt + isFlying ? 0 : acc->y * dt2 / 2;
    const auto dz = ekf.ekfState.dz * dt + acc->z * dt2 / 2; 

    // keep previous time step's state for the update
    const auto tmpSDX = ekf.ekfState.dx;
    const auto tmpSDY = ekf.ekfState.dy;
    const auto tmpSDZ = ekf.ekfState.dz;

    const auto accx = isFlying ? 0 : acc->x;
    const auto accy = isFlying ? 0 : acc->y;

    const Axis3f * gyro = &_gyroSubSampler.subSample; 

    // attitude update (rotate by gyroscope), we do this in quaternions
    // this is the gyroscope angular velocity integrated over the sample period
    const auto dtwx = dt*gyro->x;
    const auto dtwy = dt*gyro->y;
    const auto dtwz = dt*gyro->z;

    // compute the quaternion values in [w,x,y,z] order
    const auto angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
    const auto ca = cos(angle/2);
    const auto sa = sin(angle/2);
    const auto dqw = ca;
    const auto dqx = sa*dtwx/angle;
    const auto dqy = sa*dtwy/angle;
    const auto dqz = sa*dtwz/angle;

    // rotate the quad's attitude by the delta quaternion vector computed above

    const auto tmpq0 = rotateQuat(
            dqw*_qw - dqx*_qx - dqy*_qy - dqz*_qz, QW_INIT, isFlying);

    const auto tmpq1 = rotateQuat(
            dqx*_qw + dqw*_qx + dqz*_qy - dqy*_qz, QX_INIT, isFlying);

    const auto tmpq2 = rotateQuat(
            dqy*_qw - dqz*_qx + dqw*_qy + dqx*_qz, QY_INIT, isFlying);

    const auto tmpq3 = rotateQuat(
            dqz*_qw + dqy*_qx - dqx*_qy + dqw*_qz, QZ_INIT, isFlying);

    // normalize and store the result
    const auto norm = 
        sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
        EPS;

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless
    // to estimate body angle while flying)

    // XXX predict()
    // altitude update
    ekf.ekfState.z += shouldPredict ? ekf.r.x * dx + ekf.r.y * dy + ekf.r.z * dz - 
        MSS_TO_GS * dt2 / 2 :
        0;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    ekf.ekfState.dx += shouldPredict ? 
        dt * (accx + gyro->z * tmpSDY - gyro->y * tmpSDZ - 
                MSS_TO_GS * ekf.r.x) : 
        0;

    ekf.ekfState.dy += shouldPredict ?
        dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - 
                MSS_TO_GS * ekf.r.y) : 
        0;

    ekf.ekfState.dz += shouldPredict ?
        dt * (acc->z + gyro->y * tmpSDX - gyro->x * tmpSDY - 
                MSS_TO_GS * ekf.r.z) :
        0;

    // predict()
    ekf.qw = shouldPredict ? tmpq0/norm : ekf.qw;
    ekf.qx = shouldPredict ? tmpq1/norm : ekf.qx; 
    ekf.qy = shouldPredict ? tmpq2/norm : ekf.qy; 
    ekf.qz = shouldPredict ? tmpq3/norm : ekf.qz;

    ekf.isUpdated = shouldPredict ? true : ekf.isUpdated;

    ekf.lastPredictionMs = shouldPredict ? nowMsec : ekf.lastPredictionMs;

    // ====== COVARIANCE UPDATE ======

    const auto e0 = gyro->x*dt/2;
    const auto e1 = gyro->y*dt/2;
    const auto e2 = gyro->z*dt/2;

    const auto e0e0 =  1 - e1*e1/2 - e2*e2/2;
    const auto e0e1 =  e2 + e0*e1/2;
    const auto e0e2 = -e1 + e0*e2/2;

    const auto e1e0 =  -e2 + e0*e1/2;
    const auto e1e1 = 1 - e0*e0/2 - e2*e2/2;
    const auto e1e2 = e0 + e1*e2/2;

    const auto e2e0 = e1 + e0*e2/2;
    const auto e2e1 = -e0 + e1*e2/2;
    const auto e2e2 = 1 - e0*e0/2 - e1*e1/2;

    // altitude from body-frame velocity
    const auto zdx  = ekf.r.x*dt;
    const auto zdy  = ekf.r.y*dt;
    const auto zdz  = ekf.r.z*dt;

    // altitude from attitude error
    const auto ze0  = (_ekfState.dy*_r.z - ekf.ekfState.dz*_r.y)*dt;
    const auto ze1  = (- ekf.ekfState.dx*_r.z + ekf.ekfState.dz*_r.x)*dt;
    const auto ze2  = (_ekfState.dx*_r.y - ekf.ekfState.dy*_r.x)*dt;

    // body-frame velocity from body-frame velocity
    const auto dxdx  = 1; //drag negligible
    const auto dydx =  -gyro->z*dt;
    const auto dzdx  = gyro->y*dt;

    const auto dxdy  = gyro->z*dt;
    const auto dydy  = 1; //drag negligible
    const auto dzdy  = gyro->x*dt;

    const auto dxdz =  gyro->y*dt;
    const auto dydz  = gyro->x*dt;
    const auto dzdz  = 1; //drag negligible

    // body-frame velocity from attitude error
    const auto dxe0  = 0;
    const auto dye0  = -MSS_TO_GS*_r.z*dt;
    const auto dze0  = MSS_TO_GS*_r.y*dt;

    const auto dxe1  = MSS_TO_GS*_r.z*dt;
    const auto dye1  = 0;
    const auto dze1  = -MSS_TO_GS*_r.x*dt;

    const auto dxe2  = -MSS_TO_GS*_r.y*dt;
    const auto dye2  = MSS_TO_GS*_r.x*dt;
    const auto dze2  = 0;

    const float A[KC_STATE_DIM][KC_STATE_DIM] = 
    { 
        //        Z  DX    DY    DZ    E0    E1    E2
        /*Z*/    {0, zdx,  zdy,  zdz,  ze0,  ze1,  ze2}, 
        /*DX*/   {0, dxdx, dxdy, dxdz, dxe0, dxe1, dxe2}, 
        /*DY*/   {0, dydx, dydy, dydz, dye0, dye1, dye2},
        /*DZ*/   {0, dzdx, dzdy, dzdz, dze0, dze1, dze2},
        /*E0*/   {0, 0,    0,    0,    e0e0, e0e1, e0e2}, 
        /*E1*/   {0, 0,    0,    0,    e1e0, e1e1, e1e2}, 
        /*E2*/   {0, 0,    0,    0,    e2e0, e2e1, e2e2}  
    };

    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'
    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, ekf.Pmat, AP, true);  // AP
    multiply(AP, At, ekf.Pmat, shouldPredict); // APA'

    const auto dt1 = (nowMsec - ekf.lastProcessNoiseUpdateMs) / 1000.0f;
    const auto isDtPositive = dt1 > 0;


    // Add process noise

    const float noise[KC_STATE_DIM] = {
        powf(PROC_NOISE_ACC_Z*dt1*dt1 + PROC_NOISE_VEL*dt1 + PROC_NOISE_POS, 2), 
        powf(PROC_NOISE_ACC_XY*dt1 + PROC_NOISE_VEL, 2), 
        powf(PROC_NOISE_ACC_XY*dt1 + PROC_NOISE_VEL, 2), 
        powf(PROC_NOISE_ACC_Z*dt1 + PROC_NOISE_VEL, 2), 
        powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2), 
        powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2), 
        powf(MEAS_NOISE_GYRO_ROLL_YAW * dt1 + PROC_NOISE_ATT, 2) 
    };

    addNoiseDiagonal(_Pmat, noise, isDtPositive);

    updateCovarianceMatrix(isDtPositive);

    ekf.lastProcessNoiseUpdateMs = isDtPositive ?  nowMsec : 
        ekf.lastProcessNoiseUpdateMs;
#endif
}


static void new_ekf_step(
        const new_ekfAction_e action,
        const uint32_t nowMsec,
        const uint32_t nextPredictionMsec,
        const bool isFlying)
{
    static ekf_t _ekf;

    ekf_t ekf1 = {};
    ekf_init(ekf1, nowMsec);

    const bool shouldPredict = nowMsec >= nextPredictionMsec;

    ekf_t ekf2 = {};
    ekf_predict(ekf2, isFlying);

    (void)shouldPredict;
    (void)action;

    (void)_ekf;
}
