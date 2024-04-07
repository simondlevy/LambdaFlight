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

    Axis3f gyroLatest;

    axisSubSampler_t accelSubSampler;
    axisSubSampler_t gyroSubSampler;

    ekfState_t ekfState;

    Axis3f r;

    float p[KC_STATE_DIM][KC_STATE_DIM];

    bool isUpdated;

    uint32_t lastPredictionMs;

    uint32_t lastProcessNoiseUpdateMs;

} ekf_t;

static const float max(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static void subSamplerAccumulate(
        axisSubSampler_t * subSampler,
        const Axis3f * sample) 
{
    subSampler->sum.x += sample->x;
    subSampler->sum.y += sample->y;
    subSampler->sum.z += sample->z;

    subSampler->count++;
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

static float rotateQuat(
        const float val, 
        const float initVal,
        const bool isFlying)
{
    const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

    return (val * (isFlying ? 1: keep)) +
        (isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
}

static void addNoiseDiagonal(
        float a[KC_STATE_DIM][KC_STATE_DIM],
        const float d[KC_STATE_DIM],
        const bool doit)
{
    a[0][0] += doit ? d[0] : 0;
    a[1][1] += doit ? d[1] : 0;
    a[2][2] += doit ? d[2] : 0;
    a[3][3] += doit ? d[3] : 0;
    a[4][4] += doit ? d[4] : 0;
    a[5][5] += doit ? d[5] : 0;
    a[6][6] += doit ? d[6] : 0;
}

static void updateCovarianceCell(
        float p[KC_STATE_DIM][KC_STATE_DIM],
        const int i, 
        const int j, 
        const float variance,
        const bool shouldUpdate)
{
    const auto pval = (p[i][j] + p[j][i]) / 2 + variance;

    p[i][j] = !shouldUpdate ? p[i][j] :
        (isnan(pval) || pval > MAX_COVARIANCE) ?  MAX_COVARIANCE :
        (i==j && pval < MIN_COVARIANCE) ?  MIN_COVARIANCE :
        pval;

    p[j][i] = shouldUpdate ? p[i][j] : p[j][i];
}


static void updateCovarianceMatrix(
        float p[KC_STATE_DIM][KC_STATE_DIM],
        const bool shouldUpdate)
{
    // Enforce symmetry of the covariance matrix, and ensure the
    // values stay bounded
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {
            updateCovarianceCell(p, i, j, 0, shouldUpdate);
        }
    }
}

static void scalarUpdate(
        ekf_t & ekf,
        const float h[KC_STATE_DIM],
        const float error, 
        const float stdMeasNoise,
        const bool shouldUpdate)
{

    // ====== INNOVATION COVARIANCE ======
    float ph[KC_STATE_DIM] = {};
    multiply(ekf.p, h, ph);
    const auto r = stdMeasNoise * stdMeasNoise;
    const auto hphr = r + dot(h, ph); // HPH' + R

    // Compute the Kalman gain as a column vector
    const float g[KC_STATE_DIM] = {

        // kalman gain = (PH' (HPH' + R )^-1)
        ph[0] / hphr, 
        ph[1] / hphr, 
        ph[2] / hphr, 
        ph[3] / hphr, 
        ph[4] / hphr, 
        ph[5] / hphr, 
        ph[6] / hphr
    };

    // Perform the state update
    // XXX update()
    ekf.ekfState.z  += shouldUpdate ? g[0] * error: 0;
    ekf.ekfState.dx += shouldUpdate ? g[1] * error: 0;
    ekf.ekfState.dy += shouldUpdate ? g[2] * error: 0;
    ekf.ekfState.dz += shouldUpdate ? g[3] * error: 0;
    ekf.ekfState.e0 += shouldUpdate ? g[4] * error: 0;
    ekf.ekfState.e1 += shouldUpdate ? g[5] * error: 0;
    ekf.ekfState.e2 += shouldUpdate ? g[6] * error: 0;

    // ====== COVARIANCE UPDATE ======

    float GH[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(g, h, GH); // KH

    for (int i=0; i<KC_STATE_DIM; i++) { 
        GH[i][i] -= 1;
    } // KH - I

    float GHt[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(GH, GHt);      // (KH - I)'

    float GHIP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(GH, ekf.p, GHIP, true);  // (KH - I)*P

    multiply(GHIP, GHt, ekf.p, shouldUpdate); // (KH - I)*P*(KH - I)'

    // Add the measurement variance and ensure boundedness and symmetry
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {

            updateCovarianceCell(ekf.p, i, j, g[i] * r * g[j], shouldUpdate);
        }
    }

    ekf.isUpdated = shouldUpdate ? true : ekf.isUpdated;
}

static bool isPositionWithinBounds(const float pos)
{
    return fabs(pos) < MAX_POSITION;
}

static bool isVelocityWithinBounds(const float vel)
{
    return fabs(vel) < MAX_VELOCITY;
}

static bool isErrorLarge(const float v)
{
    return fabs(v) > 0.1e-3f;
}

static bool isErrorInBounds(const float v)
{
    return fabs(v) < 10;
}


static bool isStateWithinBounds(ekf_t & ekf) 
{
    return 
        isPositionWithinBounds(ekf.ekfState.z) &&
        isVelocityWithinBounds(ekf.ekfState.dx) &&
        isVelocityWithinBounds(ekf.ekfState.dy) &&
        isVelocityWithinBounds(ekf.ekfState.dz);
}

static void afinalize(
        const float v0, 
        const float v1, 
        const float v2,
        float A[KC_STATE_DIM][KC_STATE_DIM])
{
    // the attitude error vector (v0,v1,v2) is small,
    // so we use a first order approximation to e0 = tan(|v0|/2)*v0/|v0|
    const auto e0 = v0/2; 
    const auto e1 = v1/2; 
    const auto e2 = v2/2;

    const auto e0e0 =  1 - e1*e1/2 - e2*e2/2;
    const auto e0e1 =  e2 + e0*e1/2;
    const auto e0e2 = -e1 + e0*e2/2;

    const auto e1e0 =  -e2 + e0*e1/2;
    const auto e1e1 = 1 - e0*e0/2 - e2*e2/2;
    const auto e1e2 = e0 + e1*e2/2;

    const auto e2e0 = e1 + e0*e2/2;
    const auto e2e1 = -e0 + e1*e2/2;
    const auto e2e2 = 1 - e0*e0/2 - e1*e1/2;

    const float a[KC_STATE_DIM][KC_STATE_DIM] = 
    { 
        //    Z  DX DY DZ    E0     E1    E2
        /*Z*/   {0, 0, 0, 0, 0,     0,    0},   
        /*DX*/  {0, 1, 0, 0, 0,     0,    0},  
        /*DY*/  {0, 0, 1, 0, 0,     0,    0}, 
        /*DX*/  {0, 0, 0, 1, 0,     0,    0},  
        /*E0*/  {0, 0, 0, 0, e0e0, e0e1, e0e2},
        /*E1*/  {0, 0, 0, 0, e1e0, e1e1, e1e2},
        /*E2*/  {0, 0, 0, 0, e2e0, e2e1, e2e2}
    };

    memcpy(A, a, 7*7*sizeof(float));
} 


static bool doFinalize(ekf_t & ekf)
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = ekf.ekfState.e0;
    const auto v1 = ekf.ekfState.e1;
    const auto v2 = ekf.ekfState.e2;

    const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    const auto ca = cos(angle / 2.0f);
    const auto sa = sin(angle / 2.0f);

    const auto dqw = ca;
    const auto dqx = sa * v0 / angle;
    const auto dqy = sa * v1 / angle;
    const auto dqz = sa * v2 / angle;

    const auto qw = ekf.quat.w;
    const auto qx = ekf.quat.x;
    const auto qy = ekf.quat.y;
    const auto qz = ekf.quat.z;

    // Rotate the quad's attitude by the delta quaternion vector
    // computed above
    const auto tmpq0 = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
    const auto tmpq1 = dqx * qw + dqw * qx + dqz * qy - dqy * qz;
    const auto tmpq2 = dqy * qw - dqz * qx + dqw * qy + dqx * qz;
    const auto tmpq3 = dqz * qw + dqy * qx - dqx * qy + dqw * qz;

    // normalize and store the result
    const auto norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
            tmpq3 * tmpq3) + EPS;

    const auto isErrorSufficient  = 
        (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
        isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

    // finalize()
    ekf.quat.w = isErrorSufficient ? tmpq0 / norm : ekf.quat.w;
    ekf.quat.x = isErrorSufficient ? tmpq1 / norm : ekf.quat.x;
    ekf.quat.y = isErrorSufficient ? tmpq2 / norm : ekf.quat.y;
    ekf.quat.z = isErrorSufficient ? tmpq3 / norm : ekf.quat.z;

    // Move attitude error into attitude if any of the angle errors are
    // large enough
    float A[KC_STATE_DIM][KC_STATE_DIM] = {};
    afinalize(v0, v2, v2, A);
    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'
    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, ekf.p, AP, true);  // AP
    multiply(AP, At, ekf.p, isErrorSufficient); // APA'

    // finalize()
    // Convert the new attitude to a rotation matrix, such that we can
    // rotate body-frame velocity and acc
    ekf.r.x = 2 * ekf.quat.x * ekf.quat.z - 2 * ekf.quat.w * ekf.quat.y;
    ekf.r.y = 2 * ekf.quat.y * ekf.quat.z + 2 * ekf.quat.w * ekf.quat.x;
    ekf.r.z = ekf.quat.w * ekf.quat.w - 
        ekf.quat.x * ekf.quat.x - ekf.quat.y * ekf.quat.y + 
        ekf.quat.z * ekf.quat.z;

    // Reset the attitude error
    // XXX finalize()
    ekf.ekfState.e0 = 0;
    ekf.ekfState.e1 = 0;
    ekf.ekfState.e2 = 0;

    updateCovarianceMatrix(ekf.p, true);

    ekf.isUpdated = false;

    return isStateWithinBounds(ekf);
}


// ===========================================================================

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

static void ekf_predict(
        ekf_t & ekf, const uint32_t nowMsec, const bool isFlying) 
{
    subSamplerFinalize(&ekf.gyroSubSampler, DEGREES_TO_RADIANS);

    const float dt = (nowMsec - ekf.lastPredictionMs) / 1000.0f;

    const auto dt2 = dt * dt;

    subSamplerFinalize(&ekf.accelSubSampler, MSS_TO_GS);

    const Axis3f * acc = &ekf.accelSubSampler.subSample; 

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

    const Axis3f * gyro = &ekf.gyroSubSampler.subSample; 

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

    const auto qw = ekf.quat.w;
    const auto qx = ekf.quat.x;
    const auto qy = ekf.quat.y;
    const auto qz = ekf.quat.z;

    const auto tmpq0 = rotateQuat(
            dqw*qw - dqx*qx - dqy*qy - dqz*qz, QW_INIT, isFlying);

    const auto tmpq1 = rotateQuat(
            dqx*qw + dqw*qx + dqz*qy - dqy*qz, QX_INIT, isFlying);

    const auto tmpq2 = rotateQuat(
            dqy*qw - dqz*qx + dqw*qy + dqx*qz, QY_INIT, isFlying);

    const auto tmpq3 = rotateQuat(
            dqz*qw + dqy*qx - dqx*qy + dqw*qz, QZ_INIT, isFlying);

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
    ekf.ekfState.z += 
        ekf.r.x * dx + ekf.r.y * dy + ekf.r.z * dz - MSS_TO_GS * dt2 / 2;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    ekf.ekfState.dx += 
        dt * (accx + gyro->z * tmpSDY - gyro->y * tmpSDZ - MSS_TO_GS * ekf.r.x);

    ekf.ekfState.dy += 
        dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - MSS_TO_GS * ekf.r.y); 

    ekf.ekfState.dz += 
        dt * (acc->z + gyro->y * tmpSDX - gyro->x * tmpSDY - MSS_TO_GS * ekf.r.z);

    // predict()
    ekf.quat.w = tmpq0/norm;
    ekf.quat.x = tmpq1/norm; 
    ekf.quat.y = tmpq2/norm; 
    ekf.quat.z = tmpq3/norm;

    ekf.isUpdated =  true;

    ekf.lastPredictionMs =  nowMsec;

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
    const auto ze0  = (ekf.ekfState.dy*ekf.r.z - ekf.ekfState.dz*ekf.r.y)*dt;
    const auto ze1  = (- ekf.ekfState.dx*ekf.r.z + ekf.ekfState.dz*ekf.r.x)*dt;
    const auto ze2  = (ekf.ekfState.dx*ekf.r.y - ekf.ekfState.dy*ekf.r.x)*dt;

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
    const auto dye0  = -MSS_TO_GS*ekf.r.z*dt;
    const auto dze0  = MSS_TO_GS*ekf.r.y*dt;

    const auto dxe1  = MSS_TO_GS*ekf.r.z*dt;
    const auto dye1  = 0;
    const auto dze1  = -MSS_TO_GS*ekf.r.x*dt;

    const auto dxe2  = -MSS_TO_GS*ekf.r.y*dt;
    const auto dye2  = MSS_TO_GS*ekf.r.x*dt;
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
    multiply(A, ekf.p, AP, true);  // AP
    multiply(AP, At, ekf.p); // APA'

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

    addNoiseDiagonal(ekf.p, noise, isDtPositive);

    updateCovarianceMatrix(ekf.p, isDtPositive);

    ekf.lastProcessNoiseUpdateMs = isDtPositive ?  nowMsec : 
        ekf.lastProcessNoiseUpdateMs;
}

static void ekf_updateWithGyro(ekf_t & ekf, const Axis3f * gyro)
{
    subSamplerAccumulate(&ekf.gyroSubSampler, gyro);

    memcpy(&ekf.gyroLatest, gyro, sizeof(Axis3f));
}

static void ekf_updateWithAccel(ekf_t & ekf, const Axis3f * accel)
{
    subSamplerAccumulate(&ekf.accelSubSampler, accel);
}

static void ekf_updateWithRange(ekf_t & ekf, const rangeMeasurement_t *range)
{
    const auto angle = max( 0, 
            fabsf(acosf(ekf.r.z)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = ekf.ekfState.z / cosf(angle);
    const auto measuredDistance = range->distance; // [m]

    // The sensor model (Pg.95-96,
    // https://lup.lub.lu.se/student-papers/search/publication/8905295)
    //
    // h = z/((R*z_b).z_b) = z/cos(alpha)
    //
    // Here,
    // h (Measured variable)[m] = Distance given by TOF sensor. This is the 
    // closest point from any surface to the sensor in the measurement cone
    // z (Estimated variable)[m] = THe actual elevation of the crazyflie
    // z_b = Basis vector in z direction of body coordinate system
    // R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be 
    // stationary
    // alpha = angle between [line made by measured point <---> sensor] 
    // and [the intertial z-axis] 

    const float h[KC_STATE_DIM] = {1 / cosf(angle), 0, 0, 0, 0, 0, 0};

    // Only update the filter if the measurement is reliable 
    // (\hat{h} -> infty when R[2][2] -> 0)
    const auto shouldUpdate = fabs(ekf.r.z) > 0.1f && ekf.r.z > 0;
    scalarUpdate(ekf, h, measuredDistance-predictedDistance, 
            range->stdDev, shouldUpdate);
}

static void ekf_updateWithFlow(ekf_t & ekf, const flowMeasurement_t *flow) 
{
    const Axis3f *gyro = &ekf.gyroLatest;

    // Inclusion of flow measurements in the EKF done by two scalar updates

    // ~~~ Camera constants ~~~
    // The angle of aperture is guessed from the raw data register and
    // thankfully look to be symmetric

    float Npix = 35.0;                      // [pixels] (same in x and y)

    // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
    // corresponding ground length
    float thetapix = 0.71674f;

    //~~~ Body rates ~~~
    // TODO check if this is feasible or if some filtering has to be done
    const auto omegax_b = gyro->x * DEGREES_TO_RADIANS;
    const auto omegay_b = gyro->y * DEGREES_TO_RADIANS;

    const auto dx_g = ekf.ekfState.dx;
    const auto dy_g = ekf.ekfState.dy;

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = ekf.ekfState.z < 0.1f ? 0.1f : ekf.ekfState.z;

    // ~~~ X velocity prediction and update ~~~
    // predicts the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = {};
    auto predictedNX = (flow->dt * Npix / thetapix ) * 
        ((dx_g * ekf.r.z / z_g) - omegay_b);
    auto measuredNX = flow->dpixelx*FLOW_RESOLUTION;

    // derive measurement equation with respect to dx (and z?)
    hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
        ((ekf.r.z * dx_g) / (-z_g * z_g));
    hx[KC_STATE_DX] = (Npix * flow->dt / thetapix) * 
        (ekf.r.z / z_g);

    //First update
    scalarUpdate(ekf, hx, (measuredNX-predictedNX), 
            flow->stdDevX*FLOW_RESOLUTION, true);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = {};
    auto predictedNY = (flow->dt * Npix / thetapix ) * 
        ((dy_g * ekf.r.z / z_g) + omegax_b);
    auto measuredNY = flow->dpixely*FLOW_RESOLUTION;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
        ((ekf.r.z * dy_g) / (-z_g * z_g));
    hy[KC_STATE_DY] = (Npix * flow->dt / thetapix) * (ekf.r.z / z_g);

    // Second update
    scalarUpdate(ekf, hy, (measuredNY-predictedNY), 
            flow->stdDevY*FLOW_RESOLUTION, true);
}


static void ekf_getState(ekf_t & ekf, vehicleState_t & state)
{
    state.dx = ekf.ekfState.dx;

    state.dy = ekf.ekfState.dy;

    state.z = ekf.ekfState.z;

    state.dz = 
        ekf.r.x * ekf.ekfState.dx +
        ekf.r.y * ekf.ekfState.dy +
        ekf.r.z * ekf.ekfState.dz;

    const auto qw = ekf.quat.w;
    const auto qx = ekf.quat.x;
    const auto qy = ekf.quat.y;
    const auto qz = ekf.quat.z;

    state.phi = RADIANS_TO_DEGREES * atan2((2 * (qy*qz + qw*qx)),
            (qw*qw - qx*qx - qy*qy + qz*qz));

    // Negate for ENU
    state.theta = -RADIANS_TO_DEGREES * asin((-2) * (qx*qz - qw*qy));

    state.psi = RADIANS_TO_DEGREES * atan2((2 * (qx*qy + qw*qz)),
            (qw*qw + qx*qx - qy*qy - qz*qz));

    // Get angular velocities directly from gyro
    state.dphi =    ekf.gyroLatest.x;
    state.dtheta = -ekf.gyroLatest.y; // negate for ENU
    state.dpsi =    ekf.gyroLatest.z;
}

static bool ekf_finalize(ekf_t & ekf)
{
    // Only finalize if data is updated
    return ekf.isUpdated ? doFinalize(ekf) : isStateWithinBounds(ekf);
}


// ===========================================================================

static bool new_ekf_step(
        const new_ekfAction_e action,
        const uint32_t nowMsec,
        const uint32_t nextPredictionMsec,
        const bool isFlying,
        vehicleState_t & vehicleState)
{
    static ekf_t _ekf;

    bool result = false;

    switch (action) {

        case EKF_INIT:
            ekf_init(_ekf, nowMsec);
            break;

        case EKF_PREDICT:
            if (nowMsec >= nextPredictionMsec) {
                ekf_predict(_ekf, nowMsec, isFlying);
            }
            break;

        case EKF_FINALIZE:
            result = ekf_finalize(_ekf);
            break;

        case EKF_GET_STATE:
            ekf_getState(_ekf, vehicleState);
            break;

        case EKF_UPDATE_WITH_GYRO:
            break;

        case EKF_UPDATE_WITH_ACCEL:
            break;

        case EKF_UPDATE_WITH_FLOW:
            break;

        case EKF_UPDATE_WITH_RANGE:
            break;
    }


    (void)ekf_updateWithGyro;
    (void)ekf_updateWithAccel;
    (void)ekf_updateWithRange;
    (void)ekf_updateWithFlow;

    return result;
}
