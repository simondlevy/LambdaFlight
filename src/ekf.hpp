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

static const float GRAVITY_MAGNITUDE = 9.81;

static const float DEGREES_TO_RADIANS = M_PI / 180.0f;
static const float RADIANS_TO_DEGREES = 180.0f / M_PI;

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

} ekfAction_e;

/**
 * Vehicle State
 *
 * The internally-estimated state is:
 * - Z: the quad's altitude
 * - DX, DY, DZ: the quad's velocity in its body frame
 * - E0, E1, E2: attitude error
 *
 * For more information, refer to the paper
 */         
typedef struct {

    float z;
    float dx;
    float dy;
    float dz;
    float e0;
    float e1;
    float e2;

} ekfState_t;


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

    Axis3f sum;
    uint32_t count;
    float conversionFactor;
    Axis3f subSample;

} Axis3fSubSampler_t;

static const float max(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static void axis3fSubSamplerInit(Axis3fSubSampler_t* subSampler, const
        float conversionFactor) 
{ 
    memset(subSampler, 0, sizeof(Axis3fSubSampler_t));
    subSampler->conversionFactor = conversionFactor;
}

static void axis3fSubSamplerAccumulate(
        const Axis3f & sample, Axis3fSubSampler_t & subSampler)
{
    subSampler.sum.x += sample.x;
    subSampler.sum.y += sample.y;
    subSampler.sum.z += sample.z;

    subSampler.count++;
}

static Axis3f* axis3fSubSamplerFinalize(
        Axis3fSubSampler_t* subSampler,
        const bool shouldPredict) 
{
    const auto count  = subSampler->count; 
    const auto isCountNonzero = count > 0;
    const auto shouldFinalize = shouldPredict && isCountNonzero;

    subSampler->subSample.x = shouldFinalize ? 
        subSampler->sum.x * subSampler->conversionFactor / count :
        subSampler->subSample.x;

    subSampler->subSample.y = shouldFinalize ?
        subSampler->sum.y * subSampler->conversionFactor / count :
        subSampler->subSample.y;

    subSampler->subSample.z = shouldFinalize ?
        subSampler->sum.z * subSampler->conversionFactor / count :
        subSampler->subSample.z;

    // Reset
    subSampler->count = 0;
    subSampler->sum = (Axis3f){.axis={0}};

    return &subSampler->subSample;
}

static void updateCovarianceCell(
        const int i, 
        const int j, 
        const float variance,
        const bool shouldUpdate,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM])
{
    const auto p = (Pmat[i][j] + Pmat[j][i]) / 2 + variance;

    Pmat[i][j] = !shouldUpdate ? Pmat[i][j] :
        (isnan(p) || p > MAX_COVARIANCE) ?  MAX_COVARIANCE :
        (i==j && p < MIN_COVARIANCE) ?  MIN_COVARIANCE :
        p;

    Pmat[j][i] = shouldUpdate ? Pmat[i][j] : Pmat[j][i];
}


static void updateCovarianceMatrix(
        const bool shouldUpdate,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM])
{
    // Enforce symmetry of the covariance matrix, and ensure the
    // values stay bounded
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {
            updateCovarianceCell(i, j, 0, shouldUpdate, Pmat);
        }
    }
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

static bool isPositionWithinBounds(const float pos)
{
    return fabs(pos) < MAX_POSITION;
}

static bool isVelocityWithinBounds(const float vel)
{
    return fabs(vel) < MAX_VELOCITY;
}

static bool isStateWithinBounds(const ekfState_t & ekfState) 
{
    return 
        isPositionWithinBounds(ekfState.z) &&
        isVelocityWithinBounds(ekfState.dx) &&
        isVelocityWithinBounds(ekfState.dy) &&
        isVelocityWithinBounds(ekfState.dz);
}

static bool isErrorLarge(const float v)
{
    return fabs(v) > 0.1e-3f;
}

static bool isErrorInBounds(const float v)
{
    return fabs(v) < 10;
}

static bool doFinalize(
        ekfState_t & ekfState,
        float & qw, 
        float & qx,
        float & qy,
        float & qz,
        float & r20,
        float & r21,
        float & r22,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        bool & isUpdated)
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = ekfState.e0;
    const auto v1 = ekfState.e1;
    const auto v2 = ekfState.e2;

    const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    const auto ca = cos(angle / 2.0f);
    const auto sa = sin(angle / 2.0f);

    const auto dqw = ca;
    const auto dqx = sa * v0 / angle;
    const auto dqy = sa * v1 / angle;
    const auto dqz = sa * v2 / angle;

    // Rotate the quad's attitude by the delta quaternion vector
    // computed above
    const auto tmpq0 = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
    const auto tmpq1 = dqx * qw + dqw * qx + dqz * qy - dqy * qz;
    const auto tmpq2 = dqy * qw - dqz * qx + dqw * qy + dqx * qz;
    const auto tmpq3 = dqz * qw + dqy * qx - dqx * qy + dqw * qz;

    // normalize and store the result
    const auto norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
            tmpq3 * tmpq3) + EPS;

    /** Rotate the covariance, since we've rotated the body
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) 
     Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. 
     d = tan(|v|/2)*v/|v|
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an 
     Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

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

    // Matrix to rotate the attitude covariances once updated
    const float A[KC_STATE_DIM][KC_STATE_DIM] = 
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

    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'

    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, Pmat, AP, true);  // AP

    const auto isErrorSufficient  = 
        (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
        isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

    qw = isErrorSufficient ? tmpq0 / norm : qw;
    qx = isErrorSufficient ? tmpq1 / norm : qx;
    qy = isErrorSufficient ? tmpq2 / norm : qy;
    qz = isErrorSufficient ? tmpq3 / norm : qz;

    // Move attitude error into attitude if any of the angle errors are
    // large enough
    multiply(AP, At, Pmat, isErrorSufficient); // APA'

    // Convert the new attitude to a rotation matrix, such that we can
    // rotate body-frame velocity and acc
    r20 = 2 * qx * qz - 2 * qw * qy;
    r21 = 2 * qy * qz + 2 * qw * qx;
    r22 = qw * qw - qx * qx - qy * qy + qz * qz;

    // Reset the attitude error
    ekfState.e0 = 0;
    ekfState.e1 = 0;
    ekfState.e2 = 0;

    updateCovarianceMatrix(true, Pmat);

    isUpdated = false;

    return isStateWithinBounds(ekfState);
}


static void ekf_init(
        const uint32_t nowMsec,
        Axis3fSubSampler_t & accSubSampler,
        Axis3fSubSampler_t & gyroSubSampler,
        ekfState_t & ekfState,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        float & qw, float & qx, float & qy, float & qz,
        float & r20, float & r21, float & r22,
        bool & isUpdated,
        uint32_t & lastPredictionMsec,
        uint32_t & lastProcessNoiseUpdateMsec
        )
{
    axis3fSubSamplerInit(&accSubSampler, GRAVITY_MAGNITUDE);
    axis3fSubSamplerInit(&gyroSubSampler, DEGREES_TO_RADIANS);

    // Reset all data to 0 (like upon system reset)

    memset(&ekfState, 0, sizeof(ekfState_t));

    // set covariances to zero (diagonals will be changed from
    // zero in the next section)
    memset(Pmat, 0, KC_STATE_DIM*KC_STATE_DIM*sizeof(float));

    ekfState.z = 0;

    qw = QW_INIT;
    qx = QX_INIT;
    qy = QY_INIT;
    qz = QZ_INIT;


    // set the initial rotation matrix to the identity. This only affects
    // the first prediction step, since in the finalization, after shifting
    // attitude errors into the attitude state, the rotation matrix is updated.
    r20 = 0;
    r21 = 0;
    r22 = 1;


    // initialize state variances
    Pmat[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);

    Pmat[KC_STATE_DX][KC_STATE_DX] = powf(STDEV_INITIAL_VELOCITY, 2);
    Pmat[KC_STATE_DY][KC_STATE_DY] = powf(STDEV_INITIAL_VELOCITY, 2);
    Pmat[KC_STATE_DZ][KC_STATE_DZ] = powf(STDEV_INITIAL_VELOCITY, 2);

    Pmat[KC_STATE_E0][KC_STATE_E0] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
    Pmat[KC_STATE_E1][KC_STATE_E1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
    Pmat[KC_STATE_E2][KC_STATE_E2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);

    isUpdated = false;
    lastPredictionMsec = nowMsec;
    lastProcessNoiseUpdateMsec = nowMsec;
}


static void ekf_predict(
        const uint32_t nowMsec, 
        const uint32_t nextPredictionMsec,
        const bool isFlying, 
        const float r20, 
        const float r21,
        const float r22,
        Axis3fSubSampler_t & accSubSampler,
        Axis3fSubSampler_t & gyroSubSampler,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        ekfState_t & ekfState,
        float &qw,
        float &qx,
        float &qy,
        float &qz,
        bool & isUpdated,
        uint32_t & lastPredictionMsec,
        uint32_t & lastProcessNoiseUpdateMsec)

{
    const bool shouldPredict = nowMsec >= nextPredictionMsec;

    axis3fSubSamplerFinalize(&accSubSampler, shouldPredict);

    axis3fSubSamplerFinalize(&gyroSubSampler, shouldPredict);

    const Axis3f * acc = &accSubSampler.subSample; 
    const Axis3f * gyro = &gyroSubSampler.subSample; 
    const float dt = (nowMsec - lastPredictionMsec) / 1000.0f;

    const auto e0 = gyro->x*dt/2;
    const auto e1 = gyro->y*dt/2;
    const auto e2 = gyro->z*dt/2;

    // altitude from body-frame velocity
    const auto zdx = r20*dt;
    const auto zdy = r21*dt;
    const auto zdz = r22*dt;

    // altitude from attitude error
    const auto ze0 = (ekfState.dy*r22 - ekfState.dz*r21)*dt;
    const auto ze1 = (- ekfState.dx*r22 + ekfState.dz*r20)*dt;
    const auto ze2 = (ekfState.dx*r21 - ekfState.dy*r20)*dt;

    // body-frame velocity from body-frame velocity
    const auto dxdx = 1; //drag negligible
    const auto dydx =-gyro->z*dt;
    const auto dzdx = gyro->y*dt;

    const auto dxdy = gyro->z*dt;
    const auto dydy = 1; //drag negligible
    const auto dzdy =-gyro->x*dt;

    const auto dxdz =-gyro->y*dt;
    const auto dydz = gyro->x*dt;
    const auto dzdz = 1; //drag negligible

    // body-frame velocity from attitude error
    const auto dxe0 =  0;
    const auto dye0 = -GRAVITY_MAGNITUDE*r22*dt;
    const auto dze0 =  GRAVITY_MAGNITUDE*r21*dt;

    const auto dxe1 =  GRAVITY_MAGNITUDE*r22*dt;
    const auto dye1 =  0;
    const auto dze1 = -GRAVITY_MAGNITUDE*r20*dt;

    const auto dxe2 = -GRAVITY_MAGNITUDE*r21*dt;
    const auto dye2 =  GRAVITY_MAGNITUDE*r20*dt;
    const auto dze2 =  0;

    const auto e0e0 =  1 - e1*e1/2 - e2*e2/2;
    const auto e0e1 =  e2 + e0*e1/2;
    const auto e0e2 = -e1 + e0*e2/2;

    const auto e1e0 = -e2 + e0*e1/2;
    const auto e1e1 =  1 - e0*e0/2 - e2*e2/2;
    const auto e1e2 =  e0 + e1*e2/2;

    const auto e2e0 =  e1 + e0*e2/2;
    const auto e2e1 = -e0 + e1*e2/2;
    const auto e2e2 = 1 - e0*e0/2 - e1*e1/2;

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

    const auto dt2 = dt * dt;

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = ekfState.dx * dt + isFlying ? 0 : acc->x * dt2 / 2;
    const auto dy = ekfState.dy * dt + isFlying ? 0 : acc->y * dt2 / 2;
    const auto dz = ekfState.dz * dt + acc->z * dt2 / 2; 

    // keep previous time step's state for the update
    const auto tmpSDX = ekfState.dx;
    const auto tmpSDY = ekfState.dy;
    const auto tmpSDZ = ekfState.dz;

    const auto accx = isFlying ? 0 : acc->x;
    const auto accy = isFlying ? 0 : acc->y;

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

    // ====== COVARIANCE UPDATE ======

    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'
    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, Pmat, AP, true);  // AP
    multiply(AP, At, Pmat, shouldPredict); // APA'

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless
    // to estimate body angle while flying)

    // altitude update
    ekfState.z += shouldPredict ? r20 * dx + r21 * dy + r22 * dz - 
        GRAVITY_MAGNITUDE * dt2 / 2 :
        0;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    ekfState.dx += shouldPredict ? 
        dt * (accx + gyro->z * tmpSDY - 
                gyro->y * tmpSDZ - GRAVITY_MAGNITUDE * r20) : 
        0;

    ekfState.dy += shouldPredict ?
        dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - 
                GRAVITY_MAGNITUDE * r21) : 
        0;

    ekfState.dz += shouldPredict ?
        dt * (acc->z + gyro->y * tmpSDX - gyro->x * 
                tmpSDY - GRAVITY_MAGNITUDE * r22) :
        0;

    qw = shouldPredict ? tmpq0/norm : qw;
    qx = shouldPredict ? tmpq1/norm : qx; 
    qy = shouldPredict ? tmpq2/norm : qy; 
    qz = shouldPredict ? tmpq3/norm : qz;

    isUpdated = shouldPredict ? true : isUpdated;

    lastPredictionMsec = shouldPredict ? nowMsec : lastPredictionMsec;

    const auto dt1 = (nowMsec - lastProcessNoiseUpdateMsec) / 1000.0f;

    const auto isDtPositive = dt1 > 0;

    // add process noise on position
    Pmat[KC_STATE_Z][KC_STATE_Z] += isDtPositive ?
        powf(PROC_NOISE_ACC_Z*dt1*dt1 + PROC_NOISE_VEL*dt1 + 
                PROC_NOISE_POS, 2) : 0;  

    // add process noise on velocity
    Pmat[KC_STATE_DX][KC_STATE_DX] += isDtPositive ? 
        powf(PROC_NOISE_ACC_XY*dt1 + 
                PROC_NOISE_VEL, 2) : 0; 

    // add process noise on velocity
    Pmat[KC_STATE_DY][KC_STATE_DY] += isDtPositive ?
        powf(PROC_NOISE_ACC_XY*dt1 + 
                PROC_NOISE_VEL, 2) : 0; 

    // add process noise on velocity
    Pmat[KC_STATE_DZ][KC_STATE_DZ] += isDtPositive ?
        powf(PROC_NOISE_ACC_Z*dt1 + 
                PROC_NOISE_VEL, 2) : 0; 

    Pmat[KC_STATE_E0][KC_STATE_E0] += isDtPositive ?
        powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2) : 0;

    Pmat[KC_STATE_E1][KC_STATE_E1] += isDtPositive ?
        powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2) : 0;

    Pmat[KC_STATE_E2][KC_STATE_E2] += isDtPositive ?
        powf(MEAS_NOISE_GYRO_ROLL_YAW * dt1 + PROC_NOISE_ATT, 2) : 0;

    updateCovarianceMatrix(isDtPositive, Pmat);

    lastProcessNoiseUpdateMsec = isDtPositive ?  nowMsec : 

        lastProcessNoiseUpdateMsec;
}

static void getVehicleState(
        const ekfState_t & ekfState, 
        const Axis3f & gyroLatest,
        const float qw,
        const float qx,
        const float qy,
        const float qz,
        const float r20,
        const float r21,
        const float r22,
        vehicleState_t * vehicleState)
{
    vehicleState->dx = ekfState.dx;

    vehicleState->dy = ekfState.dy;

    vehicleState->z = ekfState.z;

    vehicleState->dz = r20 * ekfState.dx + r21 * ekfState.dy + r22 * ekfState.dz;

    vehicleState->phi = RADIANS_TO_DEGREES * atan2((2 * (qy*qz + qw*qx)),
            (qw*qw - qx*qx - qy*qy + qz*qz));

    // Negate for ENU
    vehicleState->theta = -RADIANS_TO_DEGREES * asin((-2) * (qx*qz - qw*qy));

    vehicleState->psi = RADIANS_TO_DEGREES * atan2((2 * (qx*qy + qw*qz)),
            (qw*qw + qx*qx - qy*qy - qz*qz));

    // Get angular velocities directly from gyro
    vehicleState->dphi =    gyroLatest.x;
    vehicleState->dtheta = -gyroLatest.y; // negate for ENU
    vehicleState->dpsi =    gyroLatest.z;
}

static void scalarUpdate(
        const float h[KC_STATE_DIM],
        const float error, 
        const float stdMeasNoise,
        const bool shouldUpdate,
        ekfState_t & ekfState,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        bool & isUpdated)
{

    // ====== INNOVATION COVARIANCE ======
    float Ph[KC_STATE_DIM] = {};
    multiply(Pmat, h, Ph);
    const auto R = stdMeasNoise * stdMeasNoise;
    auto HPHR = R; // HPH' + R
    for (int i=0; i<KC_STATE_DIM; i++) { 

        // Add the element of HPH' to the above

        // this obviously only works if the update is scalar (as in this function)
        HPHR += h[i]*Ph[i]; 
    }

    // Compute the Kalman gain as a column vector
    const float K[KC_STATE_DIM] = {

        // kalman gain = (PH' (HPH' + R )^-1)
        Ph[0] / HPHR, 
        Ph[1] / HPHR, 
        Ph[2] / HPHR, 
        Ph[3] / HPHR, 
        Ph[4] / HPHR, 
        Ph[5] / HPHR, 
        Ph[6] / HPHR
    };

    // Perform the state update
    ekfState.z  += shouldUpdate ? K[0] * error: 0;
    ekfState.dx += shouldUpdate ? K[1] * error: 0;
    ekfState.dy += shouldUpdate ? K[2] * error: 0;
    ekfState.dz += shouldUpdate ? K[3] * error: 0;
    ekfState.e0 += shouldUpdate ? K[4] * error: 0;
    ekfState.e1 += shouldUpdate ? K[5] * error: 0;
    ekfState.e2 += shouldUpdate ? K[6] * error: 0;

    // ====== COVARIANCE UPDATE ======

    float KH[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(K, h, KH); // KH

    for (int i=0; i<KC_STATE_DIM; i++) { 
        KH[i][i] -= 1;
    } // KH - I

    float KHt[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(KH, KHt);      // (KH - I)'

    float KHIP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(KH, Pmat, KHIP, true);  // (KH - I)*P

    multiply(KHIP, KHt, Pmat, shouldUpdate); // (KH - I)*P*(KH - I)'

    // Add the measurement variance and ensure boundedness and symmetry
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {

            updateCovarianceCell(i, j, K[i] * R * K[j], shouldUpdate, Pmat);
        }
    }

    isUpdated = shouldUpdate ? true : isUpdated;
}


static void updateWithFlow(
        const flowMeasurement_t & flow, 
        const Axis3f & gyro,
        const float r22,
        ekfState_t & ekfState,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        bool & isUpdated)
{
    // Inclusion of flow measurements in the EKF done by two scalar updates

    // ~~~ Camera constants ~~~
    // The angle of aperture is guessed from the raw data register and
    // thankfully look to be symmetric

    const float Npix = 35.0;                      // [pixels] (same in x and y)
    //float thetapix = DEGREES_TO_RADIANS * 4.0f;     // [rad]    (same in x and y)

    // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
    // corresponding ground length
    const float thetapix = 0.71674f;

    //~~~ Body rates ~~~
    // TODO check if this is feasible or if some filtering has to be done
    const auto omegax_b = gyro.x * DEGREES_TO_RADIANS;
    const auto omegay_b = gyro.y * DEGREES_TO_RADIANS;

    const auto dx_g = ekfState.dx;
    const auto dy_g = ekfState.dy;

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = ekfState.z < 0.1f ? 0.1f : ekfState.z;

    // ~~~ X velocity prediction and update ~~~
    // predicts the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = {};
    auto predictedNX = (flow.dt * Npix / thetapix ) * 
        ((dx_g * r22 / z_g) - omegay_b);
    auto measuredNX = flow.dpixelx*FLOW_RESOLUTION;

    // derive measurement equation with respect to dx (and z?)
    hx[KC_STATE_Z] = (Npix * flow.dt / thetapix) * 
        ((r22 * dx_g) / (-z_g * z_g));
    hx[KC_STATE_DX] = (Npix * flow.dt / thetapix) * 
        (r22 / z_g);

    //First update
    scalarUpdate(hx, 
            (measuredNX-predictedNX), 
            flow.stdDevX*FLOW_RESOLUTION, 
            true,
            ekfState,
            Pmat,
            isUpdated);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = {};
    const auto predictedNY = (flow.dt * Npix / thetapix ) * 
        ((dy_g * r22 / z_g) + omegax_b);
    const auto measuredNY = flow.dpixely*FLOW_RESOLUTION;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_Z] = (Npix * flow.dt / thetapix) * 
        ((r22 * dy_g) / (-z_g * z_g));
    hy[KC_STATE_DY] = (Npix * flow.dt / thetapix) * (r22 / z_g);

    // Second update
    scalarUpdate(hy, 
            (measuredNY-predictedNY), 
            flow.stdDevY*FLOW_RESOLUTION, 
            true,
            ekfState,
            Pmat,
            isUpdated);
}

static void updateWithRange(
        const rangeMeasurement_t &range,
        const float r22,
        ekfState_t & ekfState,
        float Pmat[KC_STATE_DIM][KC_STATE_DIM],
        bool & isUpdated)
{
    const auto angle = max( 0, 
            fabsf(acosf(r22)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = ekfState.z / cosf(angle);
    const auto measuredDistance = range.distance; // [m]

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
    const auto shouldUpdate = fabs(r22) > 0.1f && r22 > 0;
    scalarUpdate(
            h, 
            measuredDistance-predictedDistance, 
            range.stdDev, 
            shouldUpdate,
            ekfState,
            Pmat,
            isUpdated);
}


static bool ekf_step(
        const ekfAction_e action,
        const uint32_t nowMsec,
        const uint32_t nextPredictionMsec,
        const bool isFlying,
        const Axis3f * gyro,
        const Axis3f * accel,
        const flowMeasurement_t * flow,
        const rangeMeasurement_t *range,
        vehicleState_t * vehicleState)
{
    // State variables ---------------------------------------------------

    static Axis3fSubSampler_t _accSubSampler;
    static Axis3fSubSampler_t _gyroSubSampler;

    ekfState_t _ekfState;

    // The covariance matrix
    static float _Pmat[KC_STATE_DIM][KC_STATE_DIM];

    // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
    // to allow easy normalization (in comparison to a rotation matrix),
    // while also being robust against singularities (in comparison to euler angles)
    static float _qw;
    static float _qx;
    static float _qy;
    static float _qz;

    // Third row (Z) of attitude as a rotation matrix (used by the prediction,
    // updated by the finalization)
    static float _r20;
    static float _r21;
    static float _r22;

    // Tracks whether an update to the state has been made, and the state
    // therefore requires finalization
    static bool _isUpdated;

    static uint32_t _lastPredictionMsec;
    static uint32_t _lastProcessNoiseUpdateMsec;

    static Axis3f _gyroLatest;

    // -------------------------------------------------------------------

    switch (action) {

        case EKF_INIT:
            ekf_init(nowMsec,
                    _accSubSampler, _gyroSubSampler, 
                    _ekfState, 
                    _Pmat,
                    _qw, _qx, _qy, _qz,
                    _r20, _r21, _r22,
                    _isUpdated, 
                    _lastPredictionMsec,
                    _lastProcessNoiseUpdateMsec
                    );
            break;

        case EKF_PREDICT:
            ekf_predict(nowMsec, 
                    nextPredictionMsec, 
                    isFlying, 
                    _r20, _r21, _r22,
                    _accSubSampler, _gyroSubSampler, 
                    _Pmat,
                    _ekfState,
                    _qw, _qx, _qy, _qz,
                    _isUpdated, 
                    _lastPredictionMsec,
                    _lastProcessNoiseUpdateMsec
                    );
            break;

        case EKF_FINALIZE:
            return _isUpdated ? 
                doFinalize(_ekfState, _qw, _qx, _qy, _qz, _r20, _r21, _r22,
                        _Pmat, _isUpdated) : 
                isStateWithinBounds(_ekfState);

        case EKF_GET_STATE:
            getVehicleState( _ekfState, _gyroLatest, _qw, _qx, _qy, _qz,
                    _r20, _r21, _r22, vehicleState);
            break;

        case EKF_UPDATE_WITH_GYRO:
            axis3fSubSamplerAccumulate(*gyro, _gyroSubSampler);
            memcpy(&_gyroLatest, gyro, sizeof(Axis3f));
            break;

        case EKF_UPDATE_WITH_ACCEL:
            axis3fSubSamplerAccumulate(*accel, _accSubSampler);
            break;

        case EKF_UPDATE_WITH_FLOW:
            updateWithFlow(*flow, *gyro, _r22, _ekfState, _Pmat, _isUpdated);
            break;

        case EKF_UPDATE_WITH_RANGE:
            updateWithRange(*range, _r22, _ekfState, _Pmat, _isUpdated);
            break;
    }

    return false;
}

class Ekf { 

    public:

        void init(const uint32_t nowMs)
        {
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY_MAGNITUDE);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            // Reset all data to 0 (like upon system reset)

            memset(&_ekfState, 0, sizeof(_ekfState));
            memset(_Pmat, 0, sizeof(_Pmat));

            _ekfState.z = 0;

            _qw = QW_INIT;
            _qx = QX_INIT;
            _qy = QY_INIT;
            _qz = QZ_INIT;

            // set the initial rotation matrix to the identity. This only affects
            // the first prediction step, since in the finalization, after shifting
            // attitude errors into the attitude state, the rotation matrix is updated.
            _r20 = 0;
            _r21 = 0;
            _r22 = 1;

            // set covariances to zero (diagonals will be changed from
            // zero in the next section)
            memset(_Pmat, 0, sizeof(_Pmat));

            // initialize state variances
            _Pmat[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);

            _Pmat[KC_STATE_DX][KC_STATE_DX] = powf(STDEV_INITIAL_VELOCITY, 2);
            _Pmat[KC_STATE_DY][KC_STATE_DY] = powf(STDEV_INITIAL_VELOCITY, 2);
            _Pmat[KC_STATE_DZ][KC_STATE_DZ] = powf(STDEV_INITIAL_VELOCITY, 2);

            _Pmat[KC_STATE_E0][KC_STATE_E0] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
            _Pmat[KC_STATE_E1][KC_STATE_E1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
            _Pmat[KC_STATE_E2][KC_STATE_E2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);

            _isUpdated = false;
            _lastPredictionMs = nowMs;
            _lastProcessNoiseUpdateMs = nowMs;
        }

        void predict(
                const uint32_t nowMs, 
                const uint32_t nextPredictionMs,
                const bool isFlying) 
        {
            const bool shouldPredict = nowMs >= nextPredictionMs;

            axis3fSubSamplerFinalize(&_accSubSampler, shouldPredict);

            axis3fSubSamplerFinalize(&_gyroSubSampler, shouldPredict);

            const Axis3f * acc = &_accSubSampler.subSample; 
            const Axis3f * gyro = &_gyroSubSampler.subSample; 
            const float dt = (nowMs - _lastPredictionMs) / 1000.0f;

            const auto e0 = gyro->x*dt/2;
            const auto e1 = gyro->y*dt/2;
            const auto e2 = gyro->z*dt/2;

            // altitude from body-frame velocity
            const auto zdx = _r20*dt;
            const auto zdy = _r21*dt;
            const auto zdz = _r22*dt;

            // altitude from attitude error
            const auto ze0 = (_ekfState.dy*_r22 - _ekfState.dz*_r21)*dt;
            const auto ze1 = (- _ekfState.dx*_r22 + _ekfState.dz*_r20)*dt;
            const auto ze2 = (_ekfState.dx*_r21 - _ekfState.dy*_r20)*dt;

            // body-frame velocity from body-frame velocity
            const auto dxdx = 1; //drag negligible
            const auto dydx =-gyro->z*dt;
            const auto dzdx = gyro->y*dt;

            const auto dxdy = gyro->z*dt;
            const auto dydy = 1; //drag negligible
            const auto dzdy =-gyro->x*dt;

            const auto dxdz =-gyro->y*dt;
            const auto dydz = gyro->x*dt;
            const auto dzdz = 1; //drag negligible

            // body-frame velocity from attitude error
            const auto dxe0 =  0;
            const auto dye0 = -GRAVITY_MAGNITUDE*_r22*dt;
            const auto dze0 =  GRAVITY_MAGNITUDE*_r21*dt;

            const auto dxe1 =  GRAVITY_MAGNITUDE*_r22*dt;
            const auto dye1 =  0;
            const auto dze1 = -GRAVITY_MAGNITUDE*_r20*dt;

            const auto dxe2 = -GRAVITY_MAGNITUDE*_r21*dt;
            const auto dye2 =  GRAVITY_MAGNITUDE*_r20*dt;
            const auto dze2 =  0;

            const auto e0e0 =  1 - e1*e1/2 - e2*e2/2;
            const auto e0e1 =  e2 + e0*e1/2;
            const auto e0e2 = -e1 + e0*e2/2;

            const auto e1e0 = -e2 + e0*e1/2;
            const auto e1e1 =  1 - e0*e0/2 - e2*e2/2;
            const auto e1e2 =  e0 + e1*e2/2;

            const auto e2e0 =  e1 + e0*e2/2;
            const auto e2e1 = -e0 + e1*e2/2;
            const auto e2e2 = 1 - e0*e0/2 - e1*e1/2;

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


            const auto dt2 = dt * dt;

            // Position updates in the body frame (will be rotated to inertial frame);
            // thrust can only be produced in the body's Z direction
            const auto dx = _ekfState.dx * dt + isFlying ? 0 : acc->x * dt2 / 2;
            const auto dy = _ekfState.dy * dt + isFlying ? 0 : acc->y * dt2 / 2;
            const auto dz = _ekfState.dz * dt + acc->z * dt2 / 2; 

            // keep previous time step's state for the update
            const auto tmpSDX = _ekfState.dx;
            const auto tmpSDY = _ekfState.dy;
            const auto tmpSDZ = _ekfState.dz;

            const auto accx = isFlying ? 0 : acc->x;
            const auto accy = isFlying ? 0 : acc->y;

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

            // ====== COVARIANCE UPDATE ======

            float At[KC_STATE_DIM][KC_STATE_DIM] = {};
            transpose(A, At);     // A'
            float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
            multiply(A, _Pmat, AP, true);  // AP
            multiply(AP, At, _Pmat, shouldPredict); // APA'

            // Process noise is added after the return from the prediction step

            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust (hence is useless
            // to estimate body angle while flying)

            // altitude update
            _ekfState.z += shouldPredict ? _r20 * dx + _r21 * dy + _r22 * dz - 
                GRAVITY_MAGNITUDE * dt2 / 2 :
                0;

            // body-velocity update: accelerometers - gyros cross velocity
            // - gravity in body frame

            _ekfState.dx += shouldPredict ? 
                dt * (accx + gyro->z * tmpSDY - 
                        gyro->y * tmpSDZ - GRAVITY_MAGNITUDE * _r20) : 
                0;

            _ekfState.dy += shouldPredict ?
                dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - 
                        GRAVITY_MAGNITUDE * _r21) : 
                0;

            _ekfState.dz += shouldPredict ?
                dt * (acc->z + gyro->y * tmpSDX - gyro->x * 
                        tmpSDY - GRAVITY_MAGNITUDE * _r22) :
                0;

            _qw = shouldPredict ? tmpq0/norm : _qw;
            _qx = shouldPredict ? tmpq1/norm : _qx; 
            _qy = shouldPredict ? tmpq2/norm : _qy; 
            _qz = shouldPredict ? tmpq3/norm : _qz;

            _isUpdated = shouldPredict ? true : _isUpdated;

            _lastPredictionMs = shouldPredict ? nowMs : _lastPredictionMs;

            const auto dt1 = (nowMs - _lastProcessNoiseUpdateMs) / 1000.0f;

            const auto isDtPositive = dt1 > 0;

            // add process noise on position
            _Pmat[KC_STATE_Z][KC_STATE_Z] += isDtPositive ?
                powf(PROC_NOISE_ACC_Z*dt1*dt1 + PROC_NOISE_VEL*dt1 + 
                        PROC_NOISE_POS, 2) : 0;  

            // add process noise on velocity
            _Pmat[KC_STATE_DX][KC_STATE_DX] += isDtPositive ? 
                powf(PROC_NOISE_ACC_XY*dt1 + 
                        PROC_NOISE_VEL, 2) : 0; 

            // add process noise on velocity
            _Pmat[KC_STATE_DY][KC_STATE_DY] += isDtPositive ?
                powf(PROC_NOISE_ACC_XY*dt1 + 
                        PROC_NOISE_VEL, 2) : 0; 

            // add process noise on velocity
            _Pmat[KC_STATE_DZ][KC_STATE_DZ] += isDtPositive ?
                powf(PROC_NOISE_ACC_Z*dt1 + 
                        PROC_NOISE_VEL, 2) : 0; 

            _Pmat[KC_STATE_E0][KC_STATE_E0] += isDtPositive ?
                powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2) : 0;

            _Pmat[KC_STATE_E1][KC_STATE_E1] += isDtPositive ?
                powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT, 2) : 0;

            _Pmat[KC_STATE_E2][KC_STATE_E2] += isDtPositive ?
                powf(MEAS_NOISE_GYRO_ROLL_YAW * dt1 + PROC_NOISE_ATT, 2) : 0;

            updateCovarianceMatrix(isDtPositive);

            _lastProcessNoiseUpdateMs = isDtPositive ?  nowMs : 

                _lastProcessNoiseUpdateMs;
        }

        void updateWithRange(const rangeMeasurement_t *range)
        {
            const auto angle = max( 0, 
                    fabsf(acosf(_r22)) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f));

            const auto predictedDistance = _ekfState.z / cosf(angle);
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
            const auto shouldUpdate = fabs(_r22) > 0.1f && _r22 > 0;
            scalarUpdate(h, measuredDistance-predictedDistance, 
                    range->stdDev, shouldUpdate);
        }

        void updateWithAccel(const Axis3f & accel)
        {
            axis3fSubSamplerAccumulate(accel, _accSubSampler);
        }

        void updateWithGyro(const Axis3f & gyro)
        {
            axis3fSubSamplerAccumulate(gyro, _gyroSubSampler);
            memcpy(&_gyroLatest, &gyro, sizeof(Axis3f));
        }

        void updateWithFlow(const flowMeasurement_t *flow) 
        {
            const Axis3f *gyro = &_gyroLatest;

            // Inclusion of flow measurements in the EKF done by two scalar updates

            // ~~~ Camera constants ~~~
            // The angle of aperture is guessed from the raw data register and
            // thankfully look to be symmetric

            float Npix = 35.0;                      // [pixels] (same in x and y)
            //float thetapix = DEGREES_TO_RADIANS * 4.0f;     // [rad]    (same in x and y)

            // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
            // corresponding ground length
            float thetapix = 0.71674f;

            //~~~ Body rates ~~~
            // TODO check if this is feasible or if some filtering has to be done
            const auto omegax_b = gyro->x * DEGREES_TO_RADIANS;
            const auto omegay_b = gyro->y * DEGREES_TO_RADIANS;

            const auto dx_g = _ekfState.dx;
            const auto dy_g = _ekfState.dy;

            // Saturate elevation in prediction and correction to avoid singularities
            const auto z_g = _ekfState.z < 0.1f ? 0.1f : _ekfState.z;

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            float hx[KC_STATE_DIM] = {};
            auto predictedNX = (flow->dt * Npix / thetapix ) * 
                ((dx_g * _r22 / z_g) - omegay_b);
            auto measuredNX = flow->dpixelx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dx_g) / (-z_g * z_g));
            hx[KC_STATE_DX] = (Npix * flow->dt / thetapix) * 
                (_r22 / z_g);

            //First update
            scalarUpdate(hx, (measuredNX-predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION, true);

            // ~~~ Y velocity prediction and update ~~~
            float hy[KC_STATE_DIM] = {};
            auto predictedNY = (flow->dt * Npix / thetapix ) * 
                ((dy_g * _r22 / z_g) + omegax_b);
            auto measuredNY = flow->dpixely*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dy_g) / (-z_g * z_g));
            hy[KC_STATE_DY] = (Npix * flow->dt / thetapix) * (_r22 / z_g);

            // Second update
            scalarUpdate(hy, (measuredNY-predictedNY), 
                    flow->stdDevY*FLOW_RESOLUTION, true);
        }

        void getState(vehicleState_t & state)
        {
            state.dx = _ekfState.dx;

            state.dy = _ekfState.dy;

            state.z = _ekfState.z;

            state.dz = 
                _r20 * _ekfState.dx +
                _r21 * _ekfState.dy +
                _r22 * _ekfState.dz;

            state.phi = RADIANS_TO_DEGREES * atan2((2 * (_qy*_qz + _qw*_qx)),
                    (_qw*_qw - _qx*_qx - _qy*_qy + _qz*_qz));

            // Negate for ENU
            state.theta = -RADIANS_TO_DEGREES * asin((-2) * (_qx*_qz - _qw*_qy));

            state.psi = RADIANS_TO_DEGREES * atan2((2 * (_qx*_qy + _qw*_qz)),
                    (_qw*_qw + _qx*_qx - _qy*_qy - _qz*_qz));

            // Get angular velocities directly from gyro
            state.dphi =    _gyroLatest.x;
            state.dtheta = -_gyroLatest.y; // negate for ENU
            state.dpsi =    _gyroLatest.z;
        }

        bool finalize(void)
        {
            // Only finalize if data is updated
            return _isUpdated ? doFinalize() : isStateWithinBounds();
        }

    private:

        // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
        // to allow easy normalization (in comparison to a rotation matrix),
        // while also being robust against singularities (in comparison to euler angles)
        float _qw;
        float _qx;
        float _qy;
        float _qz;

        //////////////////////////////////////////////////////////////////////////

        Axis3f _gyroLatest;

        Axis3fSubSampler_t _accSubSampler;
        Axis3fSubSampler_t _gyroSubSampler;

        ekfState_t _ekfState;

        // Third row (Z) of attitude as a rotation matrix (used by the prediction,
        // updated by the finalization)
        float _r20;
        float _r21;
        float _r22;

        // The covariance matrix
        float _Pmat[KC_STATE_DIM][KC_STATE_DIM];

        // Tracks whether an update to the state has been made, and the state
        // therefore requires finalization
        bool _isUpdated;

        uint32_t _lastPredictionMs;
        uint32_t _lastProcessNoiseUpdateMs;

        //////////////////////////////////////////////////////////////////////////

        bool doFinalize(void)
        {
            // Incorporate the attitude error (Kalman filter state) with the attitude
            const auto v0 = _ekfState.e0;
            const auto v1 = _ekfState.e1;
            const auto v2 = _ekfState.e2;

            const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
            const auto ca = cos(angle / 2.0f);
            const auto sa = sin(angle / 2.0f);

            const auto dqw = ca;
            const auto dqx = sa * v0 / angle;
            const auto dqy = sa * v1 / angle;
            const auto dqz = sa * v2 / angle;

            // Rotate the quad's attitude by the delta quaternion vector
            // computed above
            const auto tmpq0 = dqw * _qw - dqx * _qx - dqy * _qy - dqz * _qz;
            const auto tmpq1 = dqx * _qw + dqw * _qx + dqz * _qy - dqy * _qz;
            const auto tmpq2 = dqy * _qw - dqz * _qx + dqw * _qy + dqx * _qz;
            const auto tmpq3 = dqz * _qw + dqy * _qx - dqx * _qy + dqw * _qz;

            // normalize and store the result
            const auto norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                    tmpq3 * tmpq3) + EPS;

            /** Rotate the covariance, since we've rotated the body
             *
             * This comes from a second order approximation to:
             * Sigma_post = exps(-d) Sigma_pre exps(-d)'
             *            ~ (I + [[-d]] + [[-d]]^2 / 2) 
             Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
             * where d is the attitude error expressed as Rodriges parameters, ie. 
             d = tan(|v|/2)*v/|v|
             *
             * As derived in "Covariance Correction Step for Kalman Filtering with an 
             Attitude"
             * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
             */

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

            // Matrix to rotate the attitude covariances once updated
            const float A[KC_STATE_DIM][KC_STATE_DIM] = 
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

            float At[KC_STATE_DIM][KC_STATE_DIM] = {};
            transpose(A, At);     // A'

            float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
            multiply(A, _Pmat, AP, true);  // AP

            const auto isErrorSufficient  = 
                (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
                isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

            _qw = isErrorSufficient ? tmpq0 / norm : _qw;
            _qx = isErrorSufficient ? tmpq1 / norm : _qx;
            _qy = isErrorSufficient ? tmpq2 / norm : _qy;
            _qz = isErrorSufficient ? tmpq3 / norm : _qz;

            // Move attitude error into attitude if any of the angle errors are
            // large enough
            multiply(AP, At, _Pmat, isErrorSufficient); // APA'

            // Convert the new attitude to a rotation matrix, such that we can
            // rotate body-frame velocity and acc
            _r20 = 2 * _qx * _qz - 2 * _qw * _qy;
            _r21 = 2 * _qy * _qz + 2 * _qw * _qx;
            _r22 = _qw * _qw - _qx * _qx - _qy * _qy + _qz * _qz;

            // Reset the attitude error
            _ekfState.e0 = 0;
            _ekfState.e1 = 0;
            _ekfState.e2 = 0;

            updateCovarianceMatrix(true);

            _isUpdated = false;

            return isStateWithinBounds();
        }

        static void axis3fSubSamplerInit(Axis3fSubSampler_t* subSampler, const
                float conversionFactor) 
        { 
            memset(subSampler, 0, sizeof(Axis3fSubSampler_t));
            subSampler->conversionFactor = conversionFactor;
        }

        void updateCovarianceMatrix(const bool shouldUpdate)
        {
            // Enforce symmetry of the covariance matrix, and ensure the
            // values stay bounded
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    updateCovarianceCell(i, j, 0, shouldUpdate);
                }
            }
        }

        void scalarUpdate(
                const float h[KC_STATE_DIM],
                const float error, 
                const float stdMeasNoise,
                const bool shouldUpdate)
        {

            // ====== INNOVATION COVARIANCE ======
            float Ph[KC_STATE_DIM] = {};
            multiply(_Pmat, h, Ph);
            const auto R = stdMeasNoise * stdMeasNoise;
            auto HPHR = R; // HPH' + R
            for (int i=0; i<KC_STATE_DIM; i++) { 

                // Add the element of HPH' to the above

                // this obviously only works if the update is scalar (as in this function)
                HPHR += h[i]*Ph[i]; 
            }

            // Compute the Kalman gain as a column vector
            const float K[KC_STATE_DIM] = {

                // kalman gain = (PH' (HPH' + R )^-1)
                Ph[0] / HPHR, 
                Ph[1] / HPHR, 
                Ph[2] / HPHR, 
                Ph[3] / HPHR, 
                Ph[4] / HPHR, 
                Ph[5] / HPHR, 
                Ph[6] / HPHR
            };

            // Perform the state update
            _ekfState.z  += shouldUpdate ? K[0] * error: 0;
            _ekfState.dx += shouldUpdate ? K[1] * error: 0;
            _ekfState.dy += shouldUpdate ? K[2] * error: 0;
            _ekfState.dz += shouldUpdate ? K[3] * error: 0;
            _ekfState.e0 += shouldUpdate ? K[4] * error: 0;
            _ekfState.e1 += shouldUpdate ? K[5] * error: 0;
            _ekfState.e2 += shouldUpdate ? K[6] * error: 0;

            // ====== COVARIANCE UPDATE ======

            float KH[KC_STATE_DIM][KC_STATE_DIM] = {};
            multiply(K, h, KH); // KH

            for (int i=0; i<KC_STATE_DIM; i++) { 
                KH[i][i] -= 1;
            } // KH - I

            float KHt[KC_STATE_DIM][KC_STATE_DIM] = {};
            transpose(KH, KHt);      // (KH - I)'

            float KHIP[KC_STATE_DIM][KC_STATE_DIM] = {};
            multiply(KH, _Pmat, KHIP, true);  // (KH - I)*P

            multiply(KHIP, KHt, _Pmat, shouldUpdate); // (KH - I)*P*(KH - I)'

            // Add the measurement variance and ensure boundedness and symmetry
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {

                    updateCovarianceCell(i, j, K[i] * R * K[j], shouldUpdate);
                }
            }

            _isUpdated = shouldUpdate ? true : _isUpdated;
        }

        void updateCovarianceCell(
                const int i, 
                const int j, 
                const float variance,
                const bool shouldUpdate)
        {
            const auto p = (_Pmat[i][j] + _Pmat[j][i]) / 2 + variance;

            _Pmat[i][j] = !shouldUpdate ? _Pmat[i][j] :
                (isnan(p) || p > MAX_COVARIANCE) ?  MAX_COVARIANCE :
                (i==j && p < MIN_COVARIANCE) ?  MIN_COVARIANCE :
                p;

            _Pmat[j][i] = shouldUpdate ? _Pmat[i][j] : _Pmat[j][i];
        }

       bool isStateWithinBounds(void) 
        {
            return 
                isPositionWithinBounds(_ekfState.z) &&
                isVelocityWithinBounds(_ekfState.dx) &&
                isVelocityWithinBounds(_ekfState.dy) &&
                isVelocityWithinBounds(_ekfState.dz);
        }
};
