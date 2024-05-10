/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include <ekf.h>
#include <math3d.h>

typedef struct {

    float w;
    float x;
    float y;
    float z;

} new_quat_t;

typedef struct {

    axis3_t sum;
    uint32_t count;

} imu_t;

axis3_t _gyroLatest;

new_quat_t _quat;

uint32_t _nextPredictionMsec;

axis3_t _r;

imu_t _gyroSum;
imu_t _accelSum;

// Indexes to access the state
enum {

    STATE_Z,
    STATE_DX,
    STATE_DY,
    STATE_DZ,
    STATE_E0,
    STATE_E1,
    STATE_E2
};

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

// ~~~ Camera constants ~~~
// The angle of aperture is guessed from the raw data register and
// thankfully look to be symmetric

static const float FLOW_NPIX = 35.0;   // [pixels] (same in x and y)

// 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
// corresponding ground length
static const float FLOW_THETAPIX = 0.71674;

static const float MSS_TO_GS = 9.81;

//We do get the measurements in 10x the motion pixels (experimentally measured)
static const float FLOW_RESOLUTION = 0.1;

// The bounds on states, these shouldn't be hit...
static const float MAX_POSITION = 100; //meters
static const float MAX_VELOCITY = 10; //meters per second

// Small number epsilon, to prevent dividing by zero
static const float EPS = 1e-6f;

// the reversion of pitch and roll to zero
static const float ROLLPITCH_ZERO_REVERSION = 0.001;

static const uint16_t RANGEFINDER_OUTLIER_LIMIT_MM = 5000;

// Rangefinder measurement noise model
static const float RANGEFINDER_EXP_POINT_A = 2.5;
static const float RANGEFINDER_EXP_STD_A = 0.0025; 
static const float RANGEFINDER_EXP_POINT_B = 4.0;
static const float RANGEFINDER_EXP_STD_B = 0.2;   

static const float RANGEFINDER_EXP_COEFF = 
logf( RANGEFINDER_EXP_STD_B / RANGEFINDER_EXP_STD_A) / 
(RANGEFINDER_EXP_POINT_B - RANGEFINDER_EXP_POINT_A);

static const float FLOW_STD_FIXED = 2.0;

static void imuAccum(const axis3_t vals, imu_t & imu)
{
    imu.sum.x += vals.x;
    imu.sum.y += vals.y;
    imu.sum.z += vals.z;
    imu.count++;
}

static void imuTakeMean(
        const imu_t & imu, 
        const float conversionFactor, 
        axis3_t & mean)
{
    const auto count = imu.count;

    const auto isCountNonzero = count > 0;

    mean.x = isCountNonzero ? imu.sum.x * conversionFactor / count : mean.x;
    mean.y = isCountNonzero ? imu.sum.y * conversionFactor / count : mean.y;
    mean.z = isCountNonzero ? imu.sum.z * conversionFactor / count : mean.z;
}

static const float max(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static const float min(const float val, const float maxval)
{
    return val < maxval ? maxval : val;
}

static const float square(const float x)
{
    return x * x;
}

static float rotateQuat(const float val, const float initVal)
{
    return (val * (1 - ROLLPITCH_ZERO_REVERSION)) + 
        (ROLLPITCH_ZERO_REVERSION * initVal);
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

//////////////////////////////////////////////////////////////////////////////


void getFlowUpdates(
        const float * x,
        const float dt, 
        const float dpixelx, 
        const float dpixely,
        float hx[7], 
        float & errx, 
        float hy[7], 
        float & erry, 
        float & stdev)
{
    // Inclusion of flow measurements in the EKF done by two scalar updates

    //~~~ Body rates ~~~
    const auto omegay_b = _gyroLatest.y * DEGREES_TO_RADIANS;

    const auto dx_g = x[STATE_DX];

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = x[STATE_Z] < 0.1f ? 0.1f : x[STATE_Z];

    // ~~~ X velocity prediction and update ~~~
    // predicts the number of accumulated pixels in the x-direction
    auto predictedNX = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
        ((dx_g * _r.z / z_g) - omegay_b);
    auto measuredNX = dpixelx*FLOW_RESOLUTION;

    // derive measurement equation with respect to dx (and z?)
    hx[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dx_g) / (-z_g * z_g));
    hx[1] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

    // Inclusion of flow measurements in the EKF done by two scalar updates

    //~~~ Body rates ~~~
    const auto omegax_b = _gyroLatest.x * DEGREES_TO_RADIANS;

    const auto dy_g = x[STATE_DY];

    // ~~~ Y velocity prediction and update ~~~
    auto predictedNY = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
        ((dy_g * _r.z / z_g) + omegax_b);
    auto measuredNY = dpixely*FLOW_RESOLUTION;

    // derive measurement equation with respect to dy (and z?)
    hy[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dy_g) / (-z_g * z_g));
    hy[2] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

    errx = measuredNX - predictedNX;
    erry = measuredNY - predictedNY;

    stdev = FLOW_STD_FIXED*FLOW_RESOLUTION;
}

void accumulateGyro(const uint32_t nowMsec, const axis3_t & gyro) 
{
    imuAccum(gyro, _gyroSum);

    memcpy(&_gyroLatest, &gyro, sizeof(axis3_t));
}

void accumulateAccel(const uint32_t nowMsec, const axis3_t & accel) 
{
    imuAccum(accel, _accelSum);
}

void ekf_getVehicleState(const float * x, vehicleState_t & state)
{
    state.dx = x[STATE_DX];

    state.dy = x[STATE_DY];

    state.z = x[STATE_Z];

    state.z = min(0, state.z);

    state.dz = _r.x * x[STATE_DX] + _r.y * x[STATE_DY] + 
        _r.z * x[STATE_DZ];

    // Pack Z and DZ into a single float for transmission to client
    const int8_t sgn = state.dz < 0 ? -1 : +1;
    const float s = 1000;
    state.z_dz = (int)(state.dz * s) + sgn * state.z / s;

    const auto qw = _quat.w;
    const auto qx = _quat.x;
    const auto qy = _quat.y;
    const auto qz = _quat.z;

    state.phi = RADIANS_TO_DEGREES * atan2((2 * (qy*qz + qw*qx)),
            (qw*qw - qx*qx - qy*qy + qz*qz));

    // Negate for ENU
    state.theta = -RADIANS_TO_DEGREES * asin((-2) * (qx*qz - qw*qy));

    state.psi = RADIANS_TO_DEGREES * atan2((2 * (qx*qy + qw*qz)),
            (qw*qw + qx*qx - qy*qy - qz*qz));

    // Get angular velocities directly from gyro
    state.dphi =    _gyroLatest.x;
    state.dtheta = -_gyroLatest.y; // negate for ENU
    state.dpsi =    _gyroLatest.z;
}

bool isStateWithinBounds(const float * x)
{
    return
        isPositionWithinBounds(x[STATE_Z]) &&
        isVelocityWithinBounds(x[STATE_DX]) &&
        isVelocityWithinBounds(x[STATE_DY]) &&
        isVelocityWithinBounds(x[STATE_DZ]);
}


bool shouldUpdateWithRange(const float * x, const uint32_t distance,
        float h[7], float & error, float & noise)
{
    const auto angle = max(0, 
            fabsf(acosf(_r.z)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = x[STATE_Z] / cosf(angle);

    const auto measuredDistance = distance / 1000.f; // mm => m

    h[0] = 1/cosf(angle);

    noise = RANGEFINDER_EXP_STD_A * 
        (1 + expf(RANGEFINDER_EXP_COEFF * 
                  (measuredDistance - RANGEFINDER_EXP_POINT_A)));

    error = measuredDistance-predictedDistance;

    return fabs(_r.z) > 0.1f && _r.z > 0 && 
        distance < RANGEFINDER_OUTLIER_LIMIT_MM;
}


//////////////////////////////////////////////////////////////////////////////

void TinyEkf::initialize_covariance_diagonal(float diag[EKF_N])
{
    diag[STATE_Z] = square(STDEV_INITIAL_POSITION_Z);
    diag[STATE_DX] = square(STDEV_INITIAL_VELOCITY);
    diag[STATE_DY] = square(STDEV_INITIAL_VELOCITY);
    diag[STATE_DZ] = square(STDEV_INITIAL_VELOCITY);
    diag[STATE_E0] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
    diag[STATE_E1] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
    diag[STATE_E2] = square(STDEV_INITIAL_ATTITUDE_YAW);

    _quat.w = QW_INIT;
    _quat.x = QX_INIT;
    _quat.y = QY_INIT;
    _quat.z = QZ_INIT;

    _r.x = 0;
    _r.y = 0;
    _r.z = 0;
}


bool TinyEkf::did_finalize(float x[EKF_N], float A[EKF_N][EKF_N])
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = x[STATE_E0];
    const auto v1 = x[STATE_E1];
    const auto v2 = x[STATE_E2];

    const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    const auto ca = cos(angle / 2.0f);
    const auto sa = sin(angle / 2.0f);

    const auto dqw = ca;
    const auto dqx = sa * v0 / angle;
    const auto dqy = sa * v1 / angle;
    const auto dqz = sa * v2 / angle;

    const auto qw = _quat.w;
    const auto qx = _quat.x;
    const auto qy = _quat.y;
    const auto qz = _quat.z;

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

    _quat.w = isErrorSufficient ? tmpq0 / norm : _quat.w;
    _quat.x = isErrorSufficient ? tmpq1 / norm : _quat.x;
    _quat.y = isErrorSufficient ? tmpq2 / norm : _quat.y;
    _quat.z = isErrorSufficient ? tmpq3 / norm : _quat.z;

    x[STATE_E0] = 0;
    x[STATE_E1] = 0;
    x[STATE_E2] = 0;

    _r.x = 2 * _quat.x * _quat.z - 2 * _quat.w * _quat.y;
    _r.y = 2 * _quat.y * _quat.z + 2 * _quat.w * _quat.x; 
    _r.z = _quat.w*_quat.w-_quat.x*_quat.x-_quat.y*_quat.y+_quat.z*_quat.z;

    // the attitude error vector (v0,v1,v2) is small,
    // so we use a first order approximation to e0 = tan(|v0|/2)*v0/|v0|
    const auto e0 = v0 / 2; 
    const auto e1 = v1 / 2; 
    const auto e2 = v2 / 2;

    A[STATE_DX][STATE_DX] = 1;
    A[STATE_DY][STATE_DY] = 1;
    A[STATE_DZ][STATE_DZ] = 1;

    A[STATE_E0][STATE_E0] =  1 - e1*e1/2 - e2*e2/2;
    A[STATE_E0][STATE_E1] =  e2 + e0*e1/2;
    A[STATE_E0][STATE_E2] = -e1 + e0*e2/2;

    A[STATE_E1][STATE_E0] =  -e2 + e0*e1/2;
    A[STATE_E1][STATE_E1] = 1 - e0*e0/2 - e2*e2/2;
    A[STATE_E1][STATE_E2] = e0 + e1*e2/2;

    A[STATE_E2][STATE_E0] = e1 + e0*e2/2;
    A[STATE_E2][STATE_E1] = -e0 + e1*e2/2;
    A[STATE_E2][STATE_E2] = 1 - e0*e0/2 - e1*e1/2;

    return isErrorSufficient;
}

void TinyEkf::get_prediction(
        const float dt,
        const bool didAddProcessNoise,
        const float xold[EKF_N],
        float xnew[EKF_N],
        float F[EKF_N][EKF_N])
{

    static axis3_t _gyro;
    static axis3_t _accel;

    const auto dt2 = dt * dt;

    imuTakeMean(_gyroSum, DEGREES_TO_RADIANS, _gyro);
    imuTakeMean(_accelSum, MSS_TO_GS, _accel);

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = xold[STATE_DX] * dt + _accel.x * dt2 / 2;
    const auto dy = xold[STATE_DY] * dt + _accel.y * dt2 / 2;
    const auto dz = xold[STATE_DZ] * dt + _accel.z * dt2 / 2; 

    const auto accx = _accel.x;
    const auto accy = _accel.y;

    // attitude update (rotate by gyroscope), we do this in quaternions
    // this is the gyroscope angular velocity integrated over the sample period
    const auto dtwx = dt*_gyro.x;
    const auto dtwy = dt*_gyro.y;
    const auto dtwz = dt*_gyro.z;

    // compute the quaternion values in [w,x,y,z] order
    const auto angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
    const auto ca = cos(angle/2);
    const auto sa = sin(angle/2);
    const auto dqw = ca;
    const auto dqx = sa*dtwx/angle;
    const auto dqy = sa*dtwy/angle;
    const auto dqz = sa*dtwz/angle;

    // rotate the quad's attitude by the delta quaternion vector
    // computed above

    const auto qw = _quat.w;
    const auto qx = _quat.x;
    const auto qy = _quat.y;
    const auto qz = _quat.z;

    const auto tmpq0 = rotateQuat(
            dqw*qw - dqx*qx - dqy*qy - dqz*qz, QW_INIT);
    const auto tmpq1 = rotateQuat(
            dqx*qw + dqw*qx + dqz*qy - dqy*qz, QX_INIT);
    const auto tmpq2 = rotateQuat(
            dqy*qw - dqz*qx + dqw*qy + dqx*qz, QY_INIT);
    const auto tmpq3 = rotateQuat(
            dqz*qw + dqy*qx - dqx*qy + dqw*qz, QZ_INIT);

    // normalize and store the result
    const auto norm = 
        sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
        EPS;

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust
    // (hence is useless to estimate body angle while flying)

    const auto tmpSDX = xold[STATE_DX];
    const auto tmpSDY = xold[STATE_DY];
    const auto tmpSDZ = xold[STATE_DZ];

    xnew[STATE_Z] = xold[STATE_Z] + 
        _r.x * dx + _r.y * dy + _r.z * dz - MSS_TO_GS * dt2 / 2;

    xnew[STATE_DX] = xold[STATE_DX] +
        dt * (accx + _gyro.z * tmpSDY - _gyro.y * tmpSDZ -
                MSS_TO_GS * _r.x);

    xnew[STATE_DY] =
        xold[STATE_DY] +
        dt * (accy - _gyro.z * tmpSDX + _gyro.x * tmpSDZ -
                MSS_TO_GS * _r.y); 

    xnew[STATE_DZ] =
        xold[STATE_DZ] +
        dt * (_accel.z + _gyro.y * tmpSDX - _gyro.x * tmpSDY -
                MSS_TO_GS * _r.z); 

    new_quat_t quat_predicted = {};

    quat_predicted.w = tmpq0/norm;
    quat_predicted.x = tmpq1/norm; 
    quat_predicted.y = tmpq2/norm; 
    quat_predicted.z = tmpq3/norm;

    const auto e0 = _gyro.x*dt/2;
    const auto e1 = _gyro.y*dt/2;
    const auto e2 = _gyro.z*dt/2;

    F[STATE_E0][STATE_E0] =  1 - e1*e1/2 - e2*e2/2;
    F[STATE_E0][STATE_E1] =  e2 + e0*e1/2;
    F[STATE_E0][STATE_E2] = -e1 + e0*e2/2;

    F[STATE_E1][STATE_E0] =  -e2 + e0*e1/2;
    F[STATE_E1][STATE_E1] = 1 - e0*e0/2 - e2*e2/2;
    F[STATE_E1][STATE_E2] = e0 + e1*e2/2;

    F[STATE_E2][STATE_E0] = e1 + e0*e2/2;
    F[STATE_E2][STATE_E1] = -e0 + e1*e2/2;
    F[STATE_E2][STATE_E2] = 1 - e0*e0/2 - e1*e1/2;

    // altitude from body-frame velocity
    F[STATE_Z][STATE_DX] = _r.x*dt;
    F[STATE_Z][STATE_DY] = _r.y*dt;
    F[STATE_Z][STATE_DZ] = _r.z*dt;

    // altitude from attitude error
    F[STATE_Z][STATE_E0] = (xnew[ STATE_DY]*_r.z -
            xnew[ STATE_DZ]*_r.y)*dt;

    F[STATE_Z][STATE_E1] = (- xnew[ STATE_DX]*_r.z +
            xnew[ STATE_DZ]*_r.x)*dt;

    F[STATE_Z][STATE_E2] = (xnew[ STATE_DX]*_r.y -
            xnew[ STATE_DY]*_r.x)*dt;

    // body-frame velocity from body-frame velocity
    F[STATE_DX][STATE_DX] = 1; //drag negligible
    F[STATE_DY][STATE_DX] =  -_gyro.z*dt;
    F[STATE_DZ][STATE_DX] = _gyro.y*dt;

    F[STATE_DX][STATE_DY] = _gyro.z*dt;
    F[STATE_DY][STATE_DY] = 1; //drag negligible
    F[STATE_DZ][STATE_DY] = _gyro.x*dt;

    F[STATE_DX][STATE_DZ] =  _gyro.y*dt;
    F[STATE_DY][STATE_DZ] = _gyro.x*dt;
    F[STATE_DZ][STATE_DZ] = 1; //drag negligible

    // body-frame velocity from attitude error
    F[STATE_DX][STATE_E0] = 0;
    F[STATE_DY][STATE_E0] = -MSS_TO_GS*_r.z*dt;
    F[STATE_DZ][STATE_E0] = MSS_TO_GS*_r.y*dt;

    F[STATE_DX][STATE_E1] = MSS_TO_GS*_r.z*dt;
    F[STATE_DY][STATE_E1] = 0;
    F[STATE_DZ][STATE_E1] = -MSS_TO_GS*_r.x*dt;

    F[STATE_DX][STATE_E2] = -MSS_TO_GS*_r.y*dt;
    F[STATE_DY][STATE_E2] = MSS_TO_GS*_r.x*dt;
    F[STATE_DZ][STATE_E2] = 0;

    if (didAddProcessNoise) {

        _quat.w = quat_predicted.w;
        _quat.x = quat_predicted.x;
        _quat.y = quat_predicted.y;
        _quat.z = quat_predicted.z;

        memset(&_gyroSum, 0, sizeof(_gyroSum));
        memset(&_accelSum, 0, sizeof(_accelSum));
    }
}

