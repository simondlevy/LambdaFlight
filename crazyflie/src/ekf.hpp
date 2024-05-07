#pragma once

#include <string.h>

#include <clock.hpp>
#include <console.h>
#include <math3d.h>
#include <datatypes.h>
#include <streams.h>

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

static const uint16_t RANGEFINDER_OUTLIER_LIMIT_MM = 5000;

// Rangefinder measurement noise model
static constexpr float RANGEFINDER_EXP_POINT_A = 2.5;
static constexpr float RANGEFINDER_EXP_STD_A = 0.0025; 
static constexpr float RANGEFINDER_EXP_POINT_B = 4.0;
static constexpr float RANGEFINDER_EXP_STD_B = 0.2;   

static constexpr float RANGEFINDER_EXP_COEFF = 
logf( RANGEFINDER_EXP_STD_B / RANGEFINDER_EXP_STD_A) / 
(RANGEFINDER_EXP_POINT_B - RANGEFINDER_EXP_POINT_A);

static constexpr float FLOW_STD_FIXED = 2.0;

// this is slower than the IMU update rate of 1000Hz
static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

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

typedef struct {

    float dat[EKF_N];

} newvec_t;

typedef struct {

    float dat[EKF_N][EKF_N];

} matrix_t;

static void makemat(const float dat[EKF_N][EKF_N], matrix_t & a)
{
    for (uint8_t i=0; i<EKF_N; ++i) {
        for (uint8_t j=0; j<EKF_N; ++j) {
            a.dat[i][j] = dat[i][j];
        }
    }

}

static void transpose(const matrix_t & a, matrix_t & at)
{
    for (uint8_t i=0; i<EKF_N; ++i) {
        for (uint8_t j=0; j<EKF_N; ++j) {
            auto tmp = a.dat[i][j];
            at.dat[i][j] = a.dat[j][i];
            at.dat[j][i] = tmp;
        }
    }
}

static float dot(const float x[EKF_N], const float y[EKF_N]) 
{
    float d = 0;

    for (uint8_t k=0; k<EKF_N; k++) {
        d += x[k] * y[k];
    }

    return d;
}

/*
static float dot(const newvec_t & x, const newvec_t & y) 
{
    float d = 0;

    for (uint8_t k=0; k<EKF_N; k++) {
        d += x.dat[k] * y.dat[k];
    }

    return d;
}*/

static float get(const newvec_t & x, const uint8_t i)
{
    return x.dat[i];
}

static void set(newvec_t & x, const uint8_t i, const float val)
{
    x.dat[i] = val;
}

static float dot(
        const matrix_t & a, 
        const matrix_t & b, 
        const uint8_t i, 
        const uint8_t j)
{
    float d = 0;

    for (uint8_t k=0; k<EKF_N; k++) {
        d += a.dat[i][k] * b.dat[k][j];
    }

    return d;
}

// Matrix * Matrix
static void multiply( const matrix_t a, const matrix_t b, matrix_t & c)
{
    for (uint8_t i=0; i<EKF_N; i++) {

        for (uint8_t j=0; j<EKF_N; j++) {

            c.dat[i][j] = dot(a, b, i, j);
        }
    }
}

// Matrix * Vector
static void multiply(const float a[EKF_N][EKF_N], const float x[EKF_N], float y[EKF_N])
{
    for (uint8_t i=0; i<EKF_N; i++) {
        y[i] = 0;
        for (uint8_t j=0; j<EKF_N; j++) {
            y[i] += a[i][j] * x[j];
        }
    }
}

// Outer product
static void multiply(const float x[EKF_N], const float y[EKF_N], float a[EKF_N][EKF_N])
{
    for (uint8_t i=0; i<EKF_N; i++) {
        for (uint8_t j=0; j<EKF_N; j++) {
            a[i][j] = x[i] * y[j];
        }
    }
}


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

static float rotateQuat( const float val, const float initVal)
{
    const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

    return (val * (stream_isFlying ? 1: keep)) +
        (stream_isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
}

static void updateCovarianceMatrix(matrix_t & p)
{
    // Enforce symmetry of the covariance matrix, and ensure the
    // values stay bounded
    for (int i=0; i<EKF_N; i++) {

        for (int j=i; j<EKF_N; j++) {

            const auto pval = (p.dat[i][j] + p.dat[j][i]) / 2;

            p.dat[i][j] = p.dat[j][i] = 
                pval > MAX_COVARIANCE ?  MAX_COVARIANCE :
                (i==j && pval < MIN_COVARIANCE) ?  MIN_COVARIANCE :
                pval;
        }
    }
}

static void scalarUpdate(
        const float h[EKF_N],
        const float error, 
        const float stdMeasNoise,
        matrix_t & p,
        float x[EKF_N])
{

    // ====== INNOVATION COVARIANCE ======
    float ph[EKF_N] = {};
    multiply(p.dat, h, ph);
    const auto r = stdMeasNoise * stdMeasNoise;
    const auto hphr = r + dot(h, ph); // HPH' + R

    // Compute the Kalman gain as a column vector
    float g[EKF_N] = {};
    for (uint8_t i=0; i<EKF_N; ++i) {
       g[i] = ph[i] / hphr;
    }

    // Perform the state update
    for (uint8_t i=0; i<EKF_N; ++i) {
        x[i] += g[i] * error;
    }

    // ====== COVARIANCE UPDATE ======

    matrix_t GH = {};
    multiply(g, h, GH.dat); // KH

    for (int i=0; i<EKF_N; i++) { 
        GH.dat[i][i] -= 1;
    } // KH - I

    matrix_t GHt = {};
    transpose(GH, GHt);      // (KH - I)'
    matrix_t GHIP = {};
    multiply(GH, p, GHIP);  // (KH - I)*P
    multiply(GHIP, GHt, p); // (KH - I)*P*(KH - I)'

    // Add the measurement variance 
    for (int i=0; i<EKF_N; i++) {
        for (int j=0; j<EKF_N; j++) {
            p.dat[i][j] += j < i ? 0 : r * g[i] * g[j];
        }
    }

    updateCovarianceMatrix(p);
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

static void afinalize(
        const float v0, 
        const float v1, 
        const float v2,
        matrix_t & A)
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

    const float a[EKF_N][EKF_N] = 
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

    memcpy(&A.dat, a, sizeof(A));
} 


// ===========================================================================

static void ekf_init(matrix_t & p, newvec_t & x)
{
    memset(&p, 0, sizeof(p));
    p.dat[STATE_Z][STATE_Z] = square(STDEV_INITIAL_POSITION_Z);
    p.dat[STATE_DX][STATE_DX] = square(STDEV_INITIAL_VELOCITY);
    p.dat[STATE_DY][STATE_DY] = square(STDEV_INITIAL_VELOCITY);
    p.dat[STATE_DZ][STATE_DZ] = square(STDEV_INITIAL_VELOCITY);
    p.dat[STATE_E0][STATE_E0] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
    p.dat[STATE_E1][STATE_E1] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
    p.dat[STATE_E2][STATE_E2] = square(STDEV_INITIAL_ATTITUDE_YAW);

    memset(&x, 0, sizeof(x));
}

static void ekf_predict(
        const imu_t & gyro,
        const imu_t & accel,
        const newvec_t & x_in,
        const new_quat_t & q,
        const axis3_t & r,
        const uint32_t lastPredictionMsec, 
        new_quat_t & quat_out,
        matrix_t & p,
        newvec_t &x_out)
{
    static axis3_t _gyro;
    static axis3_t _accel;

    const float dt = (stream_nowMsec - lastPredictionMsec) / 1000.0f;
    const auto dt2 = dt * dt;

    imuTakeMean(gyro, DEGREES_TO_RADIANS, _gyro);
    imuTakeMean(accel, MSS_TO_GS, _accel);

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = get(x_in, STATE_DX) * dt + stream_isFlying ? 0 : _accel.x * dt2 / 2;
    const auto dy = get(x_in, STATE_DY) * dt + stream_isFlying ? 0 : _accel.y * dt2 / 2;
    const auto dz = get(x_in, STATE_DZ) * dt + _accel.z * dt2 / 2; 

    const auto accx = stream_isFlying ? 0 : _accel.x;
    const auto accy = stream_isFlying ? 0 : _accel.y;

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

    // rotate the quad's attitude by the delta quaternion vector computed above

    const auto tmpq0 = rotateQuat(dqw*q.w - dqx*q.x - dqy*q.y - dqz*q.z, QW_INIT);
    const auto tmpq1 = rotateQuat(dqx*q.w + dqw*q.x + dqz*q.y - dqy*q.z, QX_INIT);
    const auto tmpq2 = rotateQuat(dqy*q.w - dqz*q.x + dqw*q.y + dqx*q.z, QY_INIT);
    const auto tmpq3 = rotateQuat(dqz*q.w + dqy*q.x - dqx*q.y + dqw*q.z, QZ_INIT);

    // normalize and store the result
    const auto norm = 
        sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
        EPS;

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless
    // to estimate body angle while flying)

    const auto tmpSDX = get(x_in, STATE_DX);
    const auto tmpSDY = get(x_in, STATE_DY);
    const auto tmpSDZ = get(x_in, STATE_DZ);

    set(x_out, STATE_Z, 
            get(x_in, STATE_Z) + r.x * dx + r.y * dy + r.z * dz - MSS_TO_GS * dt2 / 2);

    set(x_out, STATE_DX,
        get(x_in, STATE_DX) +
            dt * (accx + _gyro.z * tmpSDY - _gyro.y * tmpSDZ - MSS_TO_GS * r.x));

    set(x_out, STATE_DY,
        get(x_in, STATE_DY) +
            dt * (accy - _gyro.z * tmpSDX + _gyro.x * tmpSDZ - MSS_TO_GS * r.y)); 

    set(x_out, STATE_DZ,
        get(x_in, STATE_DZ) +
            dt * (_accel.z + _gyro.y * tmpSDX - _gyro.x * tmpSDY - MSS_TO_GS * r.z));

    // predict()
    quat_out.w = tmpq0/norm;
    quat_out.x = tmpq1/norm; 
    quat_out.y = tmpq2/norm; 
    quat_out.z = tmpq3/norm;

    // ====== COVARIANCE UPDATE ======

    const auto e0 = _gyro.x*dt/2;
    const auto e1 = _gyro.y*dt/2;
    const auto e2 = _gyro.z*dt/2;

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
    const auto zdx  = r.x*dt;
    const auto zdy  = r.y*dt;
    const auto zdz  = r.z*dt;

    // altitude from attitude error
    const auto ze0  = (get(x_out, STATE_DY)*r.z - get(x_out, STATE_DZ)*r.y)*dt;
    const auto ze1  = (- get(x_out, STATE_DX)*r.z + get(x_out, STATE_DZ)*r.x)*dt;
    const auto ze2  = (get(x_out, STATE_DX)*r.y - get(x_out, STATE_DY)*r.x)*dt;

    // body-frame velocity from body-frame velocity
    const auto dxdx  = 1; //drag negligible
    const auto dydx =  -_gyro.z*dt;
    const auto dzdx  = _gyro.y*dt;

    const auto dxdy  = _gyro.z*dt;
    const auto dydy  = 1; //drag negligible
    const auto dzdy  = _gyro.x*dt;

    const auto dxdz =  _gyro.y*dt;
    const auto dydz  = _gyro.x*dt;
    const auto dzdz  = 1; //drag negligible

    // body-frame velocity from attitude error
    const auto dxe0  = 0;
    const auto dye0  = -MSS_TO_GS*r.z*dt;
    const auto dze0  = MSS_TO_GS*r.y*dt;

    const auto dxe1  = MSS_TO_GS*r.z*dt;
    const auto dye1  = 0;
    const auto dze1  = -MSS_TO_GS*r.x*dt;

    const auto dxe2  = -MSS_TO_GS*r.y*dt;
    const auto dye2  = MSS_TO_GS*r.x*dt;
    const auto dze2  = 0;

    const float Adat[EKF_N][EKF_N] = 
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

    matrix_t A = {};
    makemat(Adat, A);

    matrix_t  At = {};
    transpose(A, At);     // A'
    matrix_t AP = {};
    multiply(A, p, AP);  // AP
    multiply(AP, At, p); // APA'
    updateCovarianceMatrix(p);
}

static void ekf_updateWithRange(const float rz, matrix_t & p, float x[EKF_N])
{
    const auto angle = max(0, 
            fabsf(acosf(rz)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = x[STATE_Z] / cosf(angle);

    const auto measuredDistance = stream_rangefinder_distance / 1000; // mm => m

    const auto stdDev =
        RANGEFINDER_EXP_STD_A * 
        (1 + expf(RANGEFINDER_EXP_COEFF * (measuredDistance - RANGEFINDER_EXP_POINT_A)));


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

    const float h[EKF_N] = {1 / cosf(angle), 0, 0, 0, 0, 0, 0};

    scalarUpdate(
            h , 
            measuredDistance-predictedDistance, 
            stdDev, 
            p,
            x);
}

static void ekf_updateWithFlow(
        const float rz,
        const axis3_t & gyroLatest,
        matrix_t & p,
        float x[EKF_N])
{
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
    const auto omegax_b = gyroLatest.x * DEGREES_TO_RADIANS;
    const auto omegay_b = gyroLatest.y * DEGREES_TO_RADIANS;

    const auto dx_g = x[STATE_DX];
    const auto dy_g = x[STATE_DY];

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = x[STATE_Z] < 0.1f ? 0.1f : x[STATE_Z];

    // ~~~ X velocity prediction and update ~~~
    // predicts the number of accumulated pixels in the x-direction
    float hx[EKF_N] = {};
    auto predictedNX = (stream_flow.dt * Npix / thetapix ) * 
        ((dx_g * rz / z_g) - omegay_b);
    auto measuredNX = stream_flow.dpixelx*FLOW_RESOLUTION;

    // derive measurement equation with respect to dx (and z?)
    hx[STATE_Z] = (Npix * stream_flow.dt / thetapix) * 
        ((rz * dx_g) / (-z_g * z_g));
    hx[STATE_DX] = (Npix * stream_flow.dt / thetapix) * 
        (rz / z_g);

    //First update
    scalarUpdate(
            hx, 
            measuredNX-predictedNX, 
            FLOW_STD_FIXED*FLOW_RESOLUTION, 
            p,
            x);

    // ~~~ Y velocity prediction and update ~~~
    float hy[EKF_N] = {};
    auto predictedNY = (stream_flow.dt * Npix / thetapix ) * 
        ((dy_g * rz / z_g) + omegax_b);
    auto measuredNY = stream_flow.dpixely*FLOW_RESOLUTION;

    // derive measurement equation with respect to dy (and z?)
    hy[STATE_Z] = (Npix * stream_flow.dt / thetapix) * 
        ((rz * dy_g) / (-z_g * z_g));
    hy[STATE_DY] = (Npix * stream_flow.dt / thetapix) * (rz / z_g);

    // Second update
    scalarUpdate(
            hy, 
            measuredNY-predictedNY, 
            FLOW_STD_FIXED*FLOW_RESOLUTION, 
            p,
            x);
}

static void ekf_finalize(
        const new_quat_t & q,
        matrix_t & p,
        newvec_t & x,
        new_quat_t & q_out)
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = get(x, STATE_E0);
    const auto v1 = get(x, STATE_E1);
    const auto v2 = get(x, STATE_E2);

    const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    const auto ca = cos(angle / 2.0f);
    const auto sa = sin(angle / 2.0f);

    const auto dqw = ca;
    const auto dqx = sa * v0 / angle;
    const auto dqy = sa * v1 / angle;
    const auto dqz = sa * v2 / angle;

    // Rotate the quad's attitude by the delta quaternion vector
    // computed above
    const auto tmpq0 = dqw * q.w - dqx * q.x - dqy * q.y - dqz * q.z;
    const auto tmpq1 = dqx * q.w + dqw * q.x + dqz * q.y - dqy * q.z;
    const auto tmpq2 = dqy * q.w - dqz * q.x + dqw * q.y + dqx * q.z;
    const auto tmpq3 = dqz * q.w + dqy * q.x - dqx * q.y + dqw * q.z;

    // normalize and store the result
    const auto norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
            tmpq3 * tmpq3) + EPS;

    const auto isErrorSufficient  = 
        (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
        isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

    // finalize()
    q_out.w = isErrorSufficient ? tmpq0 / norm : q.w;
    q_out.x = isErrorSufficient ? tmpq1 / norm : q.x;
    q_out.y = isErrorSufficient ? tmpq2 / norm : q.y;
    q_out.z = isErrorSufficient ? tmpq3 / norm : q.z;

    // Move attitude error into attitude if any of the angle errors are
    // large enough
    if (isErrorSufficient) {
        matrix_t  A = {};
        afinalize(v0, v2, v2, A);
        matrix_t At = {};
        transpose(A, At);     // A'
        matrix_t AP = {};
        multiply(A, p, AP);  // AP
        multiply(AP, At, p); // APA'
        updateCovarianceMatrix(p);
    }

    set(x, STATE_E0, 0);
    set(x, STATE_E1, 0);
    set(x, STATE_E2, 0);
} 

static void ekf_getVehicleState(
        const float x[EKF_N],
        const axis3_t & gyroLatest,
        const new_quat_t & q,
        const axis3_t & r,
        vehicleState_t & state)
{
    state.dx = x[STATE_DX];

    state.dy = x[STATE_DY];

    state.z = stream_rangefinder_distance / 1000; 

    state.z = min(0, state.z);

    state.dz = r.x * x[STATE_DX] + r.y * x[STATE_DY] + r.z * x[STATE_DZ];

    // Pack Z and DZ into a single float for transmission to client
    const int8_t sgn = state.dz < 0 ? -1 : +1;
    const float s = 1000;
    state.z_dz = (int)(state.dz * s) + sgn * state.z / s;

    state.phi = RADIANS_TO_DEGREES * atan2((2 * (q.y*q.z + q.w*q.x)),
            (q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z));

    // Negate for ENU
    state.theta = -RADIANS_TO_DEGREES * asin((-2) * (q.x*q.z - q.w*q.y));

    state.psi = RADIANS_TO_DEGREES * atan2((2 * (q.x*q.y + q.w*q.z)),
            (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z));

    // Get angular velocities directly from gyro
    state.dphi =    gyroLatest.x;
    state.dtheta = -gyroLatest.y; // negate for ENU
    state.dpsi =    gyroLatest.z;
}

// ===========================================================================

static void ekf_step(void)
{
    static matrix_t _p;
    static newvec_t _x;

    static bool _isUpdated;
    static uint32_t _lastPredictionMsec;
    static uint32_t _lastProcessNoiseUpdateMsec;

    static axis3_t _gyroLatest;

    static new_quat_t _quat;

    static uint32_t _nextPredictionMsec;

    static axis3_t _r;

    static imu_t _gyro;
    static imu_t _accel;


    // Initialize ------------------------------------------------------------

    if (stream_ekfAction == EKF_INIT) {

        ekf_init(_p, _x);

        _quat.w = 1;
        _quat.x = 0;
        _quat.y = 0;
        _quat.z = 0;

        _r.x = 0;
        _r.y = 0;
        _r.z = 0;

        _lastProcessNoiseUpdateMsec = stream_nowMsec;
        _lastPredictionMsec = stream_nowMsec;
        _isUpdated = false;
    }

    _nextPredictionMsec = stream_nowMsec > _nextPredictionMsec ?
        stream_nowMsec + PREDICTION_UPDATE_INTERVAL_MS :
        _nextPredictionMsec;

    // Predict ---------------------------------------------------------------

    const auto requestedPredict = stream_ekfAction == EKF_PREDICT;

    const auto predicting =
        requestedPredict && stream_nowMsec >= _nextPredictionMsec;

    if (requestedPredict) {
        _isUpdated = true;
    }

    auto updatingProcessNoise = false;

    if (predicting) {

        newvec_t x_predicted = {};

        new_quat_t quat_predicted = {};

        ekf_predict( _gyro, _accel, _x, _quat, _r, _lastPredictionMsec, 
                quat_predicted, _p, x_predicted);

        _lastPredictionMsec = stream_nowMsec;

        if (stream_nowMsec - _lastProcessNoiseUpdateMsec > 0) {

            _lastProcessNoiseUpdateMsec = stream_nowMsec;

            set(_x, STATE_Z , get(x_predicted, STATE_Z));
            set(_x, STATE_DX , get(x_predicted, STATE_DX));
            set(_x, STATE_DY , get(x_predicted, STATE_DY));
            set(_x, STATE_DZ , get(x_predicted, STATE_DZ));

            _quat.w = quat_predicted.w;
            _quat.x = quat_predicted.x;
            _quat.y = quat_predicted.y;
            _quat.z = quat_predicted.z;

            memset(&_gyro, 0, sizeof(_gyro));
            memset(&_accel, 0, sizeof(_gyro));

            updatingProcessNoise = true;
        }
    }

    // Update ----------------------------------------------------------------

    // Update with flow
    if (stream_ekfAction == EKF_UPDATE_WITH_FLOW) {
        ekf_updateWithFlow(_r.z, _gyroLatest, _p, _x.dat);
        _isUpdated = true;
    }

    // Update with range when the measurement is reliable 
    if (stream_ekfAction == EKF_UPDATE_WITH_RANGE &&
            fabs(_r.z) > 0.1f && _r.z > 0 && 
            stream_rangefinder_distance < RANGEFINDER_OUTLIER_LIMIT_MM) {
        ekf_updateWithRange(_r.z, _p, _x.dat);
        _isUpdated = true;
    }

    // Update with gyro
    if (stream_ekfAction == EKF_UPDATE_WITH_GYRO) {

        imuAccum(stream_gyro, _gyro);

        memcpy(&_gyroLatest, &stream_gyro, sizeof(axis3_t));
    }

    // Update with accel
    if (stream_ekfAction == EKF_UPDATE_WITH_ACCEL) {

        imuAccum(stream_accel, _accel);
    }

    // Finalize --------------------------------------------------------------

    const auto requestedFinalize = stream_ekfAction == EKF_FINALIZE;

    if (requestedFinalize && _isUpdated) {

        new_quat_t quat_finalized = {};

        ekf_finalize(_quat, _p, _x, quat_finalized);

        _quat.w = quat_finalized.w;
        _quat.x = quat_finalized.x;
        _quat.y = quat_finalized.y;
        _quat.z = quat_finalized.z;

        _r.x = 2 * _quat.x * _quat.z - 2 * _quat.w * _quat.y;
        _r.y = 2 * _quat.y * _quat.z + 2 * _quat.w * _quat.x; 
        _r.z = _quat.w*_quat.w-_quat.x*_quat.x-_quat.y*_quat.y+_quat.z*_quat.z;

        _isUpdated = false;
    }

    // Get vehicle state -----------------------------------------------------

    vehicleState_t vehicleState = {};
    ekf_getVehicleState(_x.dat, _gyroLatest, _quat, _r, vehicleState);

    if (stream_ekfAction == EKF_GET_STATE) {
        setState(vehicleState);
    }

    if (requestedFinalize) {
        setStateIsInBounds(
                isPositionWithinBounds(_x.dat[STATE_Z]) &&
                isVelocityWithinBounds(_x.dat[STATE_DX]) &&
                isVelocityWithinBounds(_x.dat[STATE_DY]) &&
                isVelocityWithinBounds(_x.dat[STATE_DZ]));
    }
}
