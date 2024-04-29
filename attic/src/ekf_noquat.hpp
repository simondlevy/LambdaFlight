#pragma once

#include <string.h>

#include <math3d.h>
#include <datatypes.h>
#include <streams.h>

static const uint8_t N = 4;

static void transpose(const float a[N][N], float at[N][N])
{
    for (uint8_t i=0; i<N; ++i) {
        for (uint8_t j=0; j<N; ++j) {
            auto tmp = a[i][j];
            at[i][j] = a[j][i];
            at[j][i] = tmp;
        }
    }
}

static float dot(const float x[N], const float y[N]) 
{
    float d = 0;

    for (uint8_t k=0; k<N; k++) {
        d += x[k] * y[k];
    }

    return d;
}

static float dot(const float a[N][N], const float b[N][N], 
        const uint8_t i, const uint8_t j)
{
    float d = 0;

    for (uint8_t k=0; k<N; k++) {
        d += a[i][k] * b[k][j];
    }

    return d;
}

// Matrix * Matrix
static void multiply(const float a[N][N], const float b[N][N], float c[N][N])
{
    for (uint8_t i=0; i<N; i++) {

        for (uint8_t j=0; j<N; j++) {

            c[i][j] = dot(a, b, i, j);
        }
    }
}

// Matrix * Vector
static void multiply(const float a[N][N], const float x[N], float y[N])
{
    for (uint8_t i=0; i<N; i++) {
        y[i] = 0;
        for (uint8_t j=0; j<N; j++) {
            y[i] += a[i][j] * x[j];
        }
    }
}

// Outer product
static void multiply(const float x[N], const float y[N], float a[N][N])
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t j=0; j<N; j++) {
            a[i][j] = x[i] * y[j];
        }
    }
}

#ifdef _TEST

static void show(const float a[N][N])
{
    for (uint8_t i=0; i<N; ++i) {

        for (uint8_t j=0; j<N; ++j) {
            printf("%3.0f ", (double)a[i][j]);
        }

        printf("\n");
    }
}

static void show(const float x[N])
{
    for (uint8_t i=0; i<N; ++i) {
        printf("%3.0f ", (double)x[i]);
    }
    printf("\n");
}
#endif
// Initial variances, uncertain of position, but know we're
// stationary and roughly flat
static const float STDEV_INITIAL_POSITION_Z = 1;
static const float STDEV_INITIAL_VELOCITY = 0.01;

static const float MSS_TO_GS = 9.81;

//We do get the measurements in 10x the motion pixels (experimentally measured)
static const float FLOW_RESOLUTION = 0.1;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
static const float MAX_COVARIANCE = 100;
static const float MIN_COVARIANCE = 1e-6;

// The bounds on states, these shouldn't be hit...
static const float MAX_POSITION = 100; //meters
static const float MAX_VELOCITY = 10; //meters per second

// Indexes to access the state
enum {

    KC_STATE_Z,
    KC_STATE_DX,
    KC_STATE_DY,
    KC_STATE_DZ,
    KC_STATE_DIM
};

typedef struct {

    float dat[KC_STATE_DIM][KC_STATE_DIM];

} matrix_t;

typedef struct {

    float z;
    float dx;
    float dy;
    float dz;

} ekfState_t;

static float invSqrt(const float x)
{
    return 1 / sqrt(x);
}


static void madgwick(
        const float gx,  const float gy, const float gz,
        const float ax,  const float ay, const float az,
        const float invSampFreq,
        float & q0, float & q1, float & q2, float & q3)
{

  // Tunable parameter
  const float b_madgwick = 0.04;

  // Normalize accelerometer measurement
  const auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  const auto aax = ax * recipNorm;
  const auto aay = ay * recipNorm;
  const auto aaz = az * recipNorm;

  // Rate of change of quaternion from gyroscope
  const auto qDot1 = 0.5f * ((-q1) * gx - q2 * gy - q3 * gz);
  const auto qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  const auto qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  const auto qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Auxiliary variables to avoid repeated arithmetic
  const auto _2q0 = 2 * q0;
  const auto _2q1 = 2 * q1;
  const auto _2q2 = 2 * q2;
  const auto _2q3 = 2 * q3;
  const auto _4q0 = 4 * q0;
  const auto _4q1 = 4 * q1;
  const auto _4q2 = 4 * q2;
  const auto _8q1 = 8 * q1;
  const auto _8q2 = 8 * q2;
  const auto q0q0 = q0 * q0;
  const auto q1q1 = q1 * q1;
  const auto q2q2 = q2 * q2;
  const auto q3q3 = q3 * q3;

  // Gradient descent algorithm corrective step
  const auto s0 = _4q0 * q2q2 + _2q2 * aax + _4q0 * q1q1 - _2q1 * aay;
  const auto s1 = _4q1 * q3q3 - _2q3 * aax + 4 * q0q0 * q1 - _2q0 * aay - _4q1 +
      _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * aaz;
  const auto s2 = 4 * q0q0 * q2 + _2q0
      * aax + _4q2 * q3q3 - _2q3 * aay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
      _4q2 * aaz;
  const auto s3 = 4 * q1q1 * q3 - _2q1 * aax + 4 * q2q2 * q3 - _2q2 * aay;

  const auto recipNorm1 = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

  const auto isAccelOkay = not (ax == 0 && ay == 0 && az == 0);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  const auto qqDot1 = qDot1 - (isAccelOkay ? b_madgwick * s0 * recipNorm1 : 0);
  const auto qqDot2 = qDot2 - (isAccelOkay ? b_madgwick * s1 * recipNorm1 : 0);
  const auto qqDot3 = qDot3 - (isAccelOkay ? b_madgwick * s2 * recipNorm1 : 0);
  const auto qqDot4 = qDot4 - (isAccelOkay ? b_madgwick * s3 * recipNorm1 : 0);

  q0 = q0 + qqDot1 * invSampFreq;
  q1 = q1 + qqDot2 * invSampFreq;
  q2 = q2 + qqDot3 * invSampFreq;
  q3 = q3 + qqDot4 * invSampFreq;
}

static const float mymax(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static const float square(const float x)
{
    return x * x;
}

static void updateCovarianceMatrix(const matrix_t & p_in, matrix_t & p_out) 
{
    // Enforce symmetry of the covariance matrix, and ensure the
    // values stay bounded
    for (int i=0; i<KC_STATE_DIM; i++) {

        for (int j=i; j<KC_STATE_DIM; j++) {

            const auto pval = (p_in.dat[i][j] + p_in.dat[j][i]) / 2;

            p_out.dat[i][j] = p_out.dat[j][i] = 
                pval > MAX_COVARIANCE ?  MAX_COVARIANCE :
                (i==j && pval < MIN_COVARIANCE) ?  MIN_COVARIANCE :
                pval;
        }
    }
}

static void scalarUpdate(
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const float h[KC_STATE_DIM],
        const float error, 
        const float stdMeasNoise,
        matrix_t & p_out,
        ekfState_t & ekfs_out)
{

    // ====== INNOVATION COVARIANCE ======
    float ph[KC_STATE_DIM] = {};
    multiply(p_in.dat, h, ph);
    const auto r = stdMeasNoise * stdMeasNoise;
    const auto hphr = r + dot(h, ph); // HPH' + R

    // Compute the Kalman gain as a column vector
    const float g[KC_STATE_DIM] = {

        // kalman gain = (PH' (HPH' + R )^-1)
        ph[0] / hphr, 
        ph[1] / hphr, 
        ph[2] / hphr, 
        ph[3] / hphr, 
    };

    // Perform the state update
    ekfs_out.z  = ekfs_in.z  + g[0] * error;
    ekfs_out.dx = ekfs_in.dx + g[1] * error;
    ekfs_out.dy = ekfs_in.dy + g[2] * error;
    ekfs_out.dz = ekfs_in.dz + g[3] * error;

    // ====== COVARIANCE UPDATE ======

    matrix_t GH = {};
    multiply(g, h, GH.dat); // KH

    for (int i=0; i<KC_STATE_DIM; i++) { 
        GH.dat[i][i] -= 1;
    } // KH - I

    matrix_t GHt = {};
    transpose(GH.dat, GHt.dat);      // (KH - I)'
    matrix_t GHIP = {};
    multiply(GH.dat, p_in.dat, GHIP.dat);  // (KH - I)*P
    multiply(GHIP.dat, GHt.dat, p_out.dat); // (KH - I)*P*(KH - I)'

    // Add the measurement variance 
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=0; j<KC_STATE_DIM; j++) {
            p_out.dat[i][j] += j < i ? 0 : r * g[i] * g[j];
        }
    }

    updateCovarianceMatrix(p_out, p_out);
}

static bool isPositionWithinBounds(const float pos)
{
    return fabs(pos) < MAX_POSITION;
}

static bool isVelocityWithinBounds(const float vel)
{
    return fabs(vel) < MAX_VELOCITY;
}

static void subSamplerTakeMean(
        const float sumx,
        const float sumy,
        const float sumz,
        const uint32_t count,
        const float conversionFactor,
        float & avgx,
        float & avgy,
        float & avgz)
{
    const auto isCountNonzero = count > 0;

    avgx = isCountNonzero ? sumx * conversionFactor / count : avgx;
    avgy = isCountNonzero ? sumy * conversionFactor / count : avgy;
    avgz = isCountNonzero ? sumz * conversionFactor / count : avgz;
}

// ===========================================================================

static void ekf_init(matrix_t & p_out)
{
    memset(&p_out, 0, sizeof(p_out));
    p_out.dat[KC_STATE_Z][KC_STATE_Z] = square(STDEV_INITIAL_POSITION_Z);
    p_out.dat[KC_STATE_DX][KC_STATE_DX] = square(STDEV_INITIAL_VELOCITY);
    p_out.dat[KC_STATE_DY][KC_STATE_DY] = square(STDEV_INITIAL_VELOCITY);
    p_out.dat[KC_STATE_DZ][KC_STATE_DZ] = square(STDEV_INITIAL_VELOCITY);
}

static void ekf_predict(
        const float gyroSum_x,
        const float gyroSum_y,
        const float gyroSum_z,
        const uint32_t gyroCount,
        const float accelSum_x,
        const float accelSum_y,
        const float accelSum_z,
        const uint32_t accelCount,
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const axis3_t & r,
        const uint32_t lastPredictionMsec, 
        new_quat_t & quat_out,
        matrix_t & p_out,
        ekfState_t & ekfs_out)
{
    static float _gyro_x;
    static float _gyro_y;
    static float _gyro_z;

    static float _accel_x;
    static float _accel_y;
    static float _accel_z;

    const float dt = (stream_nowMsec - lastPredictionMsec) / 1000.0f;
    const auto dt2 = dt * dt;

    subSamplerTakeMean(gyroSum_x, gyroSum_y, gyroSum_z, gyroCount,
            DEGREES_TO_RADIANS, _gyro_x, _gyro_y, _gyro_z);

    subSamplerTakeMean(accelSum_x, accelSum_y, accelSum_z, accelCount, 
            MSS_TO_GS, _accel_x, _accel_y, _accel_z);

    madgwick(_gyro_x, _gyro_y, _gyro_z, _accel_x, _accel_y, _accel_z, dt,
            quat_out.w, quat_out.x, quat_out.y, quat_out.z);

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = ekfs_in.dx * dt + stream_isFlying ? 0 : _accel_x * dt2 / 2;
    const auto dy = ekfs_in.dy * dt + stream_isFlying ? 0 : _accel_y * dt2 / 2;
    const auto dz = ekfs_in.dz * dt + _accel_z * dt2 / 2; 

    // keep previous time step's state for the update
    const auto tmpSDX = ekfs_in.dx;
    const auto tmpSDY = ekfs_in.dy;
    const auto tmpSDZ = ekfs_in.dz;

    const auto accx = stream_isFlying ? 0 : _accel_x;
    const auto accy = stream_isFlying ? 0 : _accel_y;

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless
    // to estimate body angle while flying)

    // altitude update
    ekfs_out.z = ekfs_in.z + r.x * dx + r.y * dy + r.z * dz - MSS_TO_GS * dt2 / 2;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    ekfs_out.dx = ekfs_in.dx +
        dt * (accx + _gyro_z * tmpSDY - _gyro_y * tmpSDZ - MSS_TO_GS * r.x);

    ekfs_out.dy = ekfs_in.dy +
        dt * (accy - _gyro_z * tmpSDX + _gyro_x * tmpSDZ - MSS_TO_GS * r.y); 

    ekfs_out.dz =  ekfs_in.dz +
        dt * (_accel_z + _gyro_y * tmpSDX - _gyro_x * tmpSDY - MSS_TO_GS * r.z);


    // ====== COVARIANCE UPDATE ======

    // altitude from body-frame velocity
    const auto zdx  = r.x*dt;
    const auto zdy  = r.y*dt;
    const auto zdz  = r.z*dt;

    // body-frame velocity from body-frame velocity
    const auto dxdx  = 1; //drag negligible
    const auto dydx =  -_gyro_z*dt;
    const auto dzdx  = _gyro_y*dt;

    const auto dxdy  = _gyro_z*dt;
    const auto dydy  = 1; //drag negligible
    const auto dzdy  = _gyro_x*dt;

    const auto dxdz =  _gyro_y*dt;
    const auto dydz  = _gyro_x*dt;
    const auto dzdz  = 1; //drag negligible

    const float A[KC_STATE_DIM][KC_STATE_DIM] = 
    { 
        //        Z  DX    DY    DZ
        /*Z*/    {0, zdx,  zdy,  zdz}, 
        /*DX*/   {0, dxdx, dxdy, dxdz}, 
        /*DY*/   {0, dydx, dydy, dydz},
        /*DZ*/   {0, dzdx, dzdy, dzdz},
    };


    matrix_t  At = {};
    transpose(A, At.dat);     // A'
    matrix_t AP = {};
    multiply(A, p_in.dat, AP.dat);  // AP
    matrix_t APA = {};
    multiply(AP.dat, At.dat, APA.dat); // APA'
    updateCovarianceMatrix(APA, p_out);
}

static bool ekf_updateWithRange(
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const float rz,
        matrix_t & p_out,
        ekfState_t & ekfs_out)
{
    const auto angle = mymax(0, 
            fabsf(acosf(rz)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = ekfs_in.z / cosf(angle);
    const auto measuredDistance = stream_range.distance; // [m]

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

    const float h[KC_STATE_DIM] = {1 / cosf(angle), 0, 0, 0};

    // Only update the filter if the measurement is reliable 
    // (\hat{h} -> infty when R[2][2] -> 0)
    const auto shouldUpdate = fabs(rz) > 0.1f && rz > 0;
    if (shouldUpdate) {
        scalarUpdate(
                p_in,
                ekfs_in,
                h , 
                measuredDistance-predictedDistance, 
                stream_range.stdDev, 
                p_out,
                ekfs_out);
    }
    return shouldUpdate;
}

static bool ekf_updateWithFlow(
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const float rz,
        const axis3_t & gyroLatest,
        matrix_t & p_out,
        ekfState_t & ekfs_out) 
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

    const auto dx_g = ekfs_in.dx;
    const auto dy_g = ekfs_in.dy;

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = ekfs_in.z < 0.1f ? 0.1f : ekfs_in.z;

    // ~~~ X velocity prediction and update ~~~
    // predicts the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = {};
    auto predictedNX = (stream_flow.dt * Npix / thetapix ) * 
        ((dx_g * rz / z_g) - omegay_b);
    auto measuredNX = stream_flow.dpixelx*FLOW_RESOLUTION;

    // derive measurement equation with respect to dx (and z?)
    hx[KC_STATE_Z] = (Npix * stream_flow.dt / thetapix) * 
        ((rz * dx_g) / (-z_g * z_g));
    hx[KC_STATE_DX] = (Npix * stream_flow.dt / thetapix) * 
        (rz / z_g);

    matrix_t p_first = {};
    ekfState_t ekfs_first = {};

    //First update
    scalarUpdate(
            p_in,
            ekfs_in,
            hx, 
            measuredNX-predictedNX, 
            stream_flow.stdDevX*FLOW_RESOLUTION, 
            p_first,
            ekfs_first);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = {};
    auto predictedNY = (stream_flow.dt * Npix / thetapix ) * 
        ((dy_g * rz / z_g) + omegax_b);
    auto measuredNY = stream_flow.dpixely*FLOW_RESOLUTION;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_Z] = (Npix * stream_flow.dt / thetapix) * 
        ((rz * dy_g) / (-z_g * z_g));
    hy[KC_STATE_DY] = (Npix * stream_flow.dt / thetapix) * (rz / z_g);

    // Second update
    scalarUpdate(
            p_first,
            ekfs_first,
            hy, 
            measuredNY-predictedNY, 
            stream_flow.stdDevY*FLOW_RESOLUTION, 
            p_out,
            ekfs_out);

    return true;
}

static void ekf_finalize(
        const matrix_t & p_in,
        const ekfState_t & ekfs,
        const new_quat_t & q,
        matrix_t & p_out,
        new_quat_t & q_out)
{
}
 
static void ekf_getVehicleState(
        const ekfState_t & ekfs, 
        const axis3_t & gyroLatest,
        const axis3_t & r,
        const float qw, 
        const float qx, 
        const float qy, 
        const float qz, 
        vehicleState_t & state)
{
    state.dx = ekfs.dx;
    state.dy = ekfs.dy;
    state.z = ekfs.z;
    state.dz = r.x * ekfs.dx + r.y * ekfs.dy + r.z * ekfs.dz;

    state.phi = RADIANS_TO_DEGREES * atan2((2 * (qy*qz + qw*qx)),
            (qw*qw - qx*qx - qy*qy + qz*qz));

    // Negate for ENU
    state.theta = -RADIANS_TO_DEGREES * asin((-2) * (qx*qz - qw*qy));

    state.psi = RADIANS_TO_DEGREES * atan2((2 * (qx*qy + qw*qz)),
            (qw*qw + qx*qx - qy*qy - qz*qz));

    // Get angular velocities directly from gyro
    state.dphi =    gyroLatest.x;
    state.dtheta = -gyroLatest.y; // negate for ENU
    state.dpsi =    gyroLatest.z;
}

// ===========================================================================

static void ekf_step(void)
{
    static float _qw;
    static float _qx;
    static float _qy;
    static float _qz;

    static float _z;
    static float _dx;
    static float _dy;
    static float _dz;

    static matrix_t _p;

    static float _gyroSum_x;
    static float _gyroSum_y;
    static float _gyroSum_z;
    static uint32_t _gyroCount;

    static float _accelSum_x;
    static float _accelSum_y;
    static float _accelSum_z;
    static uint32_t _accelCount;

    static axis3_t _gyroLatest;

    static bool _isUpdated;
    static uint32_t _lastPredictionMsec;
    static uint32_t _lastProcessNoiseUpdateMsec;

    static float _rx;
    static float _ry;
    static float _rz;

    const auto ekfs = ekfState_t {_z, _dx, _dy, _dz};

    const auto quat = new_quat_t {_qw, _qx, _qy, _qz };
    const auto r = axis3_t {_rx, _ry, _rz};

    // Initialize
    bool didInitialize = stream_ekfAction == EKF_INIT;
    matrix_t p_initialized = {};
    ekf_init(p_initialized);

    // Predict
    const auto didPredict = stream_ekfAction == EKF_PREDICT && 
        stream_nowMsec >= stream_nextPredictionMsec;
    ekfState_t ekfs_predicted = {};
    new_quat_t quat_predicted = {};
    if (didPredict) {
        ekf_predict(
                _gyroSum_x,
                _gyroSum_y,
                _gyroSum_z,
                _gyroCount,
                _accelSum_x,
                _accelSum_y,
                _accelSum_z,
                _accelCount,
                _p,
                ekfs,
                r,
                _lastPredictionMsec, 

                quat_predicted,
                _p,
                ekfs_predicted);
    }

    const auto isDtPositive = didPredict && 
        (stream_nowMsec - _lastProcessNoiseUpdateMsec) / 1000.0f;

    // Finalize
    const auto finalizing = stream_ekfAction == EKF_FINALIZE;
    const auto didFinalize = finalizing && _isUpdated;
    new_quat_t quat_finalized = {};
    if (didFinalize) {
        ekf_finalize(_p, ekfs, quat, _p, quat_finalized);
    }

    // Update with flow
    ekfState_t ekfs_updatedWithFlow = {};
    const auto didUpdateWithFlow = stream_ekfAction == EKF_UPDATE_WITH_FLOW &&
        ekf_updateWithFlow(_p, ekfs, _rz, _gyroLatest, 
                _p, ekfs_updatedWithFlow);

    // Update with range
    ekfState_t ekfs_updatedWithRange = {};
    const auto didUpdateWithRange = stream_ekfAction == EKF_UPDATE_WITH_RANGE &&
        ekf_updateWithRange(_p, ekfs, _rz, 
                _p, ekfs_updatedWithRange);

    // Update with gyro
    const auto didUpdateWithGyro = stream_ekfAction == EKF_UPDATE_WITH_GYRO;

    // Update with accel
    const auto didUpdateWithAccel = stream_ekfAction == EKF_UPDATE_WITH_ACCEL;

    // Get vehicle state
    vehicleState_t vehicleState = {};
    ekf_getVehicleState(ekfs, _gyroLatest, r, _qw, _qx, _qy, _qz, vehicleState);

    if (stream_ekfAction == EKF_GET_STATE) {
        setState(vehicleState);
    }

    if (finalizing) {
        setStateIsInBounds(
                isPositionWithinBounds(ekfs.z) &&
                isVelocityWithinBounds(ekfs.dx) &&
                isVelocityWithinBounds(ekfs.dy) &&
                isVelocityWithinBounds(ekfs.dz));
    }

    //////////////////////////////////////////////////////////////////////////

    _gyroSum_x = didUpdateWithGyro ? _gyroSum_x + stream_gyro.x :
        isDtPositive ? 0 :
        _gyroSum_x;

    _gyroSum_y = didUpdateWithGyro ? _gyroSum_y + stream_gyro.y :
        isDtPositive ? 0 :
        _gyroSum_y;

    _gyroSum_z = didUpdateWithGyro ? _gyroSum_z + stream_gyro.z :
        isDtPositive ? 0 :
        _gyroSum_z;

    _accelSum_x = didUpdateWithAccel ? _accelSum_x + stream_accel.x :
        isDtPositive ? 0 :
        _accelSum_x;

    _accelSum_y = didUpdateWithAccel ? _accelSum_y + stream_accel.y :
        isDtPositive ? 0 :
        _accelSum_y;

    _accelSum_z = didUpdateWithAccel ? _accelSum_z + stream_accel.z :
        isDtPositive ? 0 :
        _accelSum_z;

    memcpy(&_p, 
            didInitialize ? &p_initialized :
            &_p, sizeof(_p));

    memcpy(&_gyroLatest, 
            didUpdateWithGyro ?  &stream_gyro : &_gyroLatest, 
            sizeof(axis3_t));

    _gyroCount = isDtPositive ? 0 : 
        didUpdateWithGyro ? _gyroCount + 1 :
        _gyroCount;

    _accelCount = isDtPositive ? 0 : 
        didUpdateWithAccel ? _accelCount + 1 :
        _accelCount;

    _qw = didInitialize ? 1 : 
        didFinalize ? quat_finalized.w :
        isDtPositive ? quat_predicted.w :
        _qw;

    _qx = didInitialize ? 0 : 
        didFinalize ? quat_finalized.x :
        isDtPositive ? quat_predicted.x :
        _qx;

    _qy = didInitialize ? 0 : 
        didFinalize ? quat_finalized.y :
        isDtPositive ? quat_predicted.y :
        _qy;

    _qz = didInitialize ? 0 : 
        didFinalize ? quat_finalized.z :
        isDtPositive ? quat_predicted.z :
        _qz;


    _rx = didInitialize ? 0 : 
        didFinalize ? 2 * _qx * _qz - 2 * _qw * _qy :
        _rx;

    _ry = didInitialize ? 0 : 
        didFinalize ? 2 * _qy * _qz + 2 * _qw * _qx :
        _ry;

    _rz = didInitialize ? 1 : 
        didFinalize ? _qw*_qw-_qx*_qx-_qy*_qy+_qz*_qz:
        _rz;

    _z = didInitialize ? 0 : 
        isDtPositive ? ekfs_predicted.z :
        didUpdateWithFlow ? ekfs_updatedWithFlow.z :
        didUpdateWithRange ? ekfs_updatedWithRange.z :
        _z;

    _dx = didInitialize ? 0 : 
        isDtPositive ? ekfs_predicted.dx :
        didUpdateWithFlow ? ekfs_updatedWithFlow.dx :
        didUpdateWithRange ? ekfs_updatedWithRange.dx :
        _dx;

    _dy = didInitialize ? 0 : 
        isDtPositive ? ekfs_predicted.dy :
        didUpdateWithFlow ? ekfs_updatedWithFlow.dy :
        didUpdateWithRange ? ekfs_updatedWithRange.dy :
        _dy;

    _dz = didInitialize ? 0 : 
        isDtPositive ? ekfs_predicted.dz :
        didUpdateWithFlow ? ekfs_updatedWithFlow.dz :
        didUpdateWithRange ? ekfs_updatedWithRange.dz :
        _dz;

    _lastProcessNoiseUpdateMsec = 
        didInitialize || isDtPositive ?  
        stream_nowMsec : 
        _lastProcessNoiseUpdateMsec;

    _lastPredictionMsec = 
        didInitialize || didPredict ? stream_nowMsec :
        _lastPredictionMsec;

    _isUpdated = 
        didInitialize || didFinalize ? false :
        stream_ekfAction == EKF_PREDICT ? true :
        didUpdateWithFlow || didUpdateWithRange ? true :
        _isUpdated;
}
