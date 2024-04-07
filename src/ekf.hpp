#pragma once

#include <string.h>

#include <math3d.h>
#include <datatypes.h>
#include <linalg.h>
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

    Axis3f subSample;

} axisSubSampler_t;

typedef struct {

    axisSubSampler_t accelSubSampler;
    axisSubSampler_t gyroSubSampler;

    float z;
    float dx;
    float dy;
    float dz;
    float e0;
    float e1;
    float e2;

    float p[KC_STATE_DIM][KC_STATE_DIM];

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

static float rotateQuat( const float val, const float initVal)
{
    const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

    return (val * (stream_isFlying ? 1: keep)) +
        (stream_isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
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

static bool scalarUpdate(
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
    ekf.z  += shouldUpdate ? g[0] * error: 0;
    ekf.dx += shouldUpdate ? g[1] * error: 0;
    ekf.dy += shouldUpdate ? g[2] * error: 0;
    ekf.dz += shouldUpdate ? g[3] * error: 0;
    ekf.e0 += shouldUpdate ? g[4] * error: 0;
    ekf.e1 += shouldUpdate ? g[5] * error: 0;
    ekf.e2 += shouldUpdate ? g[6] * error: 0;

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

    return shouldUpdate;
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


static bool isStateWithinBounds(
        const float z, const float dx, const float dy, const float dz)
{
    return 
        isPositionWithinBounds(z) &&
        isVelocityWithinBounds(dx) &&
        isVelocityWithinBounds(dy) &&
        isVelocityWithinBounds(dz);
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


// ===========================================================================

static void ekf_init(ekf_t & ekf)
{
    ekf.z = 0;
    ekf.dx = 0;
    ekf.dy = 0;
    ekf.dz = 0;
    ekf.e0 = 0;
    ekf.e1 = 0;
    ekf.e2 = 0;

    memset(&ekf.p, 0, sizeof(ekf.p));
    ekf.p[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);
    ekf.p[KC_STATE_DX][KC_STATE_DX] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_DY][KC_STATE_DY] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_DZ][KC_STATE_DZ] = powf(STDEV_INITIAL_VELOCITY, 2);
    ekf.p[KC_STATE_E1][KC_STATE_E1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
    ekf.p[KC_STATE_E2][KC_STATE_E2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);
}

static bool ekf_predict(
        const float qw,
        const float qx,
        const float qy,
        const float qz,
        const float rx,
        const float ry,
        const float rz,
        const uint32_t lastProcessNoiseUpdateMsec, 
        const uint32_t lastPredictionMsec, 
        float & qw_out,
        float & qx_out,
        float & qy_out,
        float & qz_out,
        ekf_t & ekf) 
{
    subSamplerFinalize(&ekf.gyroSubSampler, DEGREES_TO_RADIANS);

    const float dt = (stream_nowMsec - lastPredictionMsec) / 1000.0f;

    const auto dt2 = dt * dt;

    subSamplerFinalize(&ekf.accelSubSampler, MSS_TO_GS);

    const Axis3f * acc = &ekf.accelSubSampler.subSample; 

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = ekf.dx * dt + stream_isFlying ? 0 : acc->x * dt2 / 2;
    const auto dy = ekf.dy * dt + stream_isFlying ? 0 : acc->y * dt2 / 2;
    const auto dz = ekf.dz * dt + acc->z * dt2 / 2; 

    // keep previous time step's state for the update
    const auto tmpSDX = ekf.dx;
    const auto tmpSDY = ekf.dy;
    const auto tmpSDZ = ekf.dz;

    const auto accx = stream_isFlying ? 0 : acc->x;
    const auto accy = stream_isFlying ? 0 : acc->y;

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

    const auto tmpq0 = rotateQuat(dqw*qw - dqx*qx - dqy*qy - dqz*qz, QW_INIT);
    const auto tmpq1 = rotateQuat(dqx*qw + dqw*qx + dqz*qy - dqy*qz, QX_INIT);
    const auto tmpq2 = rotateQuat(dqy*qw - dqz*qx + dqw*qy + dqx*qz, QY_INIT);
    const auto tmpq3 = rotateQuat(dqz*qw + dqy*qx - dqx*qy + dqw*qz, QZ_INIT);

    // normalize and store the result
    const auto norm = 
        sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
        EPS;

    // Process noise is added after the return from the prediction step

    // ====== PREDICTION STEP ======
    // The prediction depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless
    // to estimate body angle while flying)

    // altitude update
    ekf.z += rx * dx + ry * dy + rz * dz - MSS_TO_GS * dt2 / 2;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    ekf.dx += 
        dt * (accx + gyro->z * tmpSDY - gyro->y * tmpSDZ - MSS_TO_GS * rx);

    ekf.dy += 
        dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - MSS_TO_GS * ry); 

    ekf.dz += 
        dt * (acc->z + gyro->y * tmpSDX - gyro->x * tmpSDY - MSS_TO_GS * rz);

    // predict()
    qw_out = tmpq0/norm;
    qx_out = tmpq1/norm; 
    qy_out = tmpq2/norm; 
    qz_out = tmpq3/norm;

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
    const auto zdx  = rx*dt;
    const auto zdy  = ry*dt;
    const auto zdz  = rz*dt;

    // altitude from attitude error
    const auto ze0  = (ekf.dy*rz - ekf.dz*ry)*dt;
    const auto ze1  = (- ekf.dx*rz + ekf.dz*rx)*dt;
    const auto ze2  = (ekf.dx*ry - ekf.dy*rx)*dt;

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
    const auto dye0  = -MSS_TO_GS*rz*dt;
    const auto dze0  = MSS_TO_GS*ry*dt;

    const auto dxe1  = MSS_TO_GS*rz*dt;
    const auto dye1  = 0;
    const auto dze1  = -MSS_TO_GS*rx*dt;

    const auto dxe2  = -MSS_TO_GS*ry*dt;
    const auto dye2  = MSS_TO_GS*rx*dt;
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

    const auto dt1 = (stream_nowMsec - lastProcessNoiseUpdateMsec) / 1000.0f;
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

    return isDtPositive;
}

static bool ekf_updateWithRange(
        ekf_t & ekf, const float rx, const float ry, const float rz)
{
    const auto angle = max( 0, 
            fabsf(acosf(rz)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = ekf.z / cosf(angle);
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

    const float h[KC_STATE_DIM] = {1 / cosf(angle), 0, 0, 0, 0, 0, 0};

    // Only update the filter if the measurement is reliable 
    // (\hat{h} -> infty when R[2][2] -> 0)
    const auto shouldUpdate = fabs(rz) > 0.1f && rz > 0;
    return scalarUpdate(ekf, h, measuredDistance-predictedDistance, 
            stream_range.stdDev, shouldUpdate);
}

static bool ekf_updateWithFlow(
        const float rx, const float ry, const float rz, 
        const Axis3f & gyroLatest,
        ekf_t & ekf) 
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

    const auto dx_g = ekf.dx;
    const auto dy_g = ekf.dy;

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = ekf.z < 0.1f ? 0.1f : ekf.z;

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

    //First update
    scalarUpdate(ekf, hx, (measuredNX-predictedNX), 
            stream_flow.stdDevX*FLOW_RESOLUTION, true);

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
    return scalarUpdate(ekf, hy, (measuredNY-predictedNY), 
            stream_flow.stdDevY*FLOW_RESOLUTION, true);
}

static bool ekf_finalize(
        const float qw, const float qx, const float qy, const float qz,
        ekf_t & ekf,
        float & qw_out, float & qx_out, float & qy_out, float & qz_out)
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = ekf.e0;
    const auto v1 = ekf.e1;
    const auto v2 = ekf.e2;

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

    const auto isErrorSufficient  = 
        (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
        isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

    // finalize()
    qw_out = isErrorSufficient ? tmpq0 / norm : qw;
    qx_out = isErrorSufficient ? tmpq1 / norm : qx;
    qy_out = isErrorSufficient ? tmpq2 / norm : qy;
    qz_out = isErrorSufficient ? tmpq3 / norm : qz;

    // Move attitude error into attitude if any of the angle errors are
    // large enough
    float A[KC_STATE_DIM][KC_STATE_DIM] = {};
    afinalize(v0, v2, v2, A);
    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'
    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, ekf.p, AP, true);  // AP
    multiply(AP, At, ekf.p, isErrorSufficient); // APA'

    // Reset the attitude error
    // XXX finalize()
    ekf.e0 = 0;
    ekf.e1 = 0;
    ekf.e2 = 0;

    updateCovarianceMatrix(ekf.p, true);

    return isStateWithinBounds(ekf.z, ekf.dx, ekf.dy, ekf.dz);
} 

static void ekf_getState(
        const ekf_t & ekf, 
        const Axis3f & gyroLatest,
        const float qw,
        const float qx,
        const float qy,
        const float qz,
        const float rx,
        const float ry,
        const float rz,
        vehicleState_t & state)
{
    state.dx = ekf.dx;

    state.dy = ekf.dy;

    state.z = ekf.z;

    state.dz = rx * ekf.dx + ry * ekf.dy + rz * ekf.dz;

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
    static ekf_t _ekf;

    static bool _isUpdated;
    static uint32_t _lastPredictionMsec;
    static uint32_t _lastProcessNoiseUpdateMsec;

    static float _rx;
    static float _ry;
    static float _rz;

    static float _qw;
    static float _qx;
    static float _qy;
    static float _qz;

    static Axis3f _gyroLatest;

    bool didUpdateFlow = false;
    bool didUpdateRange = false;

    vehicleState_t vehicleState = {};

    const auto shouldPredict = stream_ekfAction == EKF_PREDICT && 
        stream_nowMsec >= stream_nextPredictionMsec;

    float qwp=0, qxp=0, qyp=0, qzp=0;
    const auto didPredict = 
        shouldPredict && 
        ekf_predict(
                _qw, _qx, _qy, _qz,
                _rx, _ry, _rz, 
                _lastPredictionMsec, _lastProcessNoiseUpdateMsec,
                qwp, qxp, qyp, qzp,
                _ekf);

    float qwf=0, qxf=0, qyf=0, qzf=0;
    const auto isStateInBounds = 
        _isUpdated && stream_ekfAction == EKF_FINALIZE ? 
        ekf_finalize(_qw, _qx, _qy, _qz, _ekf, qwf, qxf, qyf, qzf) :
        isStateWithinBounds(_ekf.z, _ekf.dx, _ekf.dy, _ekf.dz);

    switch (stream_ekfAction) {

        case EKF_INIT:
            ekf_init(_ekf);
            break;

        case EKF_GET_STATE:
            ekf_getState(_ekf, _gyroLatest,
                    _qw, _qx, _qy, _qz, _rx, _ry, _rz, vehicleState);
            setState(vehicleState);
            break;

        case EKF_UPDATE_WITH_GYRO:
            subSamplerAccumulate(&_ekf.gyroSubSampler, &stream_gyro);
            memcpy(&_gyroLatest, &stream_gyro, sizeof(Axis3f));
            break;

        case EKF_UPDATE_WITH_ACCEL:
            subSamplerAccumulate(&_ekf.accelSubSampler, &stream_accel);
            break;

        case EKF_UPDATE_WITH_FLOW:
            didUpdateFlow = ekf_updateWithFlow(_rx, _ry, _rz, _gyroLatest, _ekf);
            break;

        case EKF_UPDATE_WITH_RANGE:
            didUpdateRange = ekf_updateWithRange(_ekf, _rx, _ry, _rz);
            break;

        default:
            break;
    }

    const auto initializing = stream_ekfAction == EKF_INIT;
    const auto finalizing = stream_ekfAction == EKF_FINALIZE;

    _qw = initializing ? 1 : 
        finalizing ? qwf :
        didPredict ? qwp :
        _qw;

    _qx = initializing ? 0 : 
        finalizing ? qxf :
        didPredict ? qxp :
        _qx;

    _qy = initializing ? 0 : 
        finalizing ? qyf :
        didPredict ? qyp :
        _qy;

    _qz = initializing ? 0 : 
        finalizing ? qzf :
        didPredict ? qzp :
        _qz;

    _rx = initializing ? 0 : 
        finalizing ? 2 * _qx * _qz - 2 * _qw * _qy :
        _rx;

    _ry = initializing ? 0 : 
        finalizing ? 2 * _qy * _qz + 2 * _qw * _qx :
        _ry;

    _rz = initializing ? 1 : 
        finalizing ? _qw*_qw-_qx*_qx-_qy*_qy+_qz*_qz:
        _rz;

    if (finalizing) {
        setStateIsInBounds(isStateInBounds);
    }

    _lastProcessNoiseUpdateMsec = 
        initializing || didPredict ?  
        stream_nowMsec : 
        _lastProcessNoiseUpdateMsec;

    _lastPredictionMsec = 
        initializing || shouldPredict ? stream_nowMsec :
        _lastPredictionMsec;

    _isUpdated = 
        initializing || finalizing ? false :
        stream_ekfAction == EKF_PREDICT ? true :
        didUpdateFlow || didUpdateRange ? true :
        _isUpdated;
}
