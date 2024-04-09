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

    float dat[KC_STATE_DIM][KC_STATE_DIM];

} matrix_t;

typedef struct {

    float w;
    float x;
    float y;
    float z;

} new_quat_t;


typedef struct {

    axis3_t sum;
    axis3_t avg;
    uint32_t count;

} axisSubSampler_t;

typedef struct {

    float z;
    float dx;
    float dy;
    float dz;

} ekfLinear_t;

typedef struct {

    ekfLinear_t lin;

    axis3_t ang;

} ekfState_t;

static const float max(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static void subSamplerAccumulate(
        axisSubSampler_t & subSampler,
        const axis3_t & sample) 
{
    subSampler.sum.x += sample.x;
    subSampler.sum.y += sample.y;
    subSampler.sum.z += sample.z;

    subSampler.count++;
}

static void subSamplerTakeMean(
        const axisSubSampler_t & subSampler_in,
        const float conversionFactor,
        axisSubSampler_t & subSampler_out)
{
    const auto count  = subSampler_out.count; 
    const auto isCountNonzero = count > 0;

    subSampler_out.avg.x = isCountNonzero ? 
        subSampler_out.sum.x * conversionFactor / count :
        subSampler_out.avg.x;

    subSampler_out.avg.y = isCountNonzero ?
        subSampler_out.sum.y * conversionFactor / count :
        subSampler_out.avg.y;

    subSampler_out.avg.z = isCountNonzero ?
        subSampler_out.sum.z * conversionFactor / count :
        subSampler_out.avg.z;

    // Reset
    subSampler_out.count = 0;
    memset(&subSampler_out.sum, 0, sizeof(subSampler_out.sum));
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
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const float h[KC_STATE_DIM],
        const float error, 
        const float stdMeasNoise,
        const bool shouldUpdate,
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
        ph[4] / hphr, 
        ph[5] / hphr, 
        ph[6] / hphr
    };

    // Perform the state update
    ekfs_out.lin.z  = ekfs_in.lin.z  + (shouldUpdate ? g[0] * error: 0);
    ekfs_out.lin.dx = ekfs_in.lin.dx + (shouldUpdate ? g[1] * error: 0);
    ekfs_out.lin.dy = ekfs_in.lin.dy + (shouldUpdate ? g[2] * error: 0);
    ekfs_out.lin.dz = ekfs_in.lin.dz + (shouldUpdate ? g[3] * error: 0);
    ekfs_out.ang.x = ekfs_in.ang.x + (shouldUpdate ? g[4] * error: 0);
    ekfs_out.ang.y = ekfs_in.ang.y + (shouldUpdate ? g[5] * error: 0);
    ekfs_out.ang.z = ekfs_in.ang.z + (shouldUpdate ? g[6] * error: 0);

    // ====== COVARIANCE UPDATE ======

    float GH[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(g, h, GH); // KH

    for (int i=0; i<KC_STATE_DIM; i++) { 
        GH[i][i] -= 1;
    } // KH - I

    float GHt[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(GH, GHt);      // (KH - I)'

    float GHIP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(GH, p_in.dat, GHIP, true);  // (KH - I)*P

    multiply(GHIP, GHt, p_out.dat, shouldUpdate); // (KH - I)*P*(KH - I)'

    // Add the measurement variance and ensure boundedness and symmetry
    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {

            updateCovarianceCell(p_out.dat, i, j, g[i] * r * g[j], shouldUpdate);
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


static bool isStateWithinBounds(const ekfState_t & ekfs)
{
    return 
        isPositionWithinBounds(ekfs.lin.z) &&
        isVelocityWithinBounds(ekfs.lin.dx) &&
        isVelocityWithinBounds(ekfs.lin.dy) &&
        isVelocityWithinBounds(ekfs.lin.dz);
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

static void ekf_init(matrix_t & p_out)
{
    memset(&p_out, 0, sizeof(p_out));
    p_out.dat[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);
    p_out.dat[KC_STATE_DX][KC_STATE_DX] = powf(STDEV_INITIAL_VELOCITY, 2);
    p_out.dat[KC_STATE_DY][KC_STATE_DY] = powf(STDEV_INITIAL_VELOCITY, 2);
    p_out.dat[KC_STATE_DZ][KC_STATE_DZ] = powf(STDEV_INITIAL_VELOCITY, 2);
    p_out.dat[KC_STATE_E1][KC_STATE_E1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
    p_out.dat[KC_STATE_E2][KC_STATE_E2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);
}

static bool ekf_predict(
        const axisSubSampler_t & gyroSubSampler_in,
        const axisSubSampler_t & accelSubSampler_in,
        const matrix_t & p_in,
        const ekfLinear_t & linear_in,
        const new_quat_t & q,
        const axis3_t & r,
        const uint32_t lastProcessNoiseUpdateMsec, 
        const uint32_t lastPredictionMsec, 
        new_quat_t & quat_out,
        axisSubSampler_t & gyroSubSampler_out,
        axisSubSampler_t & accelSubSampler_out,
        matrix_t & p_out,
        ekfLinear_t & linear_out) 
{
    const float dt = (stream_nowMsec - lastPredictionMsec) / 1000.0f;
    const auto dt2 = dt * dt;

    subSamplerTakeMean(gyroSubSampler_in, DEGREES_TO_RADIANS, gyroSubSampler_out);
    subSamplerTakeMean(accelSubSampler_in, MSS_TO_GS, accelSubSampler_out);

    const axis3_t * acc = &accelSubSampler_out.avg; 

    // Position updates in the body frame (will be rotated to inertial frame);
    // thrust can only be produced in the body's Z direction
    const auto dx = linear_in.dx * dt + stream_isFlying ? 0 : acc->x * dt2 / 2;
    const auto dy = linear_in.dy * dt + stream_isFlying ? 0 : acc->y * dt2 / 2;
    const auto dz = linear_in.dz * dt + acc->z * dt2 / 2; 

    // keep previous time step's state for the update
    const auto tmpSDX = linear_in.dx;
    const auto tmpSDY = linear_in.dy;
    const auto tmpSDZ = linear_in.dz;

    const auto accx = stream_isFlying ? 0 : acc->x;
    const auto accy = stream_isFlying ? 0 : acc->y;

    const axis3_t * gyro = &gyroSubSampler_out.avg; 

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

    // altitude update
    linear_out.z = linear_in.z + r.x * dx + r.y * dy + r.z * dz - MSS_TO_GS * dt2 / 2;

    // body-velocity update: accelerometers - gyros cross velocity
    // - gravity in body frame

    linear_out.dx = linear_in.dx +
        dt * (accx + gyro->z * tmpSDY - gyro->y * tmpSDZ - MSS_TO_GS * r.x);

    linear_out.dy = linear_in.dy +
        dt * (accy - gyro->z * tmpSDX + gyro->x * tmpSDZ - MSS_TO_GS * r.y); 

    linear_out.dz =  linear_in.dz +
        dt * (acc->z + gyro->y * tmpSDX - gyro->x * tmpSDY - MSS_TO_GS * r.z);

    // predict()
    quat_out.w = tmpq0/norm;
    quat_out.x = tmpq1/norm; 
    quat_out.y = tmpq2/norm; 
    quat_out.z = tmpq3/norm;

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
    const auto zdx  = r.x*dt;
    const auto zdy  = r.y*dt;
    const auto zdz  = r.z*dt;

    // altitude from attitude error
    const auto ze0  = (linear_out.dy*r.z - linear_out.dz*r.y)*dt;
    const auto ze1  = (- linear_out.dx*r.z + linear_out.dz*r.x)*dt;
    const auto ze2  = (linear_out.dx*r.y - linear_out.dy*r.x)*dt;

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
    const auto dye0  = -MSS_TO_GS*r.z*dt;
    const auto dze0  = MSS_TO_GS*r.y*dt;

    const auto dxe1  = MSS_TO_GS*r.z*dt;
    const auto dye1  = 0;
    const auto dze1  = -MSS_TO_GS*r.x*dt;

    const auto dxe2  = -MSS_TO_GS*r.y*dt;
    const auto dye2  = MSS_TO_GS*r.x*dt;
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
    multiply(A, p_in.dat, AP, true);  // AP
    multiply(AP, At, p_out.dat); // APA'

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

    addNoiseDiagonal(p_out.dat, noise, isDtPositive);

    updateCovarianceMatrix(p_out.dat, isDtPositive);

    return isDtPositive;
}

static bool ekf_updateWithRange(
        const matrix_t & p_in,
        const ekfState_t & ekfs_in,
        const float rz,
        matrix_t & p_out,
        ekfState_t & ekfs_out)
{
    const auto angle = max(0, 
            fabsf(acosf(rz)) - 
            DEGREES_TO_RADIANS * (15.0f / 2.0f));

    const auto predictedDistance = ekfs_in.lin.z / cosf(angle);
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
    return scalarUpdate(
            p_in,
            ekfs_in,
            h , 
            measuredDistance-predictedDistance, 
            stream_range.stdDev, 
            shouldUpdate,
            p_out,
            ekfs_out);
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

    const auto dx_g = ekfs_in.lin.dx;
    const auto dy_g = ekfs_in.lin.dy;

    // Saturate elevation in prediction and correction to avoid singularities
    const auto z_g = ekfs_in.lin.z < 0.1f ? 0.1f : ekfs_in.lin.z;

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
            true,
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
    return scalarUpdate(
            p_first,
            ekfs_first,
            hy, 
            measuredNY-predictedNY, 
            stream_flow.stdDevY*FLOW_RESOLUTION, 
            true,
            p_out,
            ekfs_out);
}

static bool ekf_finalize(
        const matrix_t & p_in,
        const ekfState_t & ekfs,
        const new_quat_t & q,
        matrix_t & p_out,
        new_quat_t & q_out)
{
    // Incorporate the attitude error (Kalman filter state) with the attitude
    const auto v0 = ekfs.ang.x;
    const auto v1 = ekfs.ang.y;
    const auto v2 = ekfs.ang.z;

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
    float A[KC_STATE_DIM][KC_STATE_DIM] = {};
    afinalize(v0, v2, v2, A);
    float At[KC_STATE_DIM][KC_STATE_DIM] = {};
    transpose(A, At);     // A'
    float AP[KC_STATE_DIM][KC_STATE_DIM] = {};
    multiply(A, p_in.dat, AP, true);  // AP
    multiply(AP, At, p_out.dat, isErrorSufficient); // APA'

    updateCovarianceMatrix(p_out.dat, true);

    return isStateWithinBounds(ekfs);
} 

static void ekf_getState(
        const ekfState_t & ekfs, 
        const axis3_t & gyroLatest,
        const new_quat_t & q,
        const axis3_t & r,
        vehicleState_t & state)
{
    state.dx = ekfs.lin.dx;

    state.dy = ekfs.lin.dy;

    state.z = ekfs.lin.z;

    state.dz = r.x * ekfs.lin.dx + r.y * ekfs.lin.dy + r.z * ekfs.lin.dz;

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
    static float _z;
    static float _dx;
    static float _dy;
    static float _dz;
    static float _e0;
    static float _e1;
    static float _e2;

    static matrix_t _p;

    static float _gyroSumX;
    static float _gyroSumY;
    static float _gyroSumZ;
    static float _gyroAvgX;
    static float _gyroAvgY;
    static float _gyroAvgZ;
    static uint32_t _gyroCount;

    static float _accelSumX;
    static float _accelSumY;
    static float _accelSumZ;
    static float _accelAvgX;
    static float _accelAvgY;
    static float _accelAvgZ;
    static uint32_t _accelCount;

    static axisSubSampler_t _gyroSubSampler;

    static axisSubSampler_t _accelSubSampler;

    static axis3_t _gyroLatest;

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

    const auto gyroSubSampler = axisSubSampler_t {

        {_gyroSumX, _gyroSumY, _gyroSumZ},
        {_gyroAvgX, _gyroAvgY, _gyroAvgZ},
        _gyroCount
    };

    const auto accelSubSampler = axisSubSampler_t {

        {_accelSumX, _accelSumY, _accelSumZ},
        {_accelAvgX, _accelAvgY, _accelAvgZ},
        _accelCount
    };

    const auto ekfs = ekfState_t { {_z, _dx, _dy, _dz}, {_e0, _e1, _e2}};

    const auto quat = new_quat_t {_qw, _qx, _qy, _qz };
    const auto r = axis3_t {_rx, _ry, _rz};

    // Initialize
    bool didInitialize = stream_ekfAction == EKF_INIT;
    matrix_t p_initialized = {};
    ekf_init(p_initialized);

    // Predict
    const auto shouldPredict = stream_ekfAction == EKF_PREDICT && 
        stream_nowMsec >= stream_nextPredictionMsec;
    ekfLinear_t lin_predicted = {};
    new_quat_t quat_predicted = {};
    const auto didPredict = 
        shouldPredict && 
        ekf_predict(
                gyroSubSampler,
                accelSubSampler,
                _p,
                ekfs.lin,
                quat,
                r,
                _lastPredictionMsec, 
                _lastProcessNoiseUpdateMsec,
                quat_predicted,
                _gyroSubSampler,
                _accelSubSampler,
                _p,
                lin_predicted);

    // Finalize
    const auto finalizing = stream_ekfAction == EKF_FINALIZE;
    const auto didFinalize = finalizing && _isUpdated;
    new_quat_t quat_finalized = {};
    const auto isStateInBounds = 
        didFinalize ? 
        ekf_finalize(_p, ekfs, quat, _p, quat_finalized) :
        isStateWithinBounds(ekfs);

    // Update with flow
    ekfState_t ekfs_updatedWithFlow = {};
    const auto didUpdateWithFlow = stream_ekfAction == EKF_UPDATE_WITH_FLOW &&
        ekf_updateWithFlow(_p, ekfs, _rz, _gyroLatest, _p, ekfs_updatedWithFlow);

    // Update with range
    ekfState_t ekfs_updatedWithRange = {};
    const auto didUpdateWithRange = stream_ekfAction == EKF_UPDATE_WITH_RANGE &&
        ekf_updateWithRange(_p, ekfs, _rz, _p, ekfs_updatedWithRange);

    // Update with gyro
    const auto didUpdateWithGyro = stream_ekfAction == EKF_UPDATE_WITH_GYRO;
    if (didUpdateWithGyro) {
        subSamplerAccumulate(_gyroSubSampler, stream_gyro);
    }

    // Update with accel
    const auto didUpdateWithAccel = stream_ekfAction == EKF_UPDATE_WITH_ACCEL;
    if (didUpdateWithAccel) {
            subSamplerAccumulate(_accelSubSampler, stream_accel);
    }

    // Get vehicle state
    vehicleState_t vehicleState = {};
    ekf_getState(ekfs, _gyroLatest, quat, r, vehicleState);
    if (stream_ekfAction == EKF_GET_STATE) {
        setState(vehicleState);
    }

    if (finalizing) {
        setStateIsInBounds(isStateInBounds);
    }

    //////////////////////////////////////////////////////////////////////////

    memcpy(&_p, 
            didInitialize ? &p_initialized :
            &_p, sizeof(_p));

     memcpy(&_gyroLatest, 
             didUpdateWithGyro ?  &stream_gyro : &_gyroLatest, 
             sizeof(axis3_t));

    _qw = didInitialize ? 1 : 
        didFinalize ? quat_finalized.w :
        didPredict ? quat_predicted.w :
        _qw;

    _qx = didInitialize ? 0 : 
        didFinalize ? quat_finalized.x :
        didPredict ? quat_predicted.x :
        _qx;

    _qy = didInitialize ? 0 : 
        didFinalize ? quat_finalized.y :
        didPredict ? quat_predicted.y :
        _qy;

    _qz = didInitialize ? 0 : 
        didFinalize ? quat_finalized.z :
        didPredict ? quat_predicted.z :
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
        didPredict ? lin_predicted.z :
        didUpdateWithFlow ? ekfs_updatedWithFlow.lin.z :
        didUpdateWithRange ? ekfs_updatedWithRange.lin.z :
        _z;

    _dx = didInitialize ? 0 : 
        didPredict ? lin_predicted.dx :
        didUpdateWithFlow ? ekfs_updatedWithFlow.lin.dx :
        didUpdateWithRange ? ekfs_updatedWithRange.lin.dx :
        _dx;

    _dy = didInitialize ? 0 : 
        didPredict ? lin_predicted.dy :
        didUpdateWithFlow ? ekfs_updatedWithFlow.lin.dy :
        didUpdateWithRange ? ekfs_updatedWithRange.lin.dy :
        _dy;

    _dz = didInitialize ? 0 : 
        didPredict ? lin_predicted.dz :
        didUpdateWithFlow ? ekfs_updatedWithFlow.lin.dz :
        didUpdateWithRange ? ekfs_updatedWithRange.lin.dz :
        _dz;

    _e0 = didInitialize || didFinalize ? 0 : 
        didUpdateWithFlow ? ekfs_updatedWithFlow.ang.x :
        didUpdateWithRange ? ekfs_updatedWithRange.ang.x :
        _e0;

    _e1 = didInitialize || didFinalize ? 0 : 
        didUpdateWithFlow ? ekfs_updatedWithFlow.ang.y :
        didUpdateWithRange ? ekfs_updatedWithRange.ang.y :
        _e1;

    _e2 = didInitialize || didFinalize ? 0 : 
        didUpdateWithFlow ? ekfs_updatedWithFlow.ang.z :
        didUpdateWithRange ? ekfs_updatedWithRange.ang.z :
        _e2;

    _lastProcessNoiseUpdateMsec = 
        didInitialize || didPredict ?  
        stream_nowMsec : 
        _lastProcessNoiseUpdateMsec;

    _lastPredictionMsec = 
        didInitialize || shouldPredict ? stream_nowMsec :
        _lastPredictionMsec;

    _isUpdated = 
        didInitialize || didFinalize ? false :
        stream_ekfAction == EKF_PREDICT ? true :
        didUpdateWithFlow || didUpdateWithRange ? true :
        _isUpdated;
}
