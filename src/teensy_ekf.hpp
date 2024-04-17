#pragma once

#include <string.h>

#include "math3d.h"
#include "datatypes.h"
#include "teensy_linalg.h"

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

// This is slower than the IMU update rate of 1000Hz
static const uint32_t PREDICTION_RATE = 100;
static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICTION_RATE;

class Ekf { 

    public:

        void init(const uint32_t stream_now_msec)
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
            _lastPredictionMs = stream_now_msec;
            _lastProcessNoiseUpdateMs = stream_now_msec;
        }

        bool step(void) 
        {
            extern uint32_t stream_now_msec;

            const auto isFlying = true; // XXX

            static float _nextPredictionMsec;

            const auto shouldPredict = stream_now_msec >= _nextPredictionMsec;

            _nextPredictionMsec = 
                _nextPredictionMsec == 0 ? stream_now_msec : _nextPredictionMsec;

            axis3fSubSamplerFinalize(&_accSubSampler, shouldPredict);

            axis3fSubSamplerFinalize(&_gyroSubSampler, shouldPredict);

            const Axis3f * acc = &_accSubSampler.subSample; 
            const Axis3f * gyro = &_gyroSubSampler.subSample; 
            const float dt = (stream_now_msec - _lastPredictionMs) / 1000.0f;

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
            multiply(A, _Pmat, AP);  // AP
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

            _lastPredictionMs = shouldPredict ? stream_now_msec : _lastPredictionMs;

            const auto dt1 = (stream_now_msec - _lastProcessNoiseUpdateMs) / 1000.0f;

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

            _lastProcessNoiseUpdateMs = isDtPositive ?  
                stream_now_msec : 
                _lastProcessNoiseUpdateMs;

            _nextPredictionMsec = stream_now_msec >= _nextPredictionMsec ?
                stream_now_msec + PREDICTION_UPDATE_INTERVAL_MS :
                _nextPredictionMsec;

            extern float stream_accel_x, stream_accel_y, stream_accel_z;

            const auto accel = 
                Axis3f {stream_accel_x, stream_accel_y, stream_accel_z};

            axis3fSubSamplerAccumulate(&_accSubSampler, &accel);
        
            extern float stream_gyro_x, stream_gyro_y, stream_gyro_z;

            const auto raw_gyro = 
                Axis3f {stream_gyro_x, stream_gyro_y, stream_gyro_z};

            axis3fSubSamplerAccumulate(&_gyroSubSampler, &raw_gyro);

            memcpy(&_gyroLatest, &raw_gyro, sizeof(Axis3f));
        
            // Only finalize if data is updated
            return _isUpdated ? doFinalize() : isStateWithinBounds();
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

     private:


        // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
        // to allow easy normalization (in comparison to a rotation matrix),
        // while also being robust against singularities (in comparison to euler angles)
        float _qw;
        float _qx;
        float _qy;
        float _qz;

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
            multiply(A, _Pmat, AP);  // AP

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

        static float rotateQuat(
                const float val, 
                const float initVal,
                const bool isFlying)
        {
            const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

            return (val * (isFlying ? 1: keep)) +
                (isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
        }

        static void axis3fSubSamplerInit(Axis3fSubSampler_t* subSampler, const
                float conversionFactor) 
        { 
            memset(subSampler, 0, sizeof(Axis3fSubSampler_t));
            subSampler->conversionFactor = conversionFactor;
        }

        static void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* subSampler,
                const Axis3f* sample) 
        {
            subSampler->sum.x += sample->x;
            subSampler->sum.y += sample->y;
            subSampler->sum.z += sample->z;

            subSampler->count++;
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

        static bool isErrorLarge(const float v)
        {
            return fabs(v) > 0.1e-3f;
        }

        static bool isErrorInBounds(const float v)
        {
            return fabs(v) < 10;
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

        static const float max(const float val, const float maxval)
        {
            return val > maxval ? maxval : val;
        }

        bool isStateWithinBounds(void) 
        {
            return 
                isPositionWithinBounds(_ekfState.z) &&
                isVelocityWithinBounds(_ekfState.dx) &&
                isVelocityWithinBounds(_ekfState.dy) &&
                isVelocityWithinBounds(_ekfState.dz);
        }

        static bool isPositionWithinBounds(const float pos)
        {
            return fabs(pos) < MAX_POSITION;
        }

        static bool isVelocityWithinBounds(const float vel)
        {
            return fabs(vel) < MAX_VELOCITY;
        }
};
