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
static const float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
static const float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

static const float PROC_NOISE_ATT = 0;
static const float MEAS_NOISE_GYRO_ROLL_PITCH = 0.1; // radians per second
static const float MEAS_NOISE_GYRO_ROLL_YAW = 0.1;   // radians per second

static const float GRAVITY_MAGNITUDE = 9.81;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
static const float MAX_COVARIANCE = 100;
static const float MIN_COVARIANCE = 1e-6;

// Small number epsilon, to prevent dividing by zero
static const float EPS = 1e-6f;

// the reversion of pitch and roll to zero
static const float ROLLPITCH_ZERO_REVERSION = 0.001;

// This is slower than the IMU update rate of 1000Hz
static const uint32_t PREDICTION_RATE = 100;
static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICTION_RATE;

class Ekf { 

    public:

        void step(vehicleState_t & vehicleState)
        {

            static bool _didInit;

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            static bool _isUpdated;

            // Quaternion
            static float _qw;
            static float _qx;
            static float _qy;
            static float _qz;

            // Third row (Z) of attitude as a rotation matrix (used by prediction,
            // updated by finalization)
            static float _r20;
            static float _r21;
            static float _r22;

            // Attitude error
            static float _e0;
            static float _e1;
            static float _e2;

            _isUpdated = !_didInit ? false : _isUpdated;

            _qw = !_didInit ? QW_INIT : _qw;
            _qx = !_didInit ? QX_INIT : _qx;
            _qy = !_didInit ? QY_INIT : _qy;
            _qz = !_didInit ? QZ_INIT : _qz;

            // Set the initial rotation matrix to the identity. This only affects
            // the first prediction step, since in the finalization, after shifting
            // attitude errors into the attitude state, the rotation matrix is updated.
            _r20 = !_didInit ? 0 : _r20;
            _r21 = !_didInit ? 0 : _r21;
            _r22 = !_didInit ? 1 : _r22;

            _e0 = !_didInit ? 0 : _e0;
            _e1 = !_didInit ? 0 : _e1;
            _e2 = !_didInit ? 0 : _e2;

            if (!_didInit) {
                step_init();
            }
            else {

                step_normal(
                        _isUpdated, 
                        _e0, _e1, _e2,
                        _qw, _qx, _qy, _qz, 
                        _r20, _r21, _r22);

                vehicleState.phi = RADIANS_TO_DEGREES * atan2((2 * (_qy*_qz + _qw*_qx)),
                        (_qw*_qw - _qx*_qx - _qy*_qy + _qz*_qz));

                // Negate for ENU
                vehicleState.theta = -RADIANS_TO_DEGREES * asin((-2) * 
                        (_qx*_qz - _qw*_qy));

                vehicleState.psi = RADIANS_TO_DEGREES * 
                    atan2((2 * (_qx*_qy + _qw*_qz)),
                        (_qw*_qw + _qx*_qx - _qy*_qy - _qz*_qz));

                // Get angular velocities directly from gyro
                vehicleState.dphi =    _gyroLatest.x;
                vehicleState.dtheta = -_gyroLatest.y; // negate for ENU
                vehicleState.dpsi =    _gyroLatest.z;
            }

            _didInit = true;
        }

    private:

        // Indexes to access the state
        enum {

            KC_STATE_E0,
            KC_STATE_E1,
            KC_STATE_E2,
            KC_STATE_DIM

        };

        typedef struct {
            axis3_t sum;
            uint32_t count;
            float conversionFactor;
            axis3_t subSample;
        } axis3_tSubSampler_t;

        //////////////////////////////////////////////////////////////////////////

        axis3_t _gyroLatest;

        axis3_tSubSampler_t _accSubSampler;
        axis3_tSubSampler_t _gyroSubSampler;

        // The covariance matrix
        float _Pmat[KC_STATE_DIM][KC_STATE_DIM];

        //////////////////////////////////////////////////////////////////////////

        void step_init(void)
        {
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY_MAGNITUDE);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            // set covariances to zero (diagonals will be changed from
            // zero in the next section)
            memset(_Pmat, 0, sizeof(_Pmat));

            // initialize state variances
            _Pmat[KC_STATE_E0][KC_STATE_E0] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
            _Pmat[KC_STATE_E1][KC_STATE_E1] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
            _Pmat[KC_STATE_E2][KC_STATE_E2] = square(STDEV_INITIAL_ATTITUDE_YAW);
        }

        void step_normal(
                bool & _isUpdated,
                float & _e0, float & _e1, float & _e2,
                float & _qw, float & _qx, float & _qy, float & _qz,
                float & _r20, float & _r21, float & _r22)

        {
            extern uint32_t stream_now_msec;

            const auto isFlying = true; // XXX

            static uint32_t _nextPredictionMsec;

            static uint32_t _lastPredictionMsec;

            const auto shouldPredict = stream_now_msec >= _nextPredictionMsec;

            _lastPredictionMsec = _lastPredictionMsec == 0 ? 
                stream_now_msec :
                _lastPredictionMsec;

            _nextPredictionMsec = 
                _nextPredictionMsec == 0 ? stream_now_msec : _nextPredictionMsec;

            axis3fSubSamplerFinalize(&_accSubSampler, shouldPredict);

            axis3fSubSamplerFinalize(&_gyroSubSampler, shouldPredict);

            const axis3_t * gyro = &_gyroSubSampler.subSample; 
            const float dt = (stream_now_msec - _lastPredictionMsec) / 1000.0f;

            const auto e0 = gyro->x*dt/2;
            const auto e1 = gyro->y*dt/2;
            const auto e2 = gyro->z*dt/2;

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
                //        E0    E1    E2
                /*E0*/   {e0e0, e0e1, e0e2}, 
                /*E1*/   {e1e0, e1e1, e1e2}, 
                /*E2*/   {e2e0, e2e1, e2e2}  
            };


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

            _qw = shouldPredict ? tmpq0/norm : _qw;
            _qx = shouldPredict ? tmpq1/norm : _qx; 
            _qy = shouldPredict ? tmpq2/norm : _qy; 
            _qz = shouldPredict ? tmpq3/norm : _qz;

            _isUpdated = shouldPredict ? true : _isUpdated;

            _lastPredictionMsec = shouldPredict ? stream_now_msec : _lastPredictionMsec;

            static uint32_t _lastUpdateMsec;

            _lastUpdateMsec = _lastUpdateMsec == 0 ?
                stream_now_msec :
                _lastUpdateMsec;

            const auto dt1 = (stream_now_msec - _lastUpdateMsec) / 1000.0f;

            const auto isDtPositive = dt1 > 0;

            _Pmat[KC_STATE_E0][KC_STATE_E0] += isDtPositive ?
                square(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT) : 0;

            _Pmat[KC_STATE_E1][KC_STATE_E1] += isDtPositive ?
                square(MEAS_NOISE_GYRO_ROLL_PITCH * dt1 + PROC_NOISE_ATT) : 0;

            _Pmat[KC_STATE_E2][KC_STATE_E2] += isDtPositive ?
                square(MEAS_NOISE_GYRO_ROLL_YAW * dt1 + PROC_NOISE_ATT) : 0;

            updateCovarianceMatrix(isDtPositive);

            _lastUpdateMsec = isDtPositive ?  
                stream_now_msec : 
                _lastUpdateMsec;

            _nextPredictionMsec = stream_now_msec >= _nextPredictionMsec ?
                stream_now_msec + PREDICTION_UPDATE_INTERVAL_MS :
                _nextPredictionMsec;

            extern float stream_accel_x, stream_accel_y, stream_accel_z;

            const auto accel = 
                axis3_t {stream_accel_x, stream_accel_y, stream_accel_z};

            axis3fSubSamplerAccumulate(&_accSubSampler, &accel);
        
            extern float stream_gyro_x, stream_gyro_y, stream_gyro_z;

            const auto raw_gyro = 
                axis3_t {stream_gyro_x, stream_gyro_y, stream_gyro_z};

            axis3fSubSamplerAccumulate(&_gyroSubSampler, &raw_gyro);

            memcpy(&_gyroLatest, &raw_gyro, sizeof(axis3_t));
        
            // Only finalize if data is updated
            if (_isUpdated) {
                doFinalize(
                        _isUpdated, 
                        _e0, _e1, _e2,
                        _qw, _qx, _qy, _qz, 
                        _r20, _r21, _r22);
            }
        }

        //////////////////////////////////////////////////////////////////////////

        void doFinalize(
                bool & _isUpdated,
                float & _e0, float & _e1, float & _e2,
                float & _qw, float & _qx, float & _qy, float & _qz,
                float & _r20, float & _r21, float & _r22)
        {
            // Incorporate the attitude error (Kalman filter state) with the attitude
            const auto v0 = _e0;
            const auto v1 = _e1;
            const auto v2 = _e2;

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
                //       E0     E1    E2
                /*E0*/  {e0e0, e0e1, e0e2},
                /*E1*/  {e1e0, e1e1, e1e2},
                /*E2*/  {e2e0, e2e1, e2e2}
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
            _e0 = 0;
            _e1 = 0;
            _e2 = 0;

            updateCovarianceMatrix(true);

            _isUpdated = false;

        } // doFinalize

        static float rotateQuat(
                const float val, 
                const float initVal,
                const bool isFlying)
        {
            const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

            return (val * (isFlying ? 1: keep)) +
                (isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
        }

        static void axis3fSubSamplerInit(axis3_tSubSampler_t* subSampler, const
                float conversionFactor) 
        { 
            memset(subSampler, 0, sizeof(axis3_tSubSampler_t));
            subSampler->conversionFactor = conversionFactor;
        }

        static void axis3fSubSamplerAccumulate(axis3_tSubSampler_t* subSampler,
                const axis3_t* sample) 
        {
            subSampler->sum.x += sample->x;
            subSampler->sum.y += sample->y;
            subSampler->sum.z += sample->z;

            subSampler->count++;
        }

        static axis3_t* axis3fSubSamplerFinalize(
                axis3_tSubSampler_t* subSampler,
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
            subSampler->sum = {0, 0, 0};
            subSampler->count = 0;

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

        static float max(const float val, const float maxval)
        {
            return val > maxval ? maxval : val;
        }

        static float square(const float val)
        {
            return val * val;
        }
};
