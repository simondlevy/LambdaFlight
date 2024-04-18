#pragma once

#include <string.h>

#include "math3d.h"
#include "datatypes.h"

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
static const float MEAS_NOISE_GYRO = 0.1; // radians per second

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

static const uint8_t N = 3;

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

extern float stream_gyro_x, stream_gyro_y, stream_gyro_z;
extern float stream_accel_x, stream_accel_y, stream_accel_z;
extern uint32_t stream_now_msec;

class Ekf { 

    public:

        static void step(vehicleState_t & vehicleState)
        {

            static bool _didInit;

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            static bool _isUpdated;

            static float _p[3][3];

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

            static float _gyro_sum_x;
            static float _gyro_sum_y;
            static float _gyro_sum_z;
            static float _gyro_sample_x;
            static float _gyro_sample_y;
            static float _gyro_sample_z;
            static uint32_t _gyro_count;

            static float _accel_sum_x;
            static float _accel_sum_y;
            static float _accel_sum_z;
            static float _accel_sample_x;
            static float _accel_sample_y;
            static float _accel_sample_z;
            static uint32_t _accel_count;

            _gyro_sum_x = !_didInit ? 0 : _gyro_sum_x;
            _gyro_sum_y = !_didInit ? 0 : _gyro_sum_y;
            _gyro_sum_z = !_didInit ? 0 : _gyro_sum_z;
            _gyro_sample_x = !_didInit ? 0 : _gyro_sample_x;
            _gyro_sample_y = !_didInit ? 0 : _gyro_sample_y;
            _gyro_sample_z = !_didInit ? 0 : _gyro_sample_z;
            _gyro_count = !_didInit ? 0 : _gyro_count;

            _accel_sum_x = !_didInit ? 0 : _accel_sum_x;
            _accel_sum_y = !_didInit ? 0 : _accel_sum_y;
            _accel_sum_z = !_didInit ? 0 : _accel_sum_z;
            _accel_sample_x = !_didInit ? 0 : _accel_sample_x;
            _accel_sample_y = !_didInit ? 0 : _accel_sample_y;
            _accel_sample_z = !_didInit ? 0 : _accel_sample_z;
            _accel_count = !_didInit ? 0 : _accel_count;

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

            _p[0][0] = !_didInit ? square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH) : _p[0][0];
            _p[0][1] = !_didInit ? 0 : _p[0][1];
            _p[0][2] = !_didInit ? 0 : _p[0][2];

            _p[1][0] = !_didInit ? 0 : _p[1][0];
            _p[1][1] = !_didInit ? square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH) : _p[1][1];
            _p[1][2] = !_didInit ? 0 : _p[1][2];

            _p[2][0] = !_didInit ? 0 : _p[2][0];
            _p[2][1] = !_didInit ? 0 : _p[2][1];
            _p[2][2] = !_didInit ? square(STDEV_INITIAL_ATTITUDE_YAW) : _p[2][2];

            if (_didInit) {

                step_normal(
                        _p,
                        _p,
                        _isUpdated, 
                        _gyro_sum_x, _gyro_sum_y, _gyro_sum_z,
                        _gyro_sample_x, _gyro_sample_y, _gyro_sample_z,
                        _gyro_count,
                        _accel_sum_x, _accel_sum_y, _accel_sum_z,
                        _accel_sample_x, _accel_sample_y, _accel_sample_z,
                        _accel_count,
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
                vehicleState.dphi =    stream_gyro_x;
                vehicleState.dtheta = -stream_gyro_y; // negate for ENU
                vehicleState.dpsi =    stream_gyro_z;
            }

            _didInit = true;
        }

    private:

        static void step_normal(
                const float p_in[3][3],
                float p_out[3][3],
                bool & _isUpdated,
                float & _gyro_sum_x, float & _gyro_sum_y, float & _gyro_sum_z,
                float & _gyro_sample_x, float & _gyro_sample_y, float & _gyro_sample_z,
                uint32_t & _gyro_count,
                float & _accel_sum_x, float & _accel_sum_y, float & _accel_sum_z,
                float & _accel_sample_x, float & _accel_sample_y, float & _accel_sample_z,
                uint32_t & _accel_count,
                float & _e0, float & _e1, float & _e2,
                float & _qw, float & _qx, float & _qy, float & _qz,
                float & _r20, float & _r21, float & _r22)

        {
            const auto isFlying = true; // XXX

            static uint32_t _nextPredictionMsec;

            static uint32_t _lastPredictionMsec;

            const auto shouldPredict = stream_now_msec >= _nextPredictionMsec;

            const float dt = (stream_now_msec - _lastPredictionMsec) / 1000.0f;

            const auto e0 = _gyro_sample_x*dt/2;
            const auto e1 = _gyro_sample_y*dt/2;
            const auto e2 = _gyro_sample_z*dt/2;

            const auto e0e0 =  1 - e1*e1/2 - e2*e2/2;
            const auto e0e1 =  e2 + e0*e1/2;
            const auto e0e2 = -e1 + e0*e2/2;

            const auto e1e0 = -e2 + e0*e1/2;
            const auto e1e1 =  1 - e0*e0/2 - e2*e2/2;
            const auto e1e2 =  e0 + e1*e2/2;

            const auto e2e0 =  e1 + e0*e2/2;
            const auto e2e1 = -e0 + e1*e2/2;
            const auto e2e2 = 1 - e0*e0/2 - e1*e1/2;

            const float A[3][3] = 
            { 
                //        E0    E1    E2
                /*E0*/   {e0e0, e0e1, e0e2}, 
                /*E1*/   {e1e0, e1e1, e1e2}, 
                /*E2*/   {e2e0, e2e1, e2e2}  
            };


            // attitude update (rotate by gyroscope), we do this in quaternions
            // this is the gyroscope angular velocity integrated over
            // the sample period
            const auto dtwx = dt*_gyro_sample_x;
            const auto dtwy = dt*_gyro_sample_y;
            const auto dtwz = dt*_gyro_sample_z;

            // compute the quaternion values in [w,x,y,z] order
            const auto angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
            const auto ca = cos(angle/2);
            const auto sa = sin(angle/2);
            const auto dqw = ca;
            const auto dqx = sa*dtwx/angle;
            const auto dqy = sa*dtwy/angle;
            const auto dqz = sa*dtwz/angle;

            // rotate the quad's attitude by the delta quaternion
            // vector computed above

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

            float At[3][3] = {};
            transpose(A, At);     // A'
            float AP[3][3] = {};
            multiply(A, p_in, AP);  // AP
            float APA[3][3] = {};
            multiply(AP, At, APA); // APA'

            _lastPredictionMsec = _lastPredictionMsec == 0 ? 
                stream_now_msec :
                _lastPredictionMsec;

            _nextPredictionMsec = 
                _nextPredictionMsec == 0 ? stream_now_msec : _nextPredictionMsec;

            subsamplerFinalize(shouldPredict, DEGREES_TO_RADIANS, 
                    _gyro_sum_x,
                    _gyro_sum_y,
                    _gyro_sum_z,
                    _gyro_sample_x,
                    _gyro_sample_y,
                    _gyro_sample_z,
                    _gyro_count);

            subsamplerFinalize(shouldPredict, GRAVITY_MAGNITUDE, 
                    _accel_sum_x,
                    _accel_sum_y,
                    _accel_sum_z,
                    _accel_sample_x,
                    _accel_sample_y,
                    _accel_sample_z,
                    _accel_count);

            // Process noise is added after the return from the prediction step

            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust
            // (hence is useless to estimate body angle while flying)

            _qw = shouldPredict ? tmpq0/norm : _qw;
            _qx = shouldPredict ? tmpq1/norm : _qx; 
            _qy = shouldPredict ? tmpq2/norm : _qy; 
            _qz = shouldPredict ? tmpq3/norm : _qz;

            _isUpdated = shouldPredict ? true : _isUpdated;

            _lastPredictionMsec = shouldPredict ? 
                stream_now_msec : 
                _lastPredictionMsec;

            static uint32_t _lastUpdateMsec;

            _lastUpdateMsec = _lastUpdateMsec == 0 ?
                stream_now_msec :
                _lastUpdateMsec;

            const auto dt1 = (stream_now_msec - _lastUpdateMsec) / 1000.0f;

            const auto isDtPositive = dt1 > 0;

            const auto noise =
                isDtPositive ?  
                square(MEAS_NOISE_GYRO * dt1 + PROC_NOISE_ATT) :
                0;

            p_out[0][0] = shouldPredict ? APA[0][0] + noise : p_in[0][0];

            p_out[0][1] = shouldPredict ? APA[0][1] : p_in[0][1];

            p_out[0][2] = shouldPredict ? APA[0][2] : p_in[0][2];

            p_out[1][0] = shouldPredict ? APA[1][0] : p_in[1][0];

            p_out[1][1] = shouldPredict ? APA[1][1] + noise: p_in[1][1];

            p_out[1][2] = shouldPredict ? APA[1][2] : p_in[1][2];

            p_out[2][0] = shouldPredict ? APA[2][0] : p_in[2][0];

            p_out[2][1] = shouldPredict ? APA[2][1] : p_in[2][1];

            p_out[2][2] = shouldPredict ? APA[2][2] + noise : p_in[2][2];

            updateCovarianceMatrix(p_out, isDtPositive, p_out);

            _lastUpdateMsec = isDtPositive ?  
                stream_now_msec : 
                _lastUpdateMsec;

            _nextPredictionMsec = stream_now_msec >= _nextPredictionMsec ?
                stream_now_msec + PREDICTION_UPDATE_INTERVAL_MS :
                _nextPredictionMsec;

            subsamplerAccumulate(
                    stream_gyro_x,
                    stream_gyro_y,
                    stream_gyro_z,
                    _gyro_sum_x,
                    _gyro_sum_y,
                    _gyro_sum_z,
                    _gyro_count);

            subsamplerAccumulate(
                    stream_accel_x,
                    stream_accel_y,
                    stream_accel_z,
                    _accel_sum_x,
                    _accel_sum_y,
                    _accel_sum_z,
                    _accel_count);

            // Incorporate the attitude error (Kalman filter state) with the attitude
            const auto v0 = _e0;
            const auto v1 = _e1;
            const auto v2 = _e2;

            const auto newangle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
            const auto newca = cos(newangle / 2.0f);
            const auto newsa = sin(newangle / 2.0f);

            const auto newdqw = newca;
            const auto newdqx = newsa * v0 / newangle;
            const auto newdqy = newsa * v1 / newangle;
            const auto newdqz = newsa * v2 / newangle;

            // Rotate the quad's attitude by the delta quaternion vector
            // computed above
            const auto newtmpq0 = newdqw * _qw - newdqx * _qx - newdqy * _qy - newdqz * _qz;
            const auto newtmpq1 = newdqx * _qw + newdqw * _qx + newdqz * _qy - newdqy * _qz;
            const auto newtmpq2 = newdqy * _qw - newdqz * _qx + newdqw * _qy + newdqx * _qz;
            const auto newtmpq3 = newdqz * _qw + newdqy * _qx - newdqx * _qy + newdqw * _qz;

            // normalize and store the result
            const auto newnorm = sqrt(newtmpq0 * newtmpq0 + newtmpq1 * newtmpq1 + newtmpq2 * newtmpq2 + 
                    newtmpq3 * newtmpq3) + EPS;

            // the attitude error vector (v0,v1,v2) is small,
            // so we use a first order approximation to e0 = tan(|v0|/2)*v0/|v0|
            const auto newe0 = v0/2; 
            const auto newe1 = v1/2; 
            const auto newe2 = v2/2;

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

            const auto newe0e0 =  1 - newe1*newe1/2 - newe2*newe2/2;
            const auto newe0e1 =  newe2 + newe0*newe1/2;
            const auto newe0e2 = -newe1 + newe0*newe2/2;

            const auto newe1e0 =  -newe2 + newe0*newe1/2;
            const auto newe1e1 = 1 - newe0*newe0/2 - newe2*newe2/2;
            const auto newe1e2 = newe0 + newe1*newe2/2;

            const auto newe2e0 = newe1 + newe0*newe2/2;
            const auto newe2e1 = -newe0 + newe1*newe2/2;
            const auto newe2e2 = 1 - newe0*newe0/2 - newe1*newe1/2;

            const auto isErrorSufficient  = 
                (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
                isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

            // Matrix to rotate the attitude covariances once updated
            const float newA[3][3] = 
            { 
                //       E0     E1    E2
                /*E0*/  {newe0e0, newe0e1, newe0e2},
                /*E1*/  {newe1e0, newe1e1, newe1e2},
                /*E2*/  {newe2e0, newe2e1, newe2e2}
            };

            float newAt[3][3] = {};
            transpose(newA, newAt);     // A'
            float newAP[3][3] = {};
            multiply(newA, p_out, newAP);  // AP
            float newAPA[3][3] = {};
            multiply(newAP, newAt, newAPA); // APA'

            // Only finalize if data is updated
            if (_isUpdated) {

                _qw = isErrorSufficient ? newtmpq0 / newnorm : _qw;
                _qx = isErrorSufficient ? newtmpq1 / newnorm : _qx;
                _qy = isErrorSufficient ? newtmpq2 / newnorm : _qy;
                _qz = isErrorSufficient ? newtmpq3 / newnorm : _qz;

                if (isErrorSufficient) {
                    memcpy(p_out, newAPA, 3*3*sizeof(float));
                }

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and acc
                _r20 = 2 * _qx * _qz - 2 * _qw * _qy;
                _r21 = 2 * _qy * _qz + 2 * _qw * _qx;
                _r22 = _qw * _qw - _qx * _qx - _qy * _qy + _qz * _qz;

                // Reset the attitude error
                _e0 = 0;
                _e1 = 0;
                _e2 = 0;

                updateCovarianceMatrix(p_out, true, p_out);
            }

            _isUpdated = false;
        }

        //////////////////////////////////////////////////////////////////////////


        static float rotateQuat(
                const float val, 
                const float initVal,
                const bool isFlying)
        {
            const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

            return (val * (isFlying ? 1: keep)) +
                (isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
        }

        static void subsamplerAccumulate(
                const float x,
                const float y,
                const float z,
                float & _sum_x,
                float & _sum_y,
                float & _sum_z,
                uint32_t & _count)
        {
            _sum_x += x;
            _sum_y += y;
            _sum_z += z;

            _count++;
        }

        static void subsamplerFinalize(
                const bool shouldPredict, 
                const float conversionFactor,
                float & sum_x,
                float & sum_y,
                float & sum_z,
                float & sample_x,
                float & sample_y,
                float & sample_z,
                uint32_t & count)

        {
            const auto isCountNonzero = count > 0;

            const auto shouldFinalize = shouldPredict && isCountNonzero;

            sample_x = shouldFinalize ? 
                sum_x * conversionFactor / count :
                sample_x;

            sample_y = shouldFinalize ?
                sum_y * conversionFactor / count :
                sample_y;

            sample_z = shouldFinalize ?
                sum_z * conversionFactor / count :
                sample_z;

            sum_x = 0;
            sum_y = 0;
            sum_z = 0;

            count = 0;
        }

        static void updateCovarianceMatrix(
                const float p_in[3][3],
                const bool shouldUpdate, 
                float _p[3][3])
        {
            // Enforce symmetry of the covariance matrix, and ensure the
            // values stay bounded
            for (int i=0; i<3; i++) {
                for (int j=i; j<3; j++) {
                    updateCovarianceCell(p_in, i, j, 0, shouldUpdate, _p);
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

        static void updateCovarianceCell(
                const float p_in[3][3],
                const int i, 
                const int j, 
                const float variance,
                const bool shouldUpdate,
                float p_out[3][3])
        {
            const auto pval = (p_in[i][j] + p_in[j][i]) / 2 + variance;

            p_out[i][j] = !shouldUpdate ? p_in[i][j] :
                (isnan(pval) || pval > MAX_COVARIANCE) ?  MAX_COVARIANCE :
                (i==j && pval < MIN_COVARIANCE) ?  MIN_COVARIANCE :
                pval;

            p_out[j][i] = shouldUpdate ? p_out[i][j] : p_in[j][i];
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
