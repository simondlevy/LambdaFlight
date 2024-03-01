/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate
 * gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:

 @INPROCEEDINGS{MuellerHamerUWB2015,
 author = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
 title  = {Fusing ultra-wideband range measurements with accelerometers and rate 
 gyroscopes for quadrocopter state estimation},
 booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
 year   = {2015},
 month  = {May},
 pages  = {1730-1736},
 doi    = {10.1109/ICRA.2015.7139421},
 ISSN   = {1050-4729}}

 @ARTICLE{MuellerCovariance2016,
 author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
 title={Covariance Correction Step for Kalman Filtering with an Attitude},
 journal={Journal of Guidance, Control, and Dynamics},
 pages={1--7},
 year={2016},
 publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from 
 OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 * 2023.09.10, Simon D. Levy: Made a header-only C++ class
 */

#pragma once

#include <math3d.h>
#include <m_pi.h>
#include <datatypes.h>

#include <arm_math.h>

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, 
        arm_matrix_instance_f32 * pDst) 
{
  arm_mat_trans_f32(pSrc, pDst);
}

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, 
        const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) 
{
  arm_mat_mult_f32(pSrcA, pSrcB, pDst);
}

static inline float fast_sqrt(float32_t in) 
{
  float pOut = 0;
  arm_sqrt_f32(in, &pOut);
  return pOut;
}

// Quaternion used for initial orientation
static const float QW_INIT = 1;
static const float QX_INIT = 0;
static const float QY_INIT = 0;
static const float QZ_INIT = 0;

// Initial variances, uncertain of position, but know we're
// stationary and roughly flat
static const float STDEV_INITIAL_POSITION_XY = 100;
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

static const float DEGREES_TO_RADIANS = PI / 180.0f;
static const float RADIANS_TO_DEGREES = 180.0f / PI;

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

static const int MAX_ITER = 2;

static const float UPPER_BOUND = 100;
static const float LOWER_BOUND = -100;


class KalmanFilter { 

    public:

        typedef enum {

            MODE_INIT,
            MODE_PREDICT,
            MODE_UPDATE,
            MODE_FINALIZE,
            MODE_GET_STATE

        } mode_t;

        typedef enum {
            MeasurementTypeRange,
            MeasurementTypeFlow,
            MeasurementTypeGyroscope,
            MeasurementTypeAcceleration,
        } MeasurementType;

        typedef struct {

            MeasurementType type;

            union {

                rangeMeasurement_t range;
                flowMeasurement_t flow;
                gyroscopeMeasurement_t gyroscope;
                accelerationMeasurement_t acceleration;
            } data;

        } measurement_t;

        bool step(
                const mode_t mode, 
                const measurement_t &measurement,
                const uint32_t nowMsec,
                const uint32_t nextPredictionMsec,
                const bool isFlying,
                vehicleState_t & state)
        {
            bool success = true;

            switch (mode) {

                case MODE_INIT:
                    init(nowMsec);
                    break;

                case MODE_PREDICT:
                    predict(nowMsec, nextPredictionMsec, isFlying);
                    break;

                case MODE_UPDATE:
                    update(measurement, nowMsec);
                    break;

                case MODE_FINALIZE:
                    success = finalize();
                    break;

                case MODE_GET_STATE:
                    getVehicleState(state);
                    break;
            }

            return success;
        }

    private:

        // Indexes to access the quad's state, stored as a column vector
        typedef enum {

            KC_STATE_X,
            KC_STATE_Y,
            KC_STATE_Z,
            KC_STATE_PX,
            KC_STATE_PY,
            KC_STATE_PZ,
            KC_STATE_D0,
            KC_STATE_D1,
            KC_STATE_D2,
            KC_STATE_DIM

        } kalmanCoreStateIdx_t;

        typedef struct {

            float x;
            float y;
            float z;
            float px;
            float py;
            float pz;
            float d0;
            float d1;
            float d2;

        } kalmanState_t;

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

        /**
         * Vehicle State
         *
         * The internally-estimated state is:
         * - X, Y, Z: the quad's position in the global frame
         * - PX, PY, PZ: the quad's velocity in its body frame
         * - D0, D1, D2: attitude error
         *
         * For more information, refer to the paper
         */
        kalmanState_t _kalmanState;

        // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
        // to allow easy normalization (in comparison to a rotation matrix),
        // while also being robust against singularities (in comparison to euler angles)
        float _qw;
        float _qx;
        float _qy;
        float _qz;

        // The quad's attitude as a rotation matrix (used by the prediction,
        // updated by the finalization)
        float _r00;
        float _r01;
        float _r02;
        float _r10;
        float _r11;
        float _r12;
        float _r20;
        float _r21;
        float _r22;

        // The covariance matrix
        __attribute__((aligned(4))) float _P[KC_STATE_DIM][KC_STATE_DIM];
        arm_matrix_instance_f32 _Pm;

        // Tracks whether an update to the state has been made, and the state
        // therefore requires finalization
        bool _isUpdated;

        uint32_t _lastPredictionMs;
        uint32_t _lastProcessNoiseUpdateMs;

        //////////////////////////////////////////////////////////////////////////

        void getVehicleState(vehicleState_t & state)
        {
            state.x = _kalmanState.x;

            state.dx = _r00*_kalmanState.px + 
                _r01*_kalmanState.py + 
                _r02*_kalmanState.pz;

            state.y = _kalmanState.y;

            state.dy = _r10*_kalmanState.px + 
                _r11*_kalmanState.py + 
                _r12*_kalmanState.pz;

            state.z = _kalmanState.z;

            state.dz = _r20*_kalmanState.px + 
                _r21*_kalmanState.py + 
                _r22*_kalmanState.pz;

            state.phi = RADIANS_TO_DEGREES *
                atan2f(2*(_qy*_qz+_qw*
                            _qx) ,
                        _qw*_qw -
                        _qx*_qx -
                        _qy*_qy +
                        _qz*_qz);

            state.theta = -RADIANS_TO_DEGREES * // note negation
                asinf(-2*(_qx*_qz -
                            _qw*_qy));

            state.psi = RADIANS_TO_DEGREES *
                atan2f(2*(_qx*_qy+_qw*
                            _qz)
                        , _qw*_qw +
                        _qx*_qx -
                        _qy*_qy -
                        _qz*_qz);

            // Get angular velocities directly from gyro
            state.dphi =    _gyroLatest.x;     
            state.dtheta = -_gyroLatest.y; // (negate for ENU)
            state.dpsi =    _gyroLatest.z; 
        }


        bool finalize(void)
        {
            // Matrix to rotate the attitude covariances once updated
            static float A[KC_STATE_DIM][KC_STATE_DIM];
            static arm_matrix_instance_f32 Am = {
                KC_STATE_DIM, KC_STATE_DIM, (float *)A
            };

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static arm_matrix_instance_f32 tmpNN1m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static arm_matrix_instance_f32 tmpNN2m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            }; 

            finalize(A, &Am, &tmpNN1m, &tmpNN2m);

            return isStateWithinBounds();
        }


        void update(const measurement_t & m, const uint32_t nowMs)
        {
            switch (m.type) {

                case MeasurementTypeRange:
                    updateWithRange(&m.data.range);
                    break;

                case MeasurementTypeFlow:
                    updateWithFlow(&m.data.flow);
                    break;

                case MeasurementTypeGyroscope:
                    updateWithGyro(m);
                    break;

                case MeasurementTypeAcceleration:
                    updateWithAccel(m);
                    break;

                default:
                    break;
            }
        }

        void predict(
                const uint32_t nowMs, 
                const uint32_t nextPredictionMs,
                const bool isFlying) 
        {
            if (nowMs >= nextPredictionMs) {

                predict(nowMs, isFlying); 
            }

            // Add process noise every loop, rather than every prediction
            addProcessNoise(nowMs);
        }

        void init(const uint32_t nowMs)
        {
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY_MAGNITUDE);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            // Reset all data to 0 (like upon system reset)

            memset(&_kalmanState, 0, sizeof(_kalmanState));
            memset(&_P, 0, sizeof(_P));
            memset(&_Pm, 0, sizeof(_Pm));

            _isUpdated = false;
            _lastPredictionMs = 0;
            _lastProcessNoiseUpdateMs = 0;

            _kalmanState.x = 0;
            _kalmanState.y = 0;
            _kalmanState.z = 0;

            _qw = QW_INIT;
            _qx = QX_INIT;
            _qy = QY_INIT;
            _qz = QZ_INIT;

            // set the initial rotation matrix to the identity. This only affects
            // the first prediction step, since in the finalization, after shifting
            // attitude errors into the attitude state, the rotation matrix is updated.
            _r00 = 1;
            _r01 = 0;
            _r02 = 0;
            _r10 = 0;
            _r11 = 1;
            _r12 = 0;
            _r20 = 0;
            _r21 = 0;
            _r22 = 1;

            // set covariances to zero (diagonals will be changed from
            // zero in the next section)
            for (int i=0; i< KC_STATE_DIM; i++) {

                for (int j=0; j < KC_STATE_DIM; j++) {

                    _P[i][j] = 0; 
                }
            }

            // initialize state variances
            _P[KC_STATE_X][KC_STATE_X] = powf(STDEV_INITIAL_POSITION_XY, 2);
            _P[KC_STATE_Y][KC_STATE_Y] = powf(STDEV_INITIAL_POSITION_XY, 2);
            _P[KC_STATE_Z][KC_STATE_Z] = powf(STDEV_INITIAL_POSITION_Z, 2);

            _P[KC_STATE_PX][KC_STATE_PX] = powf(STDEV_INITIAL_VELOCITY, 2);
            _P[KC_STATE_PY][KC_STATE_PY] = powf(STDEV_INITIAL_VELOCITY, 2);
            _P[KC_STATE_PZ][KC_STATE_PZ] = powf(STDEV_INITIAL_VELOCITY, 2);

            _P[KC_STATE_D0][KC_STATE_D0] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
            _P[KC_STATE_D1][KC_STATE_D1] = powf(STDEV_INITIAL_ATTITUDE_ROLL_PITCH, 2);
            _P[KC_STATE_D2][KC_STATE_D2] = powf(STDEV_INITIAL_ATTITUDE_YAW, 2);

            _Pm.numRows = KC_STATE_DIM;
            _Pm.numCols = KC_STATE_DIM;
            _Pm.pData = (float*)_P;

            _isUpdated = false;
            _lastPredictionMs = nowMs;
            _lastProcessNoiseUpdateMs = nowMs;
        }

        void predictDt(Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
        {
            // The linearized update matrix
            static float A[KC_STATE_DIM][KC_STATE_DIM];
            static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { 
                KC_STATE_DIM, KC_STATE_DIM, (float *)A
            }; // linearized dynamics for covariance update;

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { 
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { 
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            };

            predictDt(A, &Am, &tmpNN1m, &tmpNN2m, acc, gyro, dt, quadIsFlying);
        }

        void finalize(
                float  A[KC_STATE_DIM][KC_STATE_DIM],
                arm_matrix_instance_f32 * Am,
                arm_matrix_instance_f32 * tmpNN1m,
                arm_matrix_instance_f32 * tmpNN2m)
        {
            // Only finalize if data is updated
            if (! _isUpdated) {
                return;
            }

            // Incorporate the attitude error (Kalman filter state) with the attitude
            auto v0 = _kalmanState.d0;
            auto v1 = _kalmanState.d1;
            auto v2 = _kalmanState.d2;

            // Move attitude error into attitude if any of the angle errors are
            // large enough
            if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) >
                        0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 &&
                            fabsf(v2) < 10)) 
            {
                auto angle = fast_sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
                auto ca = arm_cos_f32(angle / 2.0f);
                auto sa = arm_sin_f32(angle / 2.0f);

                auto dqw = ca;
                auto dqx = sa * v0 / angle;
                auto dqy = sa * v1 / angle;
                auto dqz = sa * v2 / angle;

                // Rotate the quad's attitude by the delta quaternion vector
                // computed above
                auto tmpq0 = dqw * _qw - dqx * _qx - dqy * _qy - dqz * _qz;
                auto tmpq1 = dqx * _qw + dqw * _qx + dqz * _qy - dqy * _qz;
                auto tmpq2 = dqy * _qw - dqz * _qx + dqw * _qy + dqx * _qz;
                auto tmpq3 = dqz * _qw + dqy * _qx - dqx * _qy + dqw * _qz;

                // normalize and store the result
                auto norm = fast_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                        tmpq3 * tmpq3) + EPS;

                _qw = tmpq0 / norm;
                _qx = tmpq1 / norm;
                _qy = tmpq2 / norm;
                _qz = tmpq3 / norm;

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
                // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
                float d0 = v0/2; 
                float d1 = v1/2; 
                float d2 = v2/2;

                A[KC_STATE_X][KC_STATE_X] = 1;
                A[KC_STATE_Y][KC_STATE_Y] = 1;
                A[KC_STATE_Z][KC_STATE_Z] = 1;

                A[KC_STATE_PX][KC_STATE_PX] = 1;
                A[KC_STATE_PY][KC_STATE_PY] = 1;
                A[KC_STATE_PZ][KC_STATE_PZ] = 1;

                A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
                A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

                A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
                A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

                A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
                A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
                A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

                mat_trans(Am, tmpNN1m); // A'
                mat_mult(Am, &_Pm, tmpNN2m); // AP
                mat_mult(tmpNN2m, tmpNN1m, &_Pm); //APA'
            }

            // Convert the new attitude to a rotation matrix, such that we can
            // rotate body-frame velocity and acc

            _r00 = _qw * _qw + _qx * _qx - _qy * _qy - _qz * _qz;
            _r01 = 2 * _qx * _qy - 2 * _qw * _qz; 
            _r02 = 2 * _qx * _qz + 2 * _qw * _qy; 
            _r10 = 2 * _qx * _qy + 2 * _qw * _qz;
            _r11 = _qw * _qw - _qx * _qx + _qy * _qy - _qz * _qz;
            _r12 = 2 * _qy * _qz - 2 * _qw * _qx; 
            _r20 = 2 * _qx * _qz - 2 * _qw * _qy;
            _r21 = 2 * _qy * _qz + 2 * _qw * _qx;
            _r22 = _qw * _qw - _qx * _qx - _qy * _qy + _qz * _qz;

            // reset the attitude error
            _kalmanState.d0 = 0;
            _kalmanState.d1 = 0;
            _kalmanState.d2 = 0;

            // enforce symmetry of the covariance matrix, and ensure the values
            // stay bounded
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float p = 0.5f*_P[i][j] + 0.5f*_P[j][i];
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _P[i][j] = _P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _P[i][j] = _P[j][i] = MIN_COVARIANCE;
                    } else {
                        _P[i][j] = _P[j][i] = p;
                    }
                }
            }

            _isUpdated = false;
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

        static Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* subSampler) 
        {
            if (subSampler->count > 0) {
                subSampler->subSample.x = 
                    subSampler->sum.x * subSampler->conversionFactor / subSampler->count;
                subSampler->subSample.y = 
                    subSampler->sum.y * subSampler->conversionFactor / subSampler->count;
                subSampler->subSample.z = 
                    subSampler->sum.z * subSampler->conversionFactor / subSampler->count;

                // Reset
                subSampler->count = 0;
                subSampler->sum = (Axis3f){.axis={0}};
            }

            return &subSampler->subSample;
        }

        void addProcessNoiseDt(float dt)
        {
            _P[KC_STATE_X][KC_STATE_X] += 
                powf(PROC_NOISE_ACC_XY*dt*dt + PROC_NOISE_VEL*dt + 
                        PROC_NOISE_POS, 2);  // add process noise on position

            _P[KC_STATE_Y][KC_STATE_Y] += 
                powf(PROC_NOISE_ACC_XY*dt*dt + PROC_NOISE_VEL*dt + 
                        PROC_NOISE_POS, 2);  // add process noise on position

            _P[KC_STATE_Z][KC_STATE_Z] += 
                powf(PROC_NOISE_ACC_Z*dt*dt + PROC_NOISE_VEL*dt + 
                        PROC_NOISE_POS, 2);  // add process noise on position

            _P[KC_STATE_PX][KC_STATE_PX] += 
                powf(PROC_NOISE_ACC_XY*dt + 
                        PROC_NOISE_VEL, 2); // add process noise on velocity

            _P[KC_STATE_PY][KC_STATE_PY] += 
                powf(PROC_NOISE_ACC_XY*dt + 
                        PROC_NOISE_VEL, 2); // add process noise on velocity

            _P[KC_STATE_PZ][KC_STATE_PZ] += 
                powf(PROC_NOISE_ACC_Z*dt + 
                        PROC_NOISE_VEL, 2); // add process noise on velocity

            _P[KC_STATE_D0][KC_STATE_D0] += 
                powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt + PROC_NOISE_ATT, 2);
            _P[KC_STATE_D1][KC_STATE_D1] += 
                powf(MEAS_NOISE_GYRO_ROLL_PITCH * dt + PROC_NOISE_ATT, 2);
            _P[KC_STATE_D2][KC_STATE_D2] += 
                powf(MEAS_NOISE_GYRO_ROLL_YAW * dt + PROC_NOISE_ATT, 2);

            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float p = 0.5f*_P[i][j] + 0.5f*_P[j][i];
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _P[i][j] = _P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _P[i][j] = _P[j][i] = MIN_COVARIANCE;
                    } else {
                        _P[i][j] = _P[j][i] = p;
                    }
                }
            }
        }

        void predictDt(
                float A[KC_STATE_DIM][KC_STATE_DIM],
                arm_matrix_instance_f32 * Am,
                arm_matrix_instance_f32 * tmpNN1m,
                arm_matrix_instance_f32 * tmpNN2m,
                Axis3f *acc, 
                Axis3f *gyro, 
                float dt, 
                bool quadIsFlying)
        {
            /* Here we discretize (euler forward) and linearise the quadrocopter
             * dynamics in order to push the covariance forward.
             *
             * QUADROCOPTER DYNAMICS (see paper):
             *
             * \dot{x} = R(I + [[d]])p
             * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
             * \dot{d} = \omega
             *
             * where [[.]] is the cross-product matrix of .
             *       \omega are the gyro measurements
             *       e3 is the column vector [0 0 1]'
             *       I is the identity
             *       R is the current attitude as a rotation matrix
             *       f/m is the mass-normalized motor force (acceleration in the body's z 
             direction)
             *       g is gravity
             *       x, p, d are the quad's states
             * note that d (attitude error) is zero at the beginning of each iteration,
             * since error information is incorporated into R after each Kalman update.
             */

            // ====== DYNAMICS LINEARIZATION ======
            // Initialize as the identity
            A[KC_STATE_X][KC_STATE_X] = 1;
            A[KC_STATE_Y][KC_STATE_Y] = 1;
            A[KC_STATE_Z][KC_STATE_Z] = 1;

            A[KC_STATE_PX][KC_STATE_PX] = 1;
            A[KC_STATE_PY][KC_STATE_PY] = 1;
            A[KC_STATE_PZ][KC_STATE_PZ] = 1;

            A[KC_STATE_D0][KC_STATE_D0] = 1;
            A[KC_STATE_D1][KC_STATE_D1] = 1;
            A[KC_STATE_D2][KC_STATE_D2] = 1;

            // position from body-frame velocity
            A[KC_STATE_X][KC_STATE_PX] = _r00*dt;
            A[KC_STATE_Y][KC_STATE_PX] = _r10*dt;
            A[KC_STATE_Z][KC_STATE_PX] = _r20*dt;

            A[KC_STATE_X][KC_STATE_PY] = _r01*dt;
            A[KC_STATE_Y][KC_STATE_PY] = _r11*dt;
            A[KC_STATE_Z][KC_STATE_PY] = _r21*dt;

            A[KC_STATE_X][KC_STATE_PZ] = _r02*dt;
            A[KC_STATE_Y][KC_STATE_PZ] = _r12*dt;
            A[KC_STATE_Z][KC_STATE_PZ] = _r22*dt;

            // position from attitude error
            A[KC_STATE_X][KC_STATE_D0] = (_kalmanState.py*_r02 - 
                    _kalmanState.pz*_r01)*dt;
            A[KC_STATE_Y][KC_STATE_D0] = (_kalmanState.py*_r12 - 
                    _kalmanState.pz*_r11)*dt;
            A[KC_STATE_Z][KC_STATE_D0] = (_kalmanState.py*_r22 - 
                    _kalmanState.pz*_r21)*dt;

            A[KC_STATE_X][KC_STATE_D1] = (- _kalmanState.px*_r02 + 
                    _kalmanState.pz*_r00)*dt;
            A[KC_STATE_Y][KC_STATE_D1] = (- _kalmanState.px*_r12 + 
                    _kalmanState.pz*_r10)*dt;
            A[KC_STATE_Z][KC_STATE_D1] = (- _kalmanState.px*_r22 + 
                    _kalmanState.pz*_r20)*dt;

            A[KC_STATE_X][KC_STATE_D2] = (_kalmanState.px*_r01 - 
                    _kalmanState.py*_r00)*dt;
            A[KC_STATE_Y][KC_STATE_D2] = (_kalmanState.px*_r11 - 
                    _kalmanState.py*_r10)*dt;
            A[KC_STATE_Z][KC_STATE_D2] = (_kalmanState.px*_r21 - 
                    _kalmanState.py*_r20)*dt;

            // body-frame velocity from body-frame velocity
            A[KC_STATE_PX][KC_STATE_PX] = 1; //drag negligible
            A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
            A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;

            A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
            A[KC_STATE_PY][KC_STATE_PY] = 1; //drag negligible
            A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;

            A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
            A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
            A[KC_STATE_PZ][KC_STATE_PZ] = 1; //drag negligible

            // body-frame velocity from attitude error
            A[KC_STATE_PX][KC_STATE_D0] =  0;
            A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*_r22*dt;
            A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*_r21*dt;

            A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*_r22*dt;
            A[KC_STATE_PY][KC_STATE_D1] =  0;
            A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*_r20*dt;

            A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*_r21*dt;
            A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*_r20*dt;
            A[KC_STATE_PZ][KC_STATE_D2] =  0;


            // attitude error from attitude error
            /**
             * At first glance, it may not be clear where the next values come from,
             * since they do not appear directly in the dynamics. In this prediction
             * step, we skip the step of first updating attitude-error, and then
             * incorporating the
             * new error into the current attitude (which requires a rotation of the
             * attitude-error covariance). Instead, we directly update the body attitude,
             * however still need to rotate the covariance, which is what you see below.
             *
             * This comes from a second order approximation to:
             * Sigma_post = exps(-d) Sigma_pre exps(-d)'
             *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + 
             *             [[-d]]^2 / 2)'
             * where d is the attitude error expressed as Rodriges parameters, ie. d0 =
             * 1/2*gyro.x*dt under the assumption that d = [0,0,0] at the beginning of
             * each prediction step and that gyro.x is constant over the sampling period
             *
             * As derived in "Covariance Correction Step for Kalman Filtering with an 
             Attitude"
             * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
             */
            float d0 = gyro->x*dt/2;
            float d1 = gyro->y*dt/2;
            float d2 = gyro->z*dt/2;

            A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
            A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
            A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

            A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
            A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
            A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

            A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
            A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
            A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

            // ====== COVARIANCE UPDATE ======
            mat_mult(Am, &_Pm, tmpNN1m); // A P
            mat_trans(Am, tmpNN2m); // A'
            mat_mult(tmpNN1m, tmpNN2m, &_Pm); // A P A'
            // Process noise is added after the return from the prediction step

            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust (hence is useless
            // to estimate body angle while flying)

            float dx, dy, dz;
            float tmpSPX, tmpSPY, tmpSPZ;
            float zacc;

            float dt2 = dt*dt;

            if (quadIsFlying) { // only acceleration in z direction

                // Use accelerometer and not commanded thrust, as this has
                // proper physical units
                zacc = acc->z;

                // position updates in the body frame (will be rotated to inertial frame)
                dx = _kalmanState.px * dt;
                dy = _kalmanState.py * dt;
                dz = _kalmanState.pz * dt + zacc * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _kalmanState.x += _r00 * dx + _r01 * dy + _r02 * dz;
                _kalmanState.y += _r10 * dx + _r11 * dy + _r12 * dz;
                _kalmanState.z += _r20 * dx + _r21 * dy + _r22 * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                tmpSPX = _kalmanState.px;
                tmpSPY = _kalmanState.py;
                tmpSPZ = _kalmanState.pz;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _kalmanState.px += dt * (gyro->z * tmpSPY - gyro->y *
                        tmpSPZ - GRAVITY_MAGNITUDE * _r20);
                _kalmanState.py += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - 
                        GRAVITY_MAGNITUDE * _r21);
                _kalmanState.pz += dt * (zacc + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _r22);
            }
            else {
                // Acceleration can be in any direction, as measured by the
                // accelerometer. This occurs, eg. in freefall or while being carried.

                // position updates in the body frame (will be rotated to inertial frame)
                dx = _kalmanState.px * dt + acc->x * dt2 / 2.0f;
                dy = _kalmanState.py * dt + acc->y * dt2 / 2.0f;
                dz = _kalmanState.pz * dt + acc->z * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _kalmanState.x += _r00 * dx 
                    + _r01 * dy + _r02 * dz;
                _kalmanState.y += _r10 * dx + 
                    _r11 * dy + _r12 * dz;
                _kalmanState.z += _r20 * dx + 
                    _r21 * dy + _r22 * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                tmpSPX = _kalmanState.px;
                tmpSPY = _kalmanState.py;
                tmpSPZ = _kalmanState.pz;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _kalmanState.px += dt * (acc->x + gyro->z * tmpSPY -
                        gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * _r20);
                _kalmanState.py += dt * (acc->y - gyro->z * tmpSPX + gyro->x * 
                        tmpSPZ - GRAVITY_MAGNITUDE * _r21);
                _kalmanState.pz += dt * (acc->z + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _r22);
            }

            // attitude update (rotate by gyroscope), we do this in quaternions
            // this is the gyroscope angular velocity integrated over the sample period
            auto dtwx = dt*gyro->x;
            auto dtwy = dt*gyro->y;
            auto dtwz = dt*gyro->z;

            // compute the quaternion values in [w,x,y,z] order
            auto angle = fast_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
            auto ca = arm_cos_f32(angle/2.0f);
            auto sa = arm_sin_f32(angle/2.0f);
            auto dqw = ca;
            auto dqx = sa*dtwx/angle;
            auto dqy = sa*dtwy/angle;
            auto dqz = sa*dtwz/angle;

            // rotate the quad's attitude by the delta quaternion vector computed above

            auto tmpq0 = dqw*_qw - dqx*_qx - dqy*_qy - dqz*_qz;
            auto tmpq1 = dqx*_qw + dqw*_qx + dqz*_qy - dqy*_qz;
            auto tmpq2 = dqy*_qw - dqz*_qx + dqw*_qy + dqx*_qz;
            auto tmpq3 = dqz*_qw + dqy*_qx - dqx*_qy + dqw*_qz;

            if (! quadIsFlying) {

                float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * QW_INIT;
                tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * QX_INIT;
                tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * QY_INIT;
                tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * QZ_INIT;
            }

            // normalize and store the result
            float norm = fast_sqrt(
                    tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPS;

            _qw = tmpq0/norm; 
            _qx = tmpq1/norm; 
            _qy = tmpq2/norm; 
            _qz = tmpq3/norm;


            _isUpdated = true;
        }

        void scalarUpdate(
                arm_matrix_instance_f32 *Hm, 
                float error, 
                float stdMeasNoise)
        {
            // The Kalman gain as a column vector
            static float K[KC_STATE_DIM];
            static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static arm_matrix_instance_f32 tmpNN1m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static arm_matrix_instance_f32 tmpNN2m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            };

            static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
            static arm_matrix_instance_f32 tmpNN3m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN3d
            };

            static float HTd[KC_STATE_DIM * 1];
            static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

            static float PHTd[KC_STATE_DIM * 1];
            static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

            scalarUpdate(Hm, &HTm, &Km, PHTd,
                    K, tmpNN1d, &PHTm, &tmpNN1m, &tmpNN2m, &tmpNN3m, 
                    error, stdMeasNoise);
        }

        void scalarUpdate(
                arm_matrix_instance_f32 *Hm, 
                arm_matrix_instance_f32 *HTm, 
                arm_matrix_instance_f32 *Km, 
                float * PHTd, 
                float * K, 
                float * tmpNN1d, 
                arm_matrix_instance_f32 *PHTm, 
                arm_matrix_instance_f32 *tmpNN1m, 
                arm_matrix_instance_f32 *tmpNN2m, 
                arm_matrix_instance_f32 *tmpNN3m, 
                float error, 
                float stdMeasNoise)
        {
            // ====== INNOVATION COVARIANCE ======

            mat_trans(Hm, HTm);
            mat_mult(&_Pm, HTm, PHTm); // PH'
            float R = stdMeasNoise*stdMeasNoise;
            float HPHR = R; // HPH' + R
            for (int i=0; i<KC_STATE_DIM; i++) { 

                // Add the element of HPH' to the above

                // this obviously only works if the update is scalar (as in this function)
                HPHR += Hm->pData[i]*PHTd[i]; 
            }

            // ====== MEASUREMENT UPDATE ======
            // Calculate the Kalman gain and perform the state update
            for (int i=0; i<KC_STATE_DIM; i++) {
                K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
            }
            _kalmanState.x  += K[0] * error;
            _kalmanState.y  += K[1] * error;
            _kalmanState.z  += K[2] * error;
            _kalmanState.px += K[3] * error;
            _kalmanState.py += K[4] * error;
            _kalmanState.pz += K[5] * error;
            _kalmanState.d0 += K[6] * error;
            _kalmanState.d1 += K[7] * error;
            _kalmanState.d2 += K[8] * error;


            // ====== COVARIANCE UPDATE ======
            mat_mult(Km, Hm, tmpNN1m); // KH
            for (int i=0; i<KC_STATE_DIM; i++) { 
                tmpNN1d[KC_STATE_DIM*i+i] -= 1; 
            } // KH - I
            mat_trans(tmpNN1m, tmpNN2m); // (KH - I)'
            mat_mult(tmpNN1m, &_Pm, tmpNN3m); // (KH - I)*P
            mat_mult(tmpNN3m, tmpNN2m, &_Pm); // (KH - I)*P*(KH - I)'

            // add the measurement variance and ensure boundedness and symmetry
            // TODO: Why would it hit these bounds? Needs to be investigated.
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float v = K[i] * R * K[j];

                    // add measurement noise
                    float p = 0.5f*_P[i][j] + 0.5f*_P[j][i] + v; 
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _P[i][j] = _P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _P[i][j] = _P[j][i] = MIN_COVARIANCE;
                    } else {
                        _P[i][j] = _P[j][i] = p;
                    }
                }
            }

            _isUpdated = true;
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
            float omegax_b = gyro->x * DEGREES_TO_RADIANS;
            float omegay_b = gyro->y * DEGREES_TO_RADIANS;

            // ~~~ Moves the body velocity into the global coordinate system ~~~
            // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
            //
            // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
            // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
            //
            // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
            // _G and _B refer to the global/body coordinate systems.

            // Modification 1
            //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * 
            //  S[KC_STATE_PZ];
            //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * 
            //  S[KC_STATE_PZ];


            float dx_g = _kalmanState.px;
            float dy_g = _kalmanState.py;
            float z_g = 0.0;
            // Saturate elevation in prediction and correction to avoid singularities
            if ( _kalmanState.z < 0.1f ) {
                z_g = 0.1;
            } else {
                z_g = _kalmanState.z;
            }

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            float hx[KC_STATE_DIM] = {};
            arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
            auto predictedNX = (flow->dt * Npix / thetapix ) * 
                ((dx_g * _r22 / z_g) - omegay_b);
            auto measuredNX = flow->dpixelx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dx_g) / (-z_g * z_g));
            hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * 
                (_r22 / z_g);

            //First update
            scalarUpdate(&Hx, (measuredNX-predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION);

            // ~~~ Y velocity prediction and update ~~~
            float hy[KC_STATE_DIM] = {};
            arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
            auto predictedNY = (flow->dt * Npix / thetapix ) * 
                ((dy_g * _r22 / z_g) + omegax_b);
            auto measuredNY = flow->dpixely*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dy_g) / (-z_g * z_g));
            hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (_r22 / z_g);

            // Second update
            scalarUpdate(
                    &Hy, 
                    (measuredNY-predictedNY), 
                    flow->stdDevY*FLOW_RESOLUTION);
        }

        void updateWithRange(const rangeMeasurement_t *range)
        {
            // Updates the filter with a measured distance in the zb direction using the
            float h[KC_STATE_DIM] = {};
            arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

            // Only update the filter if the measurement is reliable 
            // (\hat{h} -> infty when R[2][2] -> 0)
            if (fabs(_r22) > 0.1f && _r22 > 0) {
                float angle = 
                    fabsf(acosf(_r22)) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f);
                if (angle < 0.0f) {
                    angle = 0.0f;
                }
                float predictedDistance = _kalmanState.z / cosf(angle);
                float measuredDistance = range->distance; // [m]


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

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                h[KC_STATE_Z] = 1 / cosf(angle); 

                // Scalar update
                scalarUpdate(
                        &H, measuredDistance-predictedDistance, range->stdDev);
            }
        }

        void updateWithAccel(const measurement_t & m)
        {
            axis3fSubSamplerAccumulate(&_accSubSampler, &m.data.acceleration.acc);
        }

        void updateWithGyro(const measurement_t & m)
        {
            axis3fSubSamplerAccumulate(&_gyroSubSampler, &m.data.gyroscope.gyro);
            _gyroLatest = m.data.gyroscope.gyro;
        }

        void predict(const uint32_t nowMs, bool quadIsFlying) 
        {
            axis3fSubSamplerFinalize(&_accSubSampler);
            axis3fSubSamplerFinalize(&_gyroSubSampler);

            float dt = (nowMs - _lastPredictionMs) / 1000.0f;

            predictDt(&_accSubSampler.subSample, &_gyroSubSampler.subSample, dt,
                    quadIsFlying);

            _lastPredictionMs = nowMs;
        }

        void addProcessNoise(const uint32_t nowMs) 
        {
            float dt = (nowMs - _lastProcessNoiseUpdateMs) / 1000.0f;

            if (dt > 0.0f) {
                addProcessNoiseDt(dt);
                _lastProcessNoiseUpdateMs = nowMs;
            }
        }

        bool isStateWithinBounds(void) 
        {
            return 
                isPositionWithinBounds(_kalmanState.x) &&
                isPositionWithinBounds(_kalmanState.y) &&
                isPositionWithinBounds(_kalmanState.z) &&
                isVelocityWithinBounds(_kalmanState.px) &&
                isVelocityWithinBounds(_kalmanState.py) &&
                isVelocityWithinBounds(_kalmanState.pz);
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
