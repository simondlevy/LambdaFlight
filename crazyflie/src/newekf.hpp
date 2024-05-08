#pragma once

class Ekf {

    private:

        typedef struct {

            float dat[EKF_N];

        } myvector_t;

        typedef struct {

            float dat[EKF_N][EKF_N];

        } mymatrix_t;

    public:

        void init(void)
        {
            memset(&p, 0, sizeof(_p));

            _p.dat[STATE_Z][STATE_Z] = square(STDEV_INITIAL_POSITION_Z);
            _p.dat[STATE_DX][STATE_DX] = square(STDEV_INITIAL_VELOCITY);
            _p.dat[STATE_DY][STATE_DY] = square(STDEV_INITIAL_VELOCITY);
            _p.dat[STATE_DZ][STATE_DZ] = square(STDEV_INITIAL_VELOCITY);
            _p.dat[STATE_E0][STATE_E0] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
            _p.dat[STATE_E1][STATE_E1] = square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH);
            _p.dat[STATE_E2][STATE_E2] = square(STDEV_INITIAL_ATTITUDE_YAW);

            memset(&_x, 0, sizeof(_x));

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

        void predict(const bool isFlying) 
        {
            myvector_t x_predicted = {};

            new_quat_t quat_predicted = {};

            _predict(
                    isFlying, 
                    _gyro, 
                    _accel, 
                    _x, 
                    _quat, 
                    _r, 
                    _lastPredictionMsec, 
                    _p, 
                    quat_predicted, 
                    x_predicted);

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
            }
        }

        void updateWithFlow(void)
        {
            _isUpdated = true;
        }

        void updateWithRange(const float rangefinderDistance)
        {
            const auto angle = max(0, 
                    fabsf(acosf(_r.z)) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f));

            const auto predictedDistance = get(_x, STATE_Z) / cosf(angle);

            const auto measuredDistance = rangefinderDistance / 1000; // mm => m

            const auto stdDev =
                RANGEFINDER_EXP_STD_A * 
                (1 + expf(RANGEFINDER_EXP_COEFF * 
                          (measuredDistance - RANGEFINDER_EXP_POINT_A)));

            myvector_t h = {};
            set(h, STATE_Z, 1 / cosf(angle));

            if (fabs(_r.z) > 0.1f && _r.z > 0 && 
                    rangefinderDistance < RANGEFINDER_OUTLIER_LIMIT_MM) {

                _scalarUpdate(h, measuredDistance-predictedDistance, stdDev, _p, _x);

                _isUpdated = true;
            }
        }

        void finalize(void)
        {
        }

    private:

        // Quaternion used for initial orientation
        static constexpr float QW_INIT = 1;
        static constexpr float QX_INIT = 0;
        static constexpr float QY_INIT = 0;
        static constexpr float QZ_INIT = 0;

        // Initial variances, uncertain of position, but know we're
        // stationary and roughly flat
        static constexpr float STDEV_INITIAL_POSITION_Z = 1;
        static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
        static constexpr float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
        static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

        static constexpr float MSS_TO_GS = 9.81;

        //We do get the measurements in 10x the motion pixels (experimentally measured)
        static constexpr float FLOW_RESOLUTION = 0.1;

        // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
        static constexpr float MAX_COVARIANCE = 100;
        static constexpr float MIN_COVARIANCE = 1e-6;

        // The bounds on states, these shouldn't be hit...
        static constexpr float MAX_POSITION = 100; //meters
        static constexpr float MAX_VELOCITY = 10; //meters per second

        // Small number epsilon, to prevent dividing by zero
        static constexpr float EPS = 1e-6f;

        // the reversion of pitch and roll to zero
        static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

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

        float x[EKF_N];

        float p[EKF_N][EKF_N];

        uint32_t _lastProcessNoiseUpdateMsec;
        uint32_t _lastPredictionMsec;
        bool _isUpdated;

        mymatrix_t _p;
        myvector_t _x;

        axis3_t _gyroLatest;

        new_quat_t _quat;

        uint32_t _nextPredictionMsec;

        axis3_t _r;

        imu_t _gyro;
        imu_t _accel;

        static void makemat(const float dat[EKF_N][EKF_N], mymatrix_t & a)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {
                for (uint8_t j=0; j<EKF_N; ++j) {
                    a.dat[i][j] = dat[i][j];
                }
            }

        }

        static void transpose(const mymatrix_t & a, mymatrix_t & at)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {
                for (uint8_t j=0; j<EKF_N; ++j) {
                    auto tmp = a.dat[i][j];
                    at.dat[i][j] = a.dat[j][i];
                    at.dat[j][i] = tmp;
                }
            }
        }

        static float dot(const myvector_t & x, const myvector_t & y) 
        {
            float d = 0;

            for (uint8_t k=0; k<EKF_N; k++) {
                d += x.dat[k] * y.dat[k];
            }

            return d;
        }

        static float get(const mymatrix_t & a, const uint8_t i, const uint8_t j)
        {
            return a.dat[i][j];
        }

        static float get(const myvector_t & x, const uint8_t i)
        {
            return x.dat[i];
        }

        static void set(myvector_t & x, const uint8_t i, const float val)
        {
            x.dat[i] = val;
        }

        static void set(mymatrix_t & a, const uint8_t i, const uint8_t j, const float val)
        {
            a.dat[i][j] = val;
        }

        static float dot(
                const mymatrix_t & a, 
                const mymatrix_t & b, 
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
        static void multiply( const mymatrix_t a, const mymatrix_t b, mymatrix_t & c)
        {
            for (uint8_t i=0; i<EKF_N; i++) {

                for (uint8_t j=0; j<EKF_N; j++) {

                    c.dat[i][j] = dot(a, b, i, j);
                }
            }
        }

        // Matrix * Vector
        static void multiply(const mymatrix_t & a, const myvector_t & x, myvector_t & y)
        {
            for (uint8_t i=0; i<EKF_N; i++) {
                y.dat[i] = 0;
                for (uint8_t j=0; j<EKF_N; j++) {
                    y.dat[i] += a.dat[i][j] * x.dat[j];
                }
            }
        }

        // Outer product
        static void multiply(const myvector_t & x, const myvector_t & y, mymatrix_t & a)
        {
            for (uint8_t i=0; i<EKF_N; i++) {
                for (uint8_t j=0; j<EKF_N; j++) {
                    a.dat[i][j] = x.dat[i] * y.dat[j];
                }
            }
        }

        static void updateCovarianceMatrix(mymatrix_t & p)
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

        static void _predict(
                const bool isFlying,
                const imu_t & gyro,
                const imu_t & accel,
                const myvector_t & x_in,
                const new_quat_t & q,
                const axis3_t & r,
                const uint32_t lastPredictionMsec, 
                mymatrix_t & p,
                new_quat_t & quat_out,
                myvector_t &x_out)
        {
            static axis3_t _gyro;
            static axis3_t _accel;

            const float dt = (stream_nowMsec - lastPredictionMsec) / 1000.0f;
            const auto dt2 = dt * dt;

            imuTakeMean(gyro, DEGREES_TO_RADIANS, _gyro);
            imuTakeMean(accel, MSS_TO_GS, _accel);

            // Position updates in the body frame (will be rotated to inertial frame);
            // thrust can only be produced in the body's Z direction
            const auto dx = get(x_in, STATE_DX) * dt + isFlying ? 0 : _accel.x * dt2 / 2;
            const auto dy = get(x_in, STATE_DY) * dt + isFlying ? 0 : _accel.y * dt2 / 2;
            const auto dz = get(x_in, STATE_DZ) * dt + _accel.z * dt2 / 2; 

            const auto accx = isFlying ? 0 : _accel.x;
            const auto accy = isFlying ? 0 : _accel.y;

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

            mymatrix_t A = {};
            makemat(Adat, A);

            mymatrix_t  At = {};
            transpose(A, At);     // A'
            mymatrix_t AP = {};
            multiply(A, p, AP);  // AP
            multiply(AP, At, p); // APA'
            updateCovarianceMatrix(p);
        }

        static void _scalarUpdate(
                const myvector_t & h, const float error, const float stdMeasNoise,
                mymatrix_t & p, myvector_t & x)
        {

            // ====== INNOVATION COVARIANCE ======
            myvector_t ph = {};
            multiply(p, h, ph);
            const auto r = stdMeasNoise * stdMeasNoise;
            const auto hphr = r + dot(h, ph); // HPH' + R

            // Compute the Kalman gain as a column vector
            myvector_t g = {};
            for (uint8_t i=0; i<EKF_N; ++i) {
                set(g, i, get(ph, i) / hphr);
            }

            // Perform the state update
            for (uint8_t i=0; i<EKF_N; ++i) {
                set(x, i, get(x, i) + get(g, i) * error);
            }

            // ====== COVARIANCE UPDATE ======

            mymatrix_t GH = {};
            multiply(g, h, GH); // KH

            for (int i=0; i<EKF_N; i++) { 
                set(GH, i, i, get(GH, i, i) - 1);
            } // KH - I

            mymatrix_t GHt = {};
            transpose(GH, GHt);      // (KH - I)'
            mymatrix_t GHIP = {};
            multiply(GH, p, GHIP);  // (KH - I)*P
            multiply(GHIP, GHt, p); // (KH - I)*P*(KH - I)'

            // Add the measurement variance 
            for (int i=0; i<EKF_N; i++) {
                for (int j=0; j<EKF_N; j++) {
                    p.dat[i][j] += j < i ? 0 : r * get(g, i) * get(g, j);
                    set(p, i, j, get(p, i, j));
                }
            }

            updateCovarianceMatrix(p);
        }};
