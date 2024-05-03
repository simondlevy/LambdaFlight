#pragma once

#include <string.h>

#include <BasicLinearAlgebra.h>

#include "math3d.h"
#include "datatypes.h"

// Quaternion used for initial orientation
static const float QW_INIT = 1;
static const float QX_INIT = 0;
static const float QY_INIT = 0;
static const float QZ_INIT = 0;

// Small number epsilon, to prevent dividing by zero
static const float EPS = 1e-6f;

extern float stream_gyro_x, stream_gyro_y, stream_gyro_z;
extern float stream_accel_x, stream_accel_y, stream_accel_z;
extern float stream_rangefinder_distance;
extern uint32_t stream_now_msec;

class Ekf { 

    public:

        static void step(/*vehicleState_t & vehicleState*/)
        {
            // Internal state ------------------------------------------------

            static bool _didInit;

            // Quaternion (angular state components)
            static float _qw, _qx, _qy, _qz;

            _qw = !_didInit ? QW_INIT : _qw;
            _qx = !_didInit ? QX_INIT : _qx;
            _qy = !_didInit ? QY_INIT : _qy;
            _qz = !_didInit ? QZ_INIT : _qz;

            _didInit = true;

            // Predict --------------------------------------------------------

            const auto isFlying = true; // XXX

            const float dt = 0.005;

            const auto gyro_sample_x = stream_gyro_x * DEGREES_TO_RADIANS;
            const auto gyro_sample_y = stream_gyro_y * DEGREES_TO_RADIANS;
            const auto gyro_sample_z = stream_gyro_z * DEGREES_TO_RADIANS;

            const auto dtwx = dt * gyro_sample_x;
            const auto dtwy = dt * gyro_sample_y;
            const auto dtwz = dt * gyro_sample_z;

            const auto angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
            const auto ca = cos(angle/2);
            const auto sa = sin(angle/2);
            const auto dqw = ca;
            const auto dqx = sa*dtwx/angle;
            const auto dqy = sa*dtwy/angle;
            const auto dqz = sa*dtwz/angle;

            const auto tmpq0 = rotateQuat(
                    dqw*_qw - dqx*_qx - dqy*_qy - dqz*_qz, QW_INIT, isFlying);

            const auto tmpq1 = rotateQuat(
                    dqx*_qw + dqw*_qx + dqz*_qy - dqy*_qz, QX_INIT, isFlying);

            const auto tmpq2 = rotateQuat(
                    dqy*_qw - dqz*_qx + dqw*_qy + dqx*_qz, QY_INIT, isFlying);

            const auto tmpq3 = rotateQuat(
                    dqz*_qw + dqy*_qx - dqx*_qy + dqw*_qz, QZ_INIT, isFlying);

            const auto norm = 
                sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
                EPS;

            Serial.printf("c: %f\n", tmpq0/norm);

            _qw = tmpq0/norm;
            _qx = tmpq1/norm; 
            _qy = tmpq2/norm; 
            _qz = tmpq3/norm;
        }

    private:

        static float rotateQuat(
                const float val, 
                const float initVal,
                const bool isFlying)
        {
            const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

            return (val * (isFlying ? 1: keep)) +
                (isFlying ? 0 : ROLLPITCH_ZERO_REVERSION * initVal);
        }
};
