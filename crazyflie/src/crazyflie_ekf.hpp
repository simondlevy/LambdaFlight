/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
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
 */

#pragma once

#include <datatypes.h>

class CrazyflieEkf : public EkfInterface {

    private:

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

        axis3_t _gyroLatest;

        new_quat_t _quat;

        uint32_t _nextPredictionMsec;

        axis3_t _r;

        imu_t _gyroSum;
        imu_t _accelSum;

    public:

        virtual void do_init(const float diag[EKF_N]) override
        {
        }

        virtual void get_prediction(
                const float dt,
                const bool didAddProcessNoise,
                const float xold[EKF_N],
                float xnew[EKF_N],
                float F[EKF_N][EKF_N]) override
        {
        }

        virtual bool did_finalize(float x[EKF_N], float A[EKF_N][EKF_N]) override
        {
            return false;
        }

};
