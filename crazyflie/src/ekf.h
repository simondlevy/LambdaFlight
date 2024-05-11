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

#define EKF_N 7
#include <tinyekf.hpp>

#include <datatypes.h>

// Initial variances, uncertain of position, but know we're
// stationary and roughly flat
static const float STDEV_INITIAL_POSITION_Z = 1;
static const float STDEV_INITIAL_VELOCITY = 0.01;
static const float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
static const float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

void getFlowUpdates(
        const float * x,
        const float dt, 
        const float dpixelx, 
        const float dpixely,
        float hx[7], 
        float & errx, 
        float hy[7], 
        float & erry, 
        float & stdev);

void initialize_crazyflie_ekf(void);

void accumulateGyro(const uint32_t nowMsec, const axis3_t & gyro) ;

void accumulateAccel(const uint32_t nowMsec, const axis3_t & accel);

void ekf_getVehicleState(const float * x, vehicleState_t & state);

bool isStateWithinBounds(const float * x);


bool shouldUpdateWithRange(const float * x, const uint32_t distance,
        float h[7], float & error, float & noise);
