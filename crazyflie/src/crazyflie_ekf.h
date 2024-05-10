#pragma once

#define EKF_N 7
#include <ekf.hpp>

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

void accumulateGyro(const uint32_t nowMsec, const axis3_t & gyro) ;

void accumulateAccel(const uint32_t nowMsec, const axis3_t & accel);

void ekf_getVehicleState(const float * x, vehicleState_t & state);

bool isStateWithinBounds(const float * x);


bool shouldUpdateWithRange(const float * x, const uint32_t distance,
        float h[7], float & error, float & noise);
