#pragma once

#include "num.hpp"

float constrain(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

float altitudePid(const float desired, const float measured)
{
    static const float KP = 2;
    static const float KI = 0.5;
    static const float DT = 0.01;

    static const float INTEGRATION_LIMIT = 5000;

    static float _integ;     

    auto error = desired - measured;

    _integ = constrain(_integ + error * DT, 
            -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

    return KP * error + KI * _integ;
}


static float climbRatePid(const float desired, const float measured)
{
    static const float KP = 25;
    static const float KI = 15;
    static const float DT = 0.01;

    static const float INTEGRATION_LIMIT = 5000;

    static float _integ;     

    auto error = desired - measured;

    _integ = constrain(_integ + error * DT, 
            -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

    return KP * error + KI * _integ;
}

static void runAltitudeController(const vehicleState_t & state, demands_t & demands)
{
    // Set climb rate based on target altitude
    auto climbRate = altitudePid(demands.thrust, state.z);

    // Set thrust for desired climb rate
    demands.thrust = climbRatePid(climbRate, state.dz);
}
