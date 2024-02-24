#pragma once

static void runTmp(
        const bool hover, 
        const float dt,
        const vehicleState_t & state, 
        demands_t & demands)
{
    static float _integ;

    _integ = hover ? _integ + 1 : 0;

    demands.thrust = _integ; 

    demands.roll = 0;
    demands.pitch = 0;
    demands.yaw = 0;
}
