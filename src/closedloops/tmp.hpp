#pragma once

static void runTmp(
        const bool hover, 
        const float dt,
        const vehicleState_t & state, 
        demands_t & demands)
{
    static const float ilimit = 5000;

    static float _integ;

    auto thrustraw = demands.thrust;

    auto target = Num::rescale(thrustraw, -1, +1, 0.2, 2.0);

    auto error = target - state.z;

    _integ = hover ? 
        Num::fconstrain(_integ + error, -ilimit, ilimit) : 
        0;

    auto thrustout = hover ? _integ : thrustraw;

    demands.thrust = thrustout;

    demands.roll = 0;
    demands.pitch = 0;
    demands.yaw = 0;
}
