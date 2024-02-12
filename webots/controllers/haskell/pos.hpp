#pragma once

static float runX(const float desired, const float measured)
{
    static const float kp = 25;
    static const float ki = 1;
    static const float dt = 0.01;

    auto error = desired - measured;

    static float _integ;

    _integ += error * dt;

    return -(kp * error + ki * _integ);
}

static float runY(const float desired, const float measured)
{
    static const float kp = 25;
    static const float ki = 1;
    static const float dt = 0.01;

    auto error = desired - measured;

    static float _integ;

    _integ += error * dt;

    return -(kp * error + ki * _integ);
}

static float deg2rad(float degrees) 
{ 
    return (M_PI / 180.0f) * degrees; 
}

/**
 * Demands are input as normalized interval [-1,+1] and output as
 * angles in degrees.
 *
 * roll:  input left positive => output negative
 *
 * pitch: input forward positive => output negative
 */
static void runPositionPid(
        const bool inHoverMode, 
        const vehicleState_t & state, 
        demands_t & demands) 
{
    // Rotate world-coordinate velocities into body coordinates
    const auto dxw = state.dx;
    const auto dyw = state.dy;
    const auto psi = deg2rad(state.psi);
    const auto cospsi = cos(psi);
    const auto sinpsi = sin(psi);
    const auto dxb =  dxw * cospsi + dyw * sinpsi;
    const auto dyb = -dxw * sinpsi + dyw * cospsi;       

    // Run PID closedloops on body-coordinate velocities
    demands.roll = inHoverMode ? runY(demands.roll, dyb) : 30 * demands.roll;
    demands.pitch = inHoverMode ? runX(demands.pitch, dxb) : 30 * demands.pitch;
}
