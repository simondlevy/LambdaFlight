/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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

#include <num.hpp>

static float runpid(
        const float kp,
        const float ki,
        const float kd,
        const float ilimit,
        const float dt,
        const float error,
        float errorPrev,
        float errorInteg)
{
    auto deriv = (error - errorPrev) / dt;

    return kp * error + ki * errorInteg + kd * deriv;
}

static void runPitchRollRate(
        const bool reset, 
        const float dt,
        const vehicleState_t & state, 
        demands_t & demands)
{
    const float kp = 125;
    const float ki = 250;
    const float kd = 1.25;
    const float ilimit = 33;

    const auto isThrustZero = demands.thrust == 0;

    // --------------------------------------------------------

    static float _rollErrorInteg;
    static float _rollErrorPrev;

    auto rollError = demands.roll - state.dphi;

    demands.roll = isThrustZero ? 0 :
        runpid(kp, ki, kd, ilimit, dt, rollError,
                _rollErrorPrev, _rollErrorInteg);

    _rollErrorPrev = reset ? 0 : 
        isThrustZero ? _rollErrorPrev : 
        rollError;

    _rollErrorInteg = reset ? 0 : 
        isThrustZero ? _rollErrorInteg : 
        Num::fconstrain(_rollErrorInteg + rollError * dt, -ilimit, ilimit);

    // --------------------------------------------------------

    static float _pitchErrorInteg;
    static float _pitchErrorPrev;

    _pitchErrorInteg = reset ? 0 : _pitchErrorInteg;

    auto pitchError = demands.pitch - state.dtheta;

    demands.pitch = isThrustZero ? 0 :
        runpid(kp, ki, kd, ilimit, dt, pitchError,
                _pitchErrorPrev, _pitchErrorInteg);

    _pitchErrorPrev = 
        reset ? 0 : 
        isThrustZero ? _pitchErrorPrev : 
        pitchError;

    _pitchErrorInteg = reset ? 0 : 
        isThrustZero ? _pitchErrorInteg : 
        Num::fconstrain(_pitchErrorInteg + pitchError * dt, -ilimit, ilimit);
}
