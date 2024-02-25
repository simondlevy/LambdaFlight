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
        const float desired, 
        const float measured,
        float &errorPrevious,
        float &errorIntegral)
{
    auto error = desired - measured;

    auto deriv = (error - errorPrevious) / dt;

    errorIntegral = Num::fconstrain(errorIntegral + error * dt, -ilimit, ilimit);

    errorPrevious = error;

    return kp * error + ki * errorIntegral + kd * deriv;
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

    static float _rollErrorIntegral;
    static float _rollErrorPrevious;

    static float _pitchErrorIntegral;
    static float _pitchErrorPrevious;

    if (reset) {

        _rollErrorIntegral = 0;
        _rollErrorPrevious = 0;
        _pitchErrorIntegral = 0;
        _pitchErrorPrevious = 0;
    }

    demands.roll = demands.thrust == 0 ? 0 :
        runpid(kp, ki, kd, ilimit, dt, demands.roll, state.dphi, 
                _rollErrorPrevious, _rollErrorIntegral);

    demands.pitch = demands.thrust == 0 ? 0 :
        runpid(kp, ki, kd, ilimit, dt, demands.pitch, state.dtheta,
                _pitchErrorPrevious, _pitchErrorIntegral);
}
