/**
 * Copyright (C) 2024 Simon D. Levy
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

#include <miniflie.hpp>
#include <mixers/quadrotor.hpp>

//Un-comment if you want to try OpenCV
//#include "camera.hpp"

#include "../common.hpp"

static const Clock::rate_t PID_UPDATE_RATE = Clock::RATE_100_HZ;

// These constants allow our PID constants to be in the same intervals as in
// the actual vehicle
static const float THRUST_BASE = 48;
static const float THRUST_SCALE = 0.25;
static const float THRUST_MIN = 0;
static const float THRUST_MAX   = 60;
static const float PITCH_ROLL_SCALE = 1e-4;
static const float YAW_SCALE = 4e-5;

static Miniflie miniflie;

void step(void)
{
    // Run miniflie algorithm on open-loop demands and vehicle state to 
    // get motor values
    float motorvals[4] = {};
    miniflie.step(true, state, demands, motorvals);

    // Set simulated motor values
    runMotors(motorvals[0], motorvals[1], motorvals[2], motorvals[3]);
}

int main(int argc, char ** argv)
{
    miniflie.init(
            mixQuadrotor,
            PID_UPDATE_RATE,
            THRUST_SCALE,
            THRUST_BASE,
            THRUST_MIN,
            THRUST_MAX,
            PITCH_ROLL_SCALE,
            YAW_SCALE);

    run();

    return 0;
}
