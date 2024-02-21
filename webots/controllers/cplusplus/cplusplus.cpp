/**
 * C++ simulator support for Webots
 *
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

#include <main.hpp>
#include <mixers/quadrotor.hpp>

//Un-comment if you want to try OpenCV
//#include "camera.hpp"

#include "../common.hpp"

static Miniflie miniflie;

void step(void)
{
    // Run miniflie algorithm on open-loop demands and vehicle state to 
    // get motor values
    float motorvals[4] = {};
    miniflie.step(true, state, demands, motorvals);

    // Set simulated motor values
    setMotors(motorvals[0], motorvals[1], motorvals[2], motorvals[3]);
}

void report(const float value)
{
    printf("%f\n", value);
}

int main(int argc, char ** argv)
{
    miniflie.init(mixQuadrotor);

    run();

    return 0;
}
