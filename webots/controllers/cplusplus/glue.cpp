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

#include <main.hpp>
#include <mixers/quadrotor.hpp>

static Miniflie miniflie;

extern vehicleState_t vehicleState;

bool inHoverMode;

void step(void)
{
    // Run miniflie algorithm on open-loop demands and vehicle state to 
    // get motor values
    float motorvals[4] = {};
    miniflie.step(false, motorvals);

    // Set simulated motor values
    void setMotors(float m1, float m2, float m3, float m4);
    setMotors(motorvals[0], motorvals[1], motorvals[2], motorvals[3]);
}

void init(void)
{
    inHoverMode = true;

    miniflie.init(mixQuadrotor);
}
