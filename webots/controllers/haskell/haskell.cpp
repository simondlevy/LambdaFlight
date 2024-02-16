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

#include <stdio.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <datatypes.h>

#include "../sticks.hpp"
#include "../common.hpp"

static float m1;
static float m2;
static float m3;
static float m4;

//////////////////////////////////////////////////////////////////////////////

// Shared with Haskell Copilot

bool hover = true;

void step(void);

void report(float value)
{
    printf("%f\n", value);
}

//////////////////////////////////////////////////////////////////////////////

static float fconstrain(const float val, const float lo, const float hi)
{
    return val < lo ? lo : val > hi ? hi : val;
}

int main(int argc, char ** argv)
{
    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
    m1_motor = makeMotor("m1_motor", +1);
    m2_motor = makeMotor("m2_motor", -1);
    m3_motor = makeMotor("m3_motor", +1);
    m4_motor = makeMotor("m4_motor", -1);

    // Initialize sensors
    auto imu = makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    auto gyro = makeSensor("gyro", timestep, wb_gyro_enable);
    auto gps = makeSensor("gps", timestep, wb_gps_enable);
    auto camera = makeSensor("camera", timestep, wb_camera_enable);

    sticksInit();

    float altitudeTarget = 0;

    while (wb_robot_step(timestep) != -1) {

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        sticksRead(demands);

        // Get vehicle state from sensors
        getVehicleState(gyro, imu, gps);

        // Hover mode: integrate stick demand to get altitude target
        altitudeTarget = fconstrain(
                altitudeTarget + demands.thrust * DT, 
                ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        // Rescale altitude target to [-1,+1]
        demands.thrust = 2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
                (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;


        // Call Haskell Copilot, which will call runMotors()
        step();
    }

    wb_robot_cleanup();

    return 0;
}
