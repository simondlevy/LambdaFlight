/**
 * Common simulator support for Webots
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

#pragma once

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <datatypes.h>

#include "sticks.hpp"

// https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
static const float ALTITUDE_TARGET_INITIAL = 0.4;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

static const float DT = .01;

static WbDeviceTag _m1_motor;
static WbDeviceTag _m2_motor;
static WbDeviceTag _m3_motor;
static WbDeviceTag _m4_motor;

// These are global so they can be shared with Haskell Copilot ---------------

vehicleState_t state;

demands_t demands;

void step(void);

void runMotors(float m1, float m2, float m3, float m4)
{
    // Set simulated motor values
    wb_motor_set_velocity(_m1_motor, +m1);
    wb_motor_set_velocity(_m2_motor, -m2);
    wb_motor_set_velocity(_m3_motor, +m3);
    wb_motor_set_velocity(_m4_motor, -m4);
}

// ---------------------------------------------------------------------------

static WbDeviceTag _makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

static float _rad2deg(const float rad)
{
    return rad / M_PI * 180;
}

static void _getVehicleState(
        WbDeviceTag & gyro, 
        WbDeviceTag & imu, 
        WbDeviceTag & gps)
{
    // Track previous time and position for calculating motion
    static float tprev;
    static float xprev;
    static float yprev;
    static float zprev;

    const auto tcurr = wb_robot_get_time();
    const auto dt =  tcurr - tprev;
    tprev = tcurr;

    // Get state values (meters, degrees) from ground truth:
    //   x: positive forward
    //   y: positive leftward
    //   z: positive upward
    //   phi, dphi: positive roll right
    //   theta,dtheta: positive nose up (requires negating imu, gyro)
    //   psi,dpsi: positive nose left
    state.x = wb_gps_get_values(gps)[0];
    state.y = wb_gps_get_values(gps)[1];
    state.z = wb_gps_get_values(gps)[2];
    state.phi =     _rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[0]);
    state.dphi =    _rad2deg(wb_gyro_get_values(gyro)[0]);
    state.theta =  -_rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[1]);
    state.dtheta = -_rad2deg(wb_gyro_get_values(gyro)[1]); 
    state.psi =     _rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[2]);
    state.dpsi =    _rad2deg(wb_gyro_get_values(gyro)[2]);

    // Use temporal first difference to get world-cooredinate velocities
    state.dx = (state.x - xprev) / dt;
    state.dy = (state.y - yprev) / dt;
    state.dz = (state.z - zprev) / dt;

    // Save past time and position for next time step
    xprev = state.x;
    yprev = state.y;
    zprev = state.z;
}

static WbDeviceTag _makeSensor(
        const char * name, 
        const uint32_t timestep,
        void (*f)(WbDeviceTag tag, int sampling_period))
{
    auto sensor = wb_robot_get_device(name);
    f(sensor, timestep);
    return sensor;
}

static float _constrain(const float val, const float lo, const float hi)
{
    return val < lo ? lo : val > hi ? hi : val;
}

static void run(void)
{
    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
    _m1_motor = _makeMotor("m1_motor", +1);
    _m2_motor = _makeMotor("m2_motor", -1);
    _m3_motor = _makeMotor("m3_motor", +1);
    _m4_motor = _makeMotor("m4_motor", -1);

    // Initialize sensors
    auto imu = _makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    auto gyro = _makeSensor("gyro", timestep, wb_gyro_enable);
    auto gps = _makeSensor("gps", timestep, wb_gps_enable);
    auto camera = _makeSensor("camera", timestep, wb_camera_enable);

    sticksInit();

    float altitudeTarget = 0;

    while (wb_robot_step(timestep) != -1) {

        //Un-comment if you want to try OpenCV
        // runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        sticksRead(demands.thrust, demands.roll, demands.pitch, demands.yaw);

        // Get vehicle state from sensors
        _getVehicleState(gyro, imu, gps);

        // Hover mode: integrate stick demand to get altitude target
        altitudeTarget = _constrain(
                altitudeTarget + demands.thrust * DT, 
                ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        printf("%f %f\n", demands.thrust, altitudeTarget);

        // Rescale altitude target to [-1,+1]
        demands.thrust = 2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
                (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;

        step();
    }

    wb_robot_cleanup();
}
