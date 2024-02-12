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

#include "sticks.hpp"

static WbDeviceTag m1_motor;
static WbDeviceTag m2_motor;
static WbDeviceTag m3_motor;
static WbDeviceTag m4_motor;

static float m1;
static float m2;
static float m3;
static float m4;


//////////////////////////////////////////////////////////////////////////////

// Shared with Haskell Copilot

vehicleState_t state;

demands_t demands;

bool in_hover_mode;

extern "C" {

    void step(void);

    void runMotors(float m1, float m2, float m3, float m4)
    {
        // Set simulated motor values
        wb_motor_set_velocity(m1_motor, +m1);
        wb_motor_set_velocity(m2_motor, -m2);
        wb_motor_set_velocity(m3_motor, +m3);
        wb_motor_set_velocity(m4_motor, -m4);
    }

    void setDemands(const float t, const float r, const float p, const float y)
    {
        demands.thrust = t;
        demands.roll = r;
        demands.pitch = p;
        demands.yaw = y;
    }
}

//////////////////////////////////////////////////////////////////////////////

static float fconstrain(const float val, const float lo, const float hi)
{
    return val < lo ? lo : val > hi ? hi : val;
}

static WbDeviceTag makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

static float rad2deg(const float rad)
{
    return rad / M_PI * 180;
}

static void getVehicleState(
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
    state.phi =     rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[0]);
    state.dphi =    rad2deg(wb_gyro_get_values(gyro)[0]);
    state.theta =  -rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[1]);
    state.dtheta = -rad2deg(wb_gyro_get_values(gyro)[1]); 
    state.psi =     rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[2]);
    state.dpsi =    rad2deg(wb_gyro_get_values(gyro)[2]);

    // Use temporal first difference to get world-cooredinate velocities
    state.dx = (state.x - xprev) / dt;
    state.dy = (state.y - yprev) / dt;
    state.dz = (state.z - zprev) / dt;

    // Save past time and position for next time step
    xprev = state.x;
    yprev = state.y;
    zprev = state.z;
}

static WbDeviceTag makeSensor(
        const char * name, 
        const uint32_t timestep,
        void (*f)(WbDeviceTag tag, int sampling_period))
{
    auto sensor = wb_robot_get_device(name);
    f(sensor, timestep);
    return sensor;
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

    while (wb_robot_step(timestep) != -1) {

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        sticksRead(demands);

        // Reset closed-loop controllers on zero thrust
        if (demands.thrust == 0) {

            demands.roll = 0;
            demands.pitch = 0;
            demands.yaw = 0;
        }

        // Check where we're in hover mode (button press on game controller)
        in_hover_mode = sticksInHoverMode();

        // Altitude target, normalized to [-1,+1]
        static float _altitudeTarget;

        // Get vehicle state from sensors
        getVehicleState(gyro, imu, gps);

        // Hover mode: integrate stick demand
        if (in_hover_mode) {
            const float DT = .01;
            _altitudeTarget = fconstrain(_altitudeTarget + demands.thrust * DT, -1, +1);
            demands.thrust = _altitudeTarget;
        }

        // Non-hover mode: use raw stick value with min 0
        else {
            demands.thrust = fconstrain(demands.thrust, 0, 1);
            _altitudeTarget = 0;
        }

        // Call Haskell Copilot
        step();

        auto t = demands.thrust;
        auto r = demands.roll;
        auto p = demands.pitch;
        auto y = demands.yaw;

        auto m1 = t - r + p  + y;
        auto m2 = t - r - p  - y;
        auto m3 = t + r - p  + y;
        auto m4 = t + r + p  - y;

        runMotors(m1, m2, m3, m4);
    }

    wb_robot_cleanup();

    return 0;
}
