/**
 * Haskell Copilot simulator support for Webots
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

#include <time.h>

#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "sticks.hpp"

// ----------------------------------------------------------------------------

// https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
static const float ALTITUDE_TARGET_INITIAL = 0.4;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

static const float DT = .01;


/*
void setMotors(float m1, float m2, float m3, float m4)
{
}*/

// ---------------------------------------------------------------------------

static Sticks _sticks;

static WbDeviceTag _makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

/*
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

static uint32_t _timesec(void)
{
    struct timespec spec = {};
    clock_gettime(CLOCK_REALTIME, &spec);
    return spec.tv_sec; 
}*/

int main(int argc, char ** argv)
{

    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
   auto m1_motor = _makeMotor("m1_motor", +1);
   auto m2_motor = _makeMotor("m2_motor", -1);
   auto m3_motor = _makeMotor("m3_motor", +1);
   auto m4_motor = _makeMotor("m4_motor", -1);

    // Initialize sensors
    //auto imu = _makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    //auto gyro = _makeSensor("gyro", timestep, wb_gyro_enable);
    //auto gps = _makeSensor("gps", timestep, wb_gps_enable);
    //auto camera = _makeSensor("camera", timestep, wb_camera_enable);

    _sticks.init();

    //auto altitudeTarget = ALTITUDE_TARGET_INITIAL;

    //auto sec_start = _timesec();

    //auto inHoverMode = false;
    //auto resetPids = false;

    // Start emitter for simulating radio signals to vehicle
    auto emitter = wb_robot_get_device("emitter");
    if (!emitter) {
        printf("emitter is not available.\n");
    }

    while (wb_robot_step(timestep) != -1) {

        /*
        //Un-comment if you want to try OpenCV

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        float throttle = 0;
        float roll = 0;
        float pitch = 0;
        float yaw = 0;
        auto inHoverMode = false;
        _sticks.read(throttle, roll, pitch, yaw, inHoverMode);

        auto thrust = throttle;

        // Adjust roll for positive leftward
        roll = -roll;

        // Get vehicle state from sensors
        _getVehicleState(gyro, imu, gps);

        // Integrate stick demand to get altitude target
        altitudeTarget = _constrain(
        altitudeTarget + thrust * DT, 
        ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        // Rescale altitude target to [-1,+1]
        thrust = 2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
        (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;

        // copilot_core_step();

        report(sec_start);

        // setup emitter buffer
        if (command[0] || command[1] || command[2]) {
        printf("command = ( %g , %g , %g )\n", 
        command[0], command[1], command[2]);
        wb_emitter_send(emitter, command, sizeof(command));
        }

         */

        // Set simulated motor values
        float mvel = 100;
        wb_motor_set_velocity(m1_motor, +mvel);
        wb_motor_set_velocity(m2_motor, -mvel);
        wb_motor_set_velocity(m3_motor, +mvel);
        wb_motor_set_velocity(m4_motor, -mvel);
    }

    wb_robot_cleanup();

    return 0;
}
