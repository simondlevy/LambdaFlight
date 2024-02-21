/**
 * Main C++ class for real and simulated flight controllers.
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

#include <math.h>

#include <clock.hpp>
#include <datatypes.h>
#include <num.hpp>

#include <closedloops/altitude.hpp>
#include <closedloops/climbrate.hpp>
#include <closedloops/pitchroll_angle.hpp>
#include <closedloops/pitchroll_rate.hpp>
#include <closedloops/position.hpp>
#include <closedloops/yaw_angle.hpp>
#include <closedloops/yaw_rate.hpp>

#include <constants.h>

class Miniflie {

    public:

        void init(const mixFun_t mixFun)
        {
            _mixFun = mixFun;

            _altitudeController.init(PID_UPDATE_RATE);
            _climbRateController.init(PID_UPDATE_RATE);
            _pitchRollAngleController.init(PID_UPDATE_RATE);
            _pitchRollRateController.init(PID_UPDATE_RATE);
            _yawAngleController.init(PID_UPDATE_RATE);
            _yawRateController.init(PID_UPDATE_RATE);
            _positionController.init(PID_UPDATE_RATE);

        }

        void step(
                const bool inHoverMode,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                float motorvals[])
        {
            // Start with open-loop demands
            demands_t demands = {
                openLoopDemands.thrust,
                openLoopDemands.roll,
                openLoopDemands.pitch,
                openLoopDemands.yaw,
            };

            _altitudeController.run(inHoverMode, vehicleState, demands); 

            if (inHoverMode) {

                _climbRateController.run(vehicleState, demands); 

                // Position controller converts meters per second to
                // degrees
                _positionController.run(vehicleState, demands); 

                // Scale up thrust demand for motors
                demands.thrust = Num::fconstrain(
                        demands.thrust * THRUST_SCALE + THRUST_BASE,
                        THRUST_MIN, THRUST_MAX);
            }

            else {

                // In non-hover mode, thrust demand comes in as [0,1], so we
                // scale it up for motors
                demands.thrust *= THRUST_MAX;

                // In non-hover mode, pitch/roll demands come in as
                // [-1,+1], which we convert to degrees for input to
                // pitch/roll controller
                demands.roll *= 30;
                demands.pitch *= 30;
            }

            _pitchRollAngleController.run(vehicleState, demands);

            _pitchRollRateController.run(vehicleState, demands);

            _yawAngleController.run(vehicleState, demands);

            _yawRateController.run(vehicleState, demands);

            // Reset closed-loop controllers on zero thrust
            if (demands.thrust == 0) {
                resetControllers();
            }

            // Scale yaw, pitch and roll demands for mixer
            demands.yaw *= YAW_SCALE;
            demands.roll *= PITCH_ROLL_SCALE;
            demands.pitch *= PITCH_ROLL_SCALE;

            // Run mixer
            uint8_t count = 0;
            _mixFun(demands, motorvals, count);

            //void report(const float thrust);
            //report(demands.thrust);
        }

        void resetControllers(void)
        {        
            _pitchRollAngleController.resetPids();
            _pitchRollRateController.resetPids();
            _positionController.resetPids();
        }

    private:

        mixFun_t _mixFun;

        AltitudeController _altitudeController;
        ClimbRateController _climbRateController;
        PitchRollAngleController _pitchRollAngleController;
        PitchRollRateController _pitchRollRateController;
        PositionController _positionController;
        YawAngleController _yawAngleController;
        YawRateController _yawRateController;
};
