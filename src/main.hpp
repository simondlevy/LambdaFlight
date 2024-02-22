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

        void getDemands(demands_t & demands)
        {
            // Start with open-loop demands
            extern demands_t openLoopDemands;
            memcpy(&demands, &openLoopDemands, sizeof(demands_t));

            // Run PID controllers

            extern bool inHoverMode;

            extern vehicleState_t vehicleState;

            extern bool resetPids;

            _altitudeController.run(inHoverMode, vehicleState, demands); 

            _climbRateController.run(
                    inHoverMode,
                    THRUST_BASE,
                    THRUST_SCALE,
                    THRUST_MIN, 
                    THRUST_MAX,
                    vehicleState, 
                    demands);

            // Reset closed-loop controllers on zero thrust
            const auto reset = resetPids | (demands.thrust == 0);

            _positionController.run(inHoverMode, reset, vehicleState, demands); 

            _pitchRollAngleController.run(reset, vehicleState, demands);

            _pitchRollRateController.run(reset, vehicleState, demands);

            _yawAngleController.run(vehicleState, demands);

            _yawRateController.run(vehicleState, demands);

            // Scale yaw, pitch and roll demands for mixer
            demands.yaw *= YAW_SCALE;
            demands.roll *= PITCH_ROLL_SCALE;
            demands.pitch *= PITCH_ROLL_SCALE;
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
