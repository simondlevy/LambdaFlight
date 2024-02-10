/**
 * Miniflie class for real and simulated flight controllers.
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

#include <closedloops/pitchroll_angle.hpp>
#include <closedloops/pitchroll_rate.hpp>
#include <closedloops/position.hpp>
#include <closedloops/yaw_angle.hpp>
#include <closedloops/yaw_rate.hpp>

#include "tude.hpp"

class Miniflie {

    public:

        void init(
                const Clock::rate_t pidUpdateRate,
                const float pitchRollScale,
                const float yawScale)
        {

            _pitchRollScale = pitchRollScale;
            _yawScale = yawScale;

            initClosedLoopControllers(pidUpdateRate);
        }

        void step(
                const bool inHoverMode,
                const vehicleState_t & vehicleState,
                demands_t & demands)
        {
            demands.thrust = runAltitudeController(inHoverMode,
                    vehicleState.z, vehicleState.dz, demands.thrust); 

            if (inHoverMode) {

                // Position controller converts meters per second to
                // degrees
                _positionController.run(vehicleState, demands); 

            }

            else {

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

                demands.roll = 0;
                demands.pitch = 0;
                demands.yaw = 0;

                resetControllers();
            }

            // Scale yaw, pitch and roll demands for mixer
            demands.yaw *= _yawScale;
            demands.roll *= _pitchRollScale;
            demands.pitch *= _pitchRollScale;
        }

        void resetControllers(void)
        {
            _pitchRollAngleController.resetPids();
            _pitchRollRateController.resetPids();
            _positionController.resetPids();

            _positionController.resetFilters();
        }

        static void gyroToVehicleState(
                const Axis3f & gyro, vehicleState_t & vehicleState)
        {
            vehicleState.dphi =    gyro.x;     
            vehicleState.dtheta = -gyro.y; // (negate for ENU)
            vehicleState.dpsi =    gyro.z; 
        }

    private:

        float _pitchRollScale;
        float _yawScale;

        PitchRollAngleController _pitchRollAngleController;
        PitchRollRateController _pitchRollRateController;
        PositionController _positionController;
        YawAngleController _yawAngleController;
        YawRateController _yawRateController;

        void initClosedLoopControllers(const Clock::rate_t pidUpdateRate) 
        {
            _pitchRollAngleController.init(pidUpdateRate);
            _pitchRollRateController.init(pidUpdateRate);
            _yawAngleController.init(pidUpdateRate);
            _yawRateController.init(pidUpdateRate);
            _positionController.init(pidUpdateRate);
        }
};
