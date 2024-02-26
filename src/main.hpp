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

#include <closedloops/position.hpp>

#include <constants.h>

class Miniflie {

    public:

        void init(const mixFun_t mixFun)
        {
            _mixFun = mixFun;

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

            // Reset closed-loop controllers on zero thrust
            extern bool resetPids;

            const auto reset = resetPids || (demands.thrust == 0);

            // const auto dt = 1. / PID_UPDATE_RATE;

            _positionController.run(inHoverMode, reset, vehicleState, demands); 

            //extern demands_t tmpDemands;
            //memcpy(&tmpDemands, &demands, sizeof(demands_t));

            //void reportCpp(float value);
            //reportCpp(demands.pitch);
        }

    private:

        mixFun_t _mixFun;

        PositionController _positionController;
};
