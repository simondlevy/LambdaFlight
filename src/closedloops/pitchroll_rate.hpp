/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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

#include <num.hpp>

class NewPid {

    private:

        float _prevError;    // previous error
        float _integ;        // integral

    public:

        void init(void)
        {
            _prevError = 0;
            _integ     = 0;
        }

        void reset(void)
        {
            _prevError = 0;
            _integ     = 0;
        }

        float run(
                const float kp,
                const float ki,
                const float kd,
                const float ilimit,
                const float dt,
                const float desired, 
                const float measured)
        {
            auto error = desired - measured;

            auto deriv = (error - _prevError) / dt;

            _integ = Num::fconstrain(_integ + error * dt, -ilimit, ilimit);

            _prevError = error;

            return kp * error + ki * _integ + kd * deriv;
        }
}; 

/////////////////////////////////////////////////////////////////////////////

class PitchRollRateController {

    public:

        void init(void)
        {
            _rollPid.init();
            _pitchPid.init();
        }

        void run(
                const bool reset, 
                const float dt,
                const vehicleState_t & state, 
                demands_t & demands)
        {
            const float kp = 125;
            const float ki = 250;
            const float kd = 1.25;
            const float ilimit = 33;

            if (reset) {
                _rollPid.reset();
                _pitchPid.reset();
            }

            demands.roll = demands.thrust == 0 ? 0 :
                _rollPid.run(kp, ki, kd, ilimit, dt, demands.roll, state.dphi);

            demands.pitch = demands.thrust == 0 ? 0 :
                _pitchPid.run(kp, ki, kd, ilimit, dt, demands.pitch, state.dtheta);
        }

    private:

        static constexpr float INTEGRAL_LIMIT = 33;

        NewPid _rollPid;
        NewPid _pitchPid;

};
