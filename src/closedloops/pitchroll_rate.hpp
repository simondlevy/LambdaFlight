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

#include <closedloop.hpp>
#include <num.hpp>

class NewPid {

    private:

        float _prevError;    // previous error
        float _integ;        // integral
        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _kd;           // derivative gain
        float _iLimit;       // integral limit, absolute value. '0' means no limit.
        float _dt;           // delta-time dt

    public:

        void init(
                const float kp, 
                const float ki, 
                const float kd, 
                const float ilimit,
                const float dt)
        {
            _kp            = kp;
            _ki            = ki;
            _kd            = kd;
            _iLimit        = ilimit;
            _dt            = dt;

            _prevError     = 0;
            _integ         = 0;
        }

        void reset(void)
        {
            _prevError = 0;
            _integ     = 0;
        }

        float run(const float desired, const float measured)
        {
            auto error = desired - measured;

            auto deriv = (error - _prevError) / _dt;

            _integ = Num::fconstrain(_integ + error * _dt, -_iLimit, _iLimit);

            _prevError = error;

            return _kp * error + _ki * _integ + _kd * deriv;
        }
}; 

/////////////////////////////////////////////////////////////////////////////

class PitchRollRateController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=125,
                const float ki=250,
                const float kd=1.25)
        {
            ClosedLoopController::init(updateRate);

            initPid(kp, ki, kd, _rollPid);

            initPid(kp, ki, kd, _pitchPid);
        }

        void run(
                const bool reset, 
                const float dt,
                const vehicleState_t & state, 
                demands_t & demands)
        {
            if (reset) {
                _rollPid.reset();
                _pitchPid.reset();
            }

            demands.roll = demands.thrust == 0 ? 0 :
                _rollPid.run(demands.roll, state.dphi);

            demands.pitch = demands.thrust == 0 ? 0 :
                _pitchPid.run(demands.pitch, state.dtheta);
        }

    private:

        static constexpr float INTEGRAL_LIMIT = 33;

        NewPid _rollPid;
        NewPid _pitchPid;

        void initPid(const float kp, const float ki, const float kd, NewPid  & pid) 
        {
            pid.init(kp,  ki,  kd, INTEGRAL_LIMIT, _dt);
        }
};
