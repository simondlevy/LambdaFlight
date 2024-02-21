#pragma once

#include <pid.hpp>
#include <closedloop.hpp>

class ClimbRateController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float kp=25,
                const float ki=15)
        {
            ClosedLoopController::init(updateRate);

            _pid.init(kp, ki, 0, 0, _dt, _updateRate);
        }

        /**
         * Demand is input as climb rate in meters per second and output as
         * arbitrary positive value to be scaled according to motor
         * characteristics.
         */
        void run(
                const bool hover,
                const float base,
                const float scale,
                const float minval,
                const float maxval,
                const vehicleState_t & state, 
                demands_t & demands)
        {
            demands.thrust = 
                hover ? 
                _run (demands.thrust, state.dz, base, scale, minval, maxval) : 
                demands.thrust * maxval;
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        Pid _pid;

        float _run(
                const float thrust, 
                const float dz,
                const float base, 
                const float scale, 
                const float minval, 
                const float maxval)
        {
            return Num::fconstrain(
                    _pid.run(thrust, dz) * scale + base, minval, maxval);
        }
};
