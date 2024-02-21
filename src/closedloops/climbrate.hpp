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
        virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            demands.thrust = _pid.run(demands.thrust, state.dz);
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        Pid _pid;
};
