#pragma once

#include <pid.hpp>
#include <closedloop.hpp>

class AltitudeController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float kp=2,
                const float ki=0.5)
        {
            ClosedLoopController::init(updateRate);

            _pid.init(kp, ki, 0, 0, _dt, _updateRate);
        }

        /**
         * Demand is input as altitude target in meters and output as 
         * climb rate in meters per second.
         */
        void run(const vehicleState_t & state, demands_t & demands)
        {
            // Set climb rate based on target altitude
            demands.thrust = _pid.run(demands.thrust, state.z);
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        Pid _pid;
};
