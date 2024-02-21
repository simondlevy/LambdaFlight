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
        void run(
                const bool hover, 
                const vehicleState_t & state, 
                demands_t & demands)
        {

            auto thrustraw = demands.thrust;

            demands.thrust = hover ? run(thrustraw, state.z) : thrustraw;
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        float run(const float thrustraw, const float z)
        {
            // In hover mode, thrust demand comes in as [-1,+1], so
            // we convert it to a target altitude in meters
            auto target = Num::rescale(thrustraw, -1, +1, 0.2, 2.0);

            return _pid.run(target, z);
        }

        Pid _pid;
};
