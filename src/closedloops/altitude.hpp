#pragma once

#include <pid.hpp>
#include <closedloop.hpp>

class AltitudeController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float altitudeKp=2,
                const float altitueKi=0.5,
                const float climbRateKp=25,
                const float climbRateKi=15)
        {
            ClosedLoopController::init(updateRate);

            _altitudePid.init(altitudeKp, altitueKi, 0, 0, _dt, _updateRate);

            _climbRatePid.init(climbRateKp, climbRateKi, 0, 0, _dt, _updateRate);
        }

        /**
         * Demand is input as altitude target in meters and output as 
         * arbitrary positive value to be scaled according to motor
         * characteristics.
         */
        virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            // Set climb rate based on target altitude
            auto climbRate = _altitudePid.run(demands.thrust, state.z);

            // Set thrust for desired climb rate
            demands.thrust = _climbRatePid.run(climbRate, state.dz);
        }

        void resetPids(void)
        {
            _altitudePid.reset();
            _climbRatePid.reset();
        }

    private:

        static constexpr float VEL_MAX = 1;
        static constexpr float VEL_MAX_OVERHEAD = 1.10;

        Pid _altitudePid;
        Pid _climbRatePid;
};
