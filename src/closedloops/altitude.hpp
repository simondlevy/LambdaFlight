#pragma once

#include <num.hpp>

class AltitudeController {

    public:

        /**
         * Demand is input as altitude target in meters and output as 
         * climb rate in meters per second.
         */
        void run(
                const bool hover, 
                const float dt,
                const vehicleState_t & state, 
                demands_t & demands)
        {
            auto thrustraw = demands.thrust;

            // In hover mode, thrust demand comes in as [-1,+1], so
            // we convert it to a target altitude in meters
            auto target = Num::rescale(thrustraw, -1, +1, 0.2, 2.0);

            demands.thrust = hover ? run(dt, target, state.z) : thrustraw;
        }

    private:

        static float run(
                const float dt, 
                const float target, 
                const float z)
        {
            const float kp = 2;
            const float ki = 0.5;
            const float ilimit = 5000;

            static float _integ;

            auto error = target - z;

            _integ += error * dt;

            _integ = Num::fconstrain(_integ, -ilimit, +ilimit);

            return kp * error + ki * _integ;
        }
};
