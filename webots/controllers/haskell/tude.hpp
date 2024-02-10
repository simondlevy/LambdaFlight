#pragma once

#include "num.hpp"

class Pi {

    private:


        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _dt;           // delta-time dt

    public:

        void init(
                const float kp,
                const float ki,
                const float dt,
                const float samplingRate)
        {
            _kp    = kp;
            _ki    = ki;
            _dt    = dt;
        }

        float run(const float desired, const float measured)
        {
            static const float INTEGRATION_LIMIT = 5000;

            static float _integ;     

            auto error = desired - measured;

            _integ = Num::fconstrain(_integ + error * _dt, 
                    -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

            return _kp * error + _ki * _integ;
        }

}; // class Pi

float constrain(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

class AltitudeController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float altitudeKp=2,
                const float altitudeKi=0.5,
                const float climbRateKp=25,
                const float climbRateKi=15)
        {
            ClosedLoopController::init(updateRate);

            _altitudePi.init(altitudeKp, altitudeKi, _dt, _updateRate);
            _climbRatePi.init(climbRateKp, climbRateKi, _dt, _updateRate);
        }

        void run(const vehicleState_t & state, demands_t & demands)
        {
            // Set climb rate based on target altitude
            auto climbRate = _altitudePi.run(demands.thrust, state.z);

            // Set thrust for desired climb rate
            demands.thrust = _climbRatePi.run(climbRate, state.dz);
        }

    private:

        Pi _altitudePi;
        Pi _climbRatePi;
};
