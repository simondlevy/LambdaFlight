#pragma once

#include "num.hpp"

float constrain(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

class AltitudePi {

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

            _integ = constrain(_integ + error * _dt, 
                    -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

            return _kp * error + _ki * _integ;
        }

};

static float climbRatePid(const float desired, const float measured)
{
    static const float KP = 25;
    static const float KI = 15;
    static const float DT = 0.01;

    static const float INTEGRATION_LIMIT = 5000;

    static float _integ;     

    auto error = desired - measured;

    _integ = constrain(_integ + error * DT, 
            -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

    return KP * error + KI * _integ;
}

class AltitudeController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float altitudeKp=2,
                const float altitudeKi=0.5)
        {
            ClosedLoopController::init(updateRate);

            _altitudePi.init(altitudeKp, altitudeKi, _dt, _updateRate);
        }

        void run(const vehicleState_t & state, demands_t & demands)
        {
            // Set climb rate based on target altitude
            auto climbRate = _altitudePi.run(demands.thrust, state.z);

            // Set thrust for desired climb rate
            demands.thrust = climbRatePid(climbRate, state.dz);
        }

    private:

        AltitudePi _altitudePi;
};
