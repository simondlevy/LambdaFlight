#pragma once

#include "pi.hpp"

#include <closedloop.hpp>

float max(float a, float b)
{
    return a > b ? a : b;
}

float runAltitudePi(const float thrust, const float z)
{
    static const float kp = 25;
    static const float ki = 15;
    static const float integral_limit = 5000;
    static const float dt = 0.01;
    static const float vel_max = 1;
    static const float vel_max_overhead = 1.10;

    auto error = thrust - z;

    static float _errorIntegral;

    _errorIntegral = max((_errorIntegral + error * dt), integral_limit);

    return kp * error + ki * + _errorIntegral;
}

float runClimbRatePi(const float thrust, const float dz)
{
    static const float kp = 25;
    static const float ki = 15;
    static const float integral_limit = 5000;
    static const float dt = 0.01;

    auto error = thrust - dz;

    static float _errorIntegral;

    _errorIntegral = max((_errorIntegral + error * dt), integral_limit);

    return kp * error + ki * + _errorIntegral;
}


float runAltitudeController(const float z, const float dz, const float thrust)
{
    auto climbRate = runAltitudePi(z, thrust);

    return runClimbRatePi(dz, climbRate);
}

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

            _altitudePi.init(altitudeKp, altitueKi, _dt, _updateRate,
                    FILTER_CUTOFF, true);

            _climbRatePi.init(climbRateKp, climbRateKi, _dt, _updateRate,
                    FILTER_CUTOFF, true); 
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
            auto climbRate = _altitudePi.run(demands.thrust, state.z);

            // Set thrust for desired climb rate
            demands.thrust = _climbRatePi.run(climbRate, state.dz);
        }

    private:

        static constexpr float FILTER_CUTOFF = 20;

        Pi _altitudePi;
        Pi _climbRatePi;
};
