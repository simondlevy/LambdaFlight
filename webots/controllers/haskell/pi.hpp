#pragma once

#include "num.hpp"
#include "lpf.hpp"

class Pi {

    private:


        float _integ;        // integral
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
            _integ         = 0;
            _kp            = kp;
            _ki            = ki;
            _dt            = dt;
        }

        float run(const float desired, const float measured)
        {
            static const float INTEGRATION_LIMIT = 5000;

            auto error = desired - measured;

            auto output = _kp * error;

            _integ += error * _dt;

            _integ = Num::fconstrain(_integ, -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

            output += _ki * _integ;

            return output;
        }

}; // class Pi
