#pragma once

#include "num.hpp"
#include "lpf.hpp"

class Pi {

    private:

        static constexpr float DEFAULT_PID_INTEGRATION_LIMIT = 5000;
        static constexpr float DEFAULT_PID_OUTPUT_LIMIT =  0;

        float _integ;        // integral
        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _outI;         // integral output (debugging)
        float _iLimit;       // integral limit, absolute value. '0' means no limit.
        float _outputLimit;  // total PID output limit, absolute value. '0' means no limit.
        float _dt;           // delta-time dt
        Lpf _dFilter;        // filter for D term
        bool _enableDFilter; // filter for D term enable flag

    public:

        void init(
                const float kp,
                const float ki,
                const float dt,
                const float samplingRate,
                const float cutoffFreq,
                bool enableDFilter)
        {
            _integ         = 0;
            _kp            = kp;
            _ki            = ki;
            _iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
            _outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
            _dt            = dt;
            _enableDFilter = enableDFilter;

            if (_enableDFilter) {
                _dFilter.init(samplingRate, cutoffFreq);
            }
        }

        float run(const float desired, const float measured)
        {
            auto error = desired - measured;

            auto output = _kp * error;

            _integ += error * _dt;

            if(_iLimit != 0) {
                _integ = Num::fconstrain(_integ, -_iLimit, _iLimit);
            }

            _outI = _ki * _integ;
            output += _outI;

            if(_outputLimit != 0) {
                output = Num::fconstrain(output, -_outputLimit, _outputLimit);
            }

            return output;
        }

}; // class Pi
