#pragma once

#include <math3d.h>

#include <num.hpp>
#include <lpf.hpp>

class Pid {

    private:

        static constexpr float DEFAULT_PID_INTEGRATION_LIMIT = 5000;
        static constexpr float DEFAULT_PID_OUTPUT_LIMIT =  0;

        float _integ;        // integral
        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _outP;         // proportional output (debugging)
        float _outI;         // integral output (debugging)
        float _outD;         // derivative output (debugging)
        float _outFF;        // feedforward output (debugging)
        float _iLimit;       // integral limit, absolute value. '0' means no limit.
        float _outputLimit;  // total PID output limit, absolute value. '0' means no limit.
        float _dt;           // delta-time dt
        Lpf _dFilter;        // filter for D term
        bool _enableDFilter; // filter for D term enable flag

    public:

        void init(const float kp, const float ki)
        {
            _kp = kp;
            _ki = ki;
            _dt = 0.01;
        }

        float run(const float desired, const float measured)
        {
            auto error = desired - measured;

            static float _integ;

            _integ += error * _dt;

            return _kp * error + _ki * _integ;
        }


}; // class Pid

class PositionController {

    public:

        void init(const float kp=25, const float ki=1)
        {
            initAxis(_pidX, kp, ki);
            initAxis(_pidY, kp, ki);
        }

        /**
          * Demands are input as normalized interval [-1,+1] and output as
          * angles in degrees.
          *
          * roll:  input left positive => output negative
          *
          * pitch: input forward positive => output negative
          */
        void run(const vehicleState_t & state, demands_t & demands) 
        {
            // Rotate world-coordinate velocities into body coordinates
            const auto dxw = state.dx;
            const auto dyw = state.dy;
            const auto psi = deg2rad(state.psi);
            const auto cospsi = cos(psi);
            const auto sinpsi = sin(psi);
            const auto dxb =  dxw * cospsi + dyw * sinpsi;
            const auto dyb = -dxw * sinpsi + dyw * cospsi;       

            // Run PID closedloops on body-coordinate velocities
            demands.roll = runAxis(demands.roll, dyb, _pidY);
            demands.pitch = runAxis(demands.pitch, dxb, _pidX);
        }

    private:

        static constexpr float LIMIT = 20;
        static constexpr float LIMIT_OVERHEAD = 1.10;
        static constexpr float FILTER_CUTOFF = 20;

        static float deg2rad(float degrees) 
        { 
            return (M_PI / 180.0f) * degrees; 
        }

        Pid _pidX;
        Pid _pidY;

        void initAxis(Pid & pid, const float kp, const float ki)
        {
            pid.init(kp, ki);
        }

        float runAxis(const float demand, const float measured, Pid & pid)
        {
            // note negation
            return Num::fconstrain(-pid.run(demand, measured), -LIMIT, +LIMIT);
        }
};
