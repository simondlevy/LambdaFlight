#pragma once

#include <closedloop.hpp>
#include <math3d.h>

#include <num.hpp>
#include <lpf.hpp>

class Pid {

    private:

        static constexpr float DEFAULT_PID_INTEGRATION_LIMIT = 5000;
        static constexpr float DEFAULT_PID_OUTPUT_LIMIT =  0;

        float _desired;      // set point
        float _error;        // error
        float _integ;        // integral
        float _deriv;        // derivative
        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _kd;           // derivative gain
        float _kff;          // feedforward gain
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

        void init( const float kp, const float ki)
        {
            _error         = 0;
            _integ         = 0;
            _deriv         = 0;
            _desired       = 0;
            _kp            = kp;
            _ki            = ki;
            _dt            = 0.01;
        }

        void setIntegralLimit(const float limit)
        {
            _iLimit = limit;
        }

        void reset(void)
        {
            _error     = 0;
            _integ     = 0;
            _deriv     = 0;
        }

        float run(const float desired, const float measured)
        {
            _desired = desired;

            _error = _desired - measured;

            return run();
        }

        float run(void)
        {
            _outP = _kp * _error;

            auto output = _outP;

            _integ += _error * _dt;

            // Constrain the integral (unless the iLimit is zero)
            if(_iLimit != 0) {
                _integ = Num::fconstrain(_integ, -_iLimit, _iLimit);
            }

            _outI = _ki * _integ;
            output += _outI;

            // Constrain the total PID output (unless the outputLimit is zero)
            if(_outputLimit != 0) {
                output = Num::fconstrain(output, -_outputLimit, _outputLimit);
            }

            return output;
        }

        void setDesired(const float desired)
        {
            _desired = desired;
        }

        void setError(const float error)
        {
            _error = error;
        }

        void setOutputLimit(const float outputLimit)
        {
            _outputLimit = outputLimit;
        }

}; // class Pid

class PositionController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=25, 
                const float ki=1)
        {
            ClosedLoopController::init(updateRate);

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
         virtual void run(const vehicleState_t & state,
                demands_t & demands) override 
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

        void resetPids(void)
        {
            _pidX.reset();
            _pidY.reset();
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
            pid.setOutputLimit(LIMIT * LIMIT_OVERHEAD);
        }

        float runAxis(const float demand, const float measured, Pid & pid)
        {
            // note negation
            return Num::fconstrain(-pid.run(demand, measured), -LIMIT, +LIMIT);
        }
};
