/*
   Based on https://github.com/nickrehm/dRehmFlight
*/

#include <stdint.h>

#include "utils.hpp"

void copilot_proxy_step_core(void) 
{
    // Constants --------------------------------------------------------------

    static const float Kp_cyclic = 0.2;  
    static const float Ki_cyclic = 0.3;
    static const float Kd_cyclic = 0.05;  

    static const float Kp_yaw = 0.3;       
    static const float Ki_yaw = 0.05;      
    static const float Kd_yaw = 0.00015;

    //Integrator saturation level, mostly for safety (default 25.0)
    static const float i_limit = 25.0;     

    // Streams ----------------------------------------------------------------

    extern float thro_des;
    extern float roll_des;
    extern float pitch_des;
    extern float yaw_des;

    extern float roll_IMU;
    extern float pitch_IMU;

    extern float dt;

    extern bool throttle_is_down;

    extern float GyroX;
    extern float GyroY;
    extern float GyroZ;

    // State variables --------------------------------------------------------

    static float _integral_roll_prev;
    static float _integral_pitch_prev;
    static float _integral_yaw_prev;
    static float _error_yaw_prev;

    // Roll ------------------------------------------------------------------

    const auto error_roll = roll_des - roll_IMU;

    // Don't let integrator build if throttle is too low
    const auto integral_roll = throttle_is_down ? 0 :

        //Saturate integrator to prevent unsafe buildup
        constrain(_integral_roll_prev + error_roll * dt, -i_limit, i_limit);


    const auto derivative_roll = GyroX;

    // Scaled by .01 to bring within -1 to 1 range
    const auto roll_PID = 0.01*(Kp_cyclic*error_roll + 
            Ki_cyclic * integral_roll - 
            Kd_cyclic * derivative_roll); 

    // Pitch -----------------------------------------------------------------

    auto error_pitch = pitch_des - pitch_IMU;

    //Don't let integrator build if throttle is too low
    const auto integral_pitch = throttle_is_down ? 0 :

        //Saturate integrator to prevent unsafe buildup
        constrain(_integral_pitch_prev + error_pitch * dt, -i_limit, i_limit);


    const auto derivative_pitch = GyroY;

    //Scaled by .01 to bring within -1 to 1 range
    const auto pitch_PID = .01 * (Kp_cyclic * error_pitch + 
            Ki_cyclic * integral_pitch - 
            Kd_cyclic*derivative_pitch); 

    // Yaw, stablize on rate from GyroZ --------------------------------------

    const auto error_yaw = yaw_des - GyroZ;

    // Don't let integrator build if throttle is too low
    const auto integral_yaw = throttle_is_down ? 0 :

        // Saturate integrator to prevent unsafe buildup
        constrain(_integral_yaw_prev + error_yaw * dt, -i_limit, i_limit);

    const auto derivative_yaw = (error_yaw - _error_yaw_prev)/dt; 

    // Scaled by .01 to bring within -1 to 1 range
    const auto yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); 

    const auto m1 = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
    const auto m2 = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
    const auto m3 = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
    const auto m4 = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left

    void setMotorsProxy(
            const float m1, const float m2, const float m3, const float m4);
    setMotorsProxy(m1, m2, m3, m4);

    // Update state variables
    _integral_roll_prev = integral_roll;
    _integral_pitch_prev = integral_pitch;
    _error_yaw_prev = error_yaw;
    _integral_yaw_prev = integral_yaw;
}
