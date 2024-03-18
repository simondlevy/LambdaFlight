/*
   Based on https://github.com/nickrehm/dRehmFlight
*/

#include <stdint.h>

static float constrain(const float val, const float minval, const float maxval)
{
    return val < minval ? minval : val > maxval ? maxval : val;
}

void controlANGLE() 
{
    //DESCRIPTION: Computes control commands based on state error (angle)
    /*
     * Basic PID control to stablize on angle setpoint based on desired states
     * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is
     * simply the desired state minus the actual state (ex. roll_des -
     * roll_IMU). Two safety features are implimented here regarding the I
     * terms. The I terms are saturated within specified limits on startup to
     * prevent excessive buildup. This can be seen by holding the vehicle at an
     * angle and seeing the motors ramp up on one side until they've maxed out
     * throttle...saturating I to a specified limit fixes this. The second
     * feature defaults the I terms to 0 if the throttle is at the minimum
     * setting. This means the motors will not start spooling up on the ground,
     * and the I terms will always start from 0 on takeoff. This function
     * updates the variables roll_PID, pitch_PID, and yaw_PID which can be
     * thought of as 1-D stablized signals. They are mixed to the configuration
     * of the vehicle in controlMixer().
     */

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

    auto error_roll = roll_des - roll_IMU;

    // Don't let integrator build if throttle is too low
    auto integral_roll = throttle_is_down ? 0 :

        //Saturate integrator to prevent unsafe buildup
        constrain(_integral_roll_prev + error_roll * dt, -i_limit, i_limit);


    auto derivative_roll = GyroX;

    // Scaled by .01 to bring within -1 to 1 range
    auto roll_PID = 0.01*(Kp_cyclic*error_roll + 
            Ki_cyclic * integral_roll - 
            Kd_cyclic * derivative_roll); 

    // Pitch -----------------------------------------------------------------

    auto error_pitch = pitch_des - pitch_IMU;

    //Don't let integrator build if throttle is too low
    auto integral_pitch = throttle_is_down ? 0 :

        //Saturate integrator to prevent unsafe buildup
        constrain(_integral_pitch_prev + error_pitch * dt, -i_limit, i_limit);


    auto derivative_pitch = GyroY;

    //Scaled by .01 to bring within -1 to 1 range
    auto pitch_PID = .01 * (Kp_cyclic * error_pitch + 
            Ki_cyclic * integral_pitch - 
            Kd_cyclic*derivative_pitch); 

    // Yaw, stablize on rate from GyroZ --------------------------------------

    auto error_yaw = yaw_des - GyroZ;

    // Don't let integrator build if throttle is too low
    auto integral_yaw = throttle_is_down ? 0 :

        // Saturate integrator to prevent unsafe buildup
        constrain(_integral_yaw_prev + error_yaw * dt, -i_limit, i_limit);

    auto derivative_yaw = (error_yaw - _error_yaw_prev)/dt; 

    // Scaled by .01 to bring within -1 to 1 range
    auto yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); 

    // Update state variables
    _integral_roll_prev = integral_roll;
    _integral_pitch_prev = integral_pitch;
    _error_yaw_prev = error_yaw;
    _integral_yaw_prev = integral_yaw;

    void setPids(const float roll, const float pitch, const float yaw);
    setPids(roll_PID, pitch_PID, yaw_PID);
}
