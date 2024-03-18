/*
   Based on https://github.com/nickrehm/dRehmFlight
*/

void new_controlANGLE() 
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

    /*
    extern float roll_des;
    extern float roll_IMU;
    extern float dt;

    static float _integral_roll_prev;

    //Roll
    auto error_roll = roll_des - roll_IMU;
    auto integral_roll = _integral_roll_prev + error_roll * dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll = 0;
    }

    //Saturate integrator to prevent unsafe buildup
    integral_roll = constrain(integral_roll, -i_limit, i_limit); 

    auto derivative_roll = GyroX;

    //Scaled by .01 to bring within -1 to 1 range
    roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - 
            Kd_roll_angle*derivative_roll); 

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch*dt;

    //Don't let integrator build if throttle is too low
    if (channel_1_pwm < 1060) {   
        integral_pitch = 0;
    }

    //Saturate integrator to prevent unsafe buildup
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit); 

    derivative_pitch = GyroY;

    //Scaled by .01 to bring within -1 to 1 range
    pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - 
            Kd_pitch_angle*derivative_pitch); 

    //Yaw, stablize on rate from GyroZ
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;

    //Don't let integrator build if throttle is too low
    if (channel_1_pwm < 1060) {   
        integral_yaw = 0;
    }

    //Saturate integrator to prevent unsafe buildup
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit); 

    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 

    //Scaled by .01 to bring within -1 to 1 range
    yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); 

    //Update roll variables
    _integral_roll_prev = integral_roll;

    //Update pitch variables
    integral_pitch_prev = integral_pitch;

    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
    */
}
