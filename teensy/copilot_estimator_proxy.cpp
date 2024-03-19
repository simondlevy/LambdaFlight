#include <stdint.h>
#include <math.h>

#include "utils.hpp"

static float invSqrt(float x) 
{
    return 1.0/sqrtf(x); 
}

static float q0 = 1.0f; //Initialize quaternion for madgwick filter
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static void Madgwick6DOF(
        const float gx, 
        const float gy, 
        const float gz, 
        const float ax, 
        const float ay, 
        const float az, 
        const float invSampleFreq) 
{
    static const float B_madgwick = 0.04;  
     
    //Convert gyroscope degrees/sec to radians/sec
    const auto ggx = gx * 0.0174533f;
    const auto ggy = gy * 0.0174533f;
    const auto ggz = gz * 0.0174533f;

    //Rate of change of quaternion from gyroscope
    auto qDot1 = 0.5f * (-q1 * ggx - q2 * ggy - q3 * ggz);
    auto qDot2 = 0.5f * (q0 * ggx + q2 * ggz - q3 * ggy);
    auto qDot3 = 0.5f * (q0 * ggy - q1 * ggz + q3 * ggx);
    auto qDot4 = 0.5f * (q0 * ggz + q1 * ggy - q2 * ggx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in
    //accelerometer normalisation)
    if (!((ax == 0) && (ay == 0) && (az == 0))) {

        //Normalise accelerometer measurement
        const auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        auto aax = ax * recipNorm;
        auto aay = ay * recipNorm;
        auto aaz = az * recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        const auto _2q0 = 2 * q0;
        const auto _2q1 = 2 * q1;
        const auto _2q2 = 2 * q2;
        const auto _2q3 = 2 * q3;
        const auto _4q0 = 4 * q0;
        const auto _4q1 = 4 * q1;
        const auto _4q2 = 4 * q2;
        const auto _8q1 = 8 * q1;
        const auto _8q2 = 8 * q2;
        const auto q0q0 = q0 * q0;
        const auto q1q1 = q1 * q1;
        const auto q2q2 = q2 * q2;
        const auto q3q3 = q3 * q3;

        //Gradient decent algorithm corrective step
        const auto s0 = _4q0 * q2q2 + _2q2 * aax + _4q0 * q1q1 - _2q1 * aay;
        const auto s1 = _4q1 * q3q3 - _2q3 * aax + 4.0f * q0q0 * q1 - _2q0 * aay - _4q1 +
            _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * aaz; 
        const auto s2 = 4.0f * q0q0 * q2 + _2q0
            * aax + _4q2 * q3q3 - _2q3 * aay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
            _4q2 * aaz;
        const auto s3 = 4.0f * q1q1 * q3 - _2q1 * aax + 4.0f * q2q2 * q3 - _2q2 * aay;
        const auto rrecipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 

        //Apply feedback step
        qDot1 -= B_madgwick * s0 * rrecipNorm;
        qDot2 -= B_madgwick * s1 * rrecipNorm;
        qDot3 -= B_madgwick * s2 * rrecipNorm;
        qDot4 -= B_madgwick * s3 * rrecipNorm;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    const auto recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //Compute angles in degrees
    const auto phi = 
        atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; 
    const auto theta = 
        -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951;
    const auto psi = 
        -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951;

    void setState(const float phi, const float theta, const float psi);
    setState(phi, theta, psi);
}

void copilot_step_estimator(void) 
{
    // Streams
    extern float GyroX;
    extern float GyroY;
    extern float GyroZ;
    extern float AccX;
    extern float AccY;
    extern float AccZ;
    extern float dt;

    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); 
}
