#pragma once

static float constrain(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

static float rescale(
        const float value,
        const float oldmin, 
        const float oldmax, 
        const float newmin, 
        const float newmax) 
{
    return (value - oldmin) / (oldmax - oldmin) * 
        (newmax - newmin) + newmin;
}


static float altitudePid(const float desired, const float measured)
{
    static const float KP = 2;
    static const float KI = 0.5;
    static const float DT = 0.01;

    static const float INTEGRATION_LIMIT = 5000;

    static float _integ;     

    auto error = desired - measured;

    _integ = constrain(_integ + error * DT, 
            -INTEGRATION_LIMIT, INTEGRATION_LIMIT);

    return KP * error + KI * _integ;
}


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

static float runAltitudeController(const float z, const float dz, const float thrust)
{
    // In hover mode, thrust demand comes in as [-1,+1], so
    // we convert it to a target altitude in meters
    const auto sthrust = rescale(thrust, -1, +1, 0.2, 2.0);

    // Set climb rate based on target altitude
    auto climbRate = altitudePid(sthrust, z);

    // Set thrust for desired climb rate
    return climbRatePid(climbRate, dz);
}
