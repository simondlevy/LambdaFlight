/*
   Based on https://github.com/nickrehm/dRehmFlight
 */


#include <Wire.h> 

#include <sbus.h>

#define _MAIN
#include <teensy_streams.h>

#include <usfs.hpp>
#include <oneshot125.hpp>

#include <vector>

// LED settings -------------------------------------------------------------

static const uint8_t LED_PIN = 13;
static const uint32_t NUM_BLINKS = 3;
static const uint32_t BLINK_UPTIME = 160;
static const uint32_t BLINK_DOWNTIME = 70;

// USFS settings ------------------------------------------------------------

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
Usfs::INTERRUPT_ERROR |
Usfs::INTERRUPT_QUAT;

static const bool VERBOSE = false;

// VL53L1 settings -----------------------------------------------------------

static const uint32_t RANGEFINDER_FREQ = 40;

// ---------------------------------------------------------------------------

// Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
static const uint32_t LOOP_RATE = 2000;

static const std::vector<uint8_t> MOTOR_PINS = {2, 3, 0, 1};

static auto motors = OneShot125(MOTOR_PINS);

bfs::SbusRx sbus(&Serial2);

// Motors set by Haskell
static int m1_command;
static int m2_command;
static int m3_command;
static int m4_command;

static float _phi;
static float _theta;
static float _psi;

// ---------------------------------------------------------------------------

static const uint8_t REPORT_HZ = 2;

static Usfs usfs;

static void powerPin(const uint8_t pin, const bool hilo)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, hilo);
}

static void initLed(void)
{
    pinMode(LED_PIN, OUTPUT); 
    digitalWrite(LED_PIN, HIGH);

    for (uint32_t j = 1; j<= NUM_BLINKS; j++) {
        digitalWrite(LED_PIN, LOW);
        delay(BLINK_DOWNTIME);
        digitalWrite(LED_PIN, HIGH);
        delay(BLINK_UPTIME);
    }
}

static void blinkLed(const uint32_t current_time) 
{
    //DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
     * It looks cool.
     */
    static uint32_t blink_counter;
    static uint32_t blink_delay;
    static bool blinkAlternate;

    if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(LED_PIN, blinkAlternate); //Pin LED_PIN is built in LED

        if (blinkAlternate == 1) {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0) {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

static void initImu(void)
{
    powerPin(21, HIGH);
    powerPin(22, LOW);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(100);

    Wire1.begin(); 
    Wire1.setClock(400000);
    delay(100);

    usfs.loadFirmware(VERBOSE); 

    usfs.begin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE,
            VERBOSE); 

    // Clear interrupts
    Usfs::checkStatus();
}

static void readImu(void)
{
    const auto eventStatus = Usfs::checkStatus(); 

    if (Usfs::eventStatusIsError(eventStatus)) { 

        Usfs::reportError(eventStatus);
    }

    if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

        usfs.readAccelerometerScaled(
                stream_accel_x, stream_accel_y, stream_accel_z);
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        usfs.readGyrometerScaled(
                stream_gyro_x, stream_gyro_y, stream_gyro_z);
    }
}

static void readReceiver() 
{
    if (sbus.Read()) {

        stream_channel1_raw = sbus.data().ch[0];
        stream_channel2_raw = sbus.data().ch[1];
        stream_channel3_raw = sbus.data().ch[2];
        stream_channel4_raw = sbus.data().ch[3];
        stream_channel5_raw = sbus.data().ch[4];
    }

    if (sbus.data().failsafe) {
        stream_radio_failsafe = true;
    }
}

static uint32_t updateTime(void)
{
    static uint32_t _current_time;
    static uint32_t _prev_time;

    _prev_time = _current_time;      
    _current_time = micros();      
    stream_dt = (_current_time - _prev_time)/1e6;

    return _current_time;
}

static void runMotors() 
{
    motors.set(0, m1_command);
    motors.set(1, m2_command);
    motors.set(2, m3_command);
    motors.set(3, m4_command);

    motors.run();
}

static void maintainLoopRate(const uint32_t current_time) 
{
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain
     * stable and whatnot. Interrupt routines running in the background cause
     * the loop rate to fluctuate. This function basically just waits at the
     * end of every loop iteration until the correct time has passed since the
     * start of the current loop for the desired loop rate in Hz. 2kHz is a
     * good rate to be at because the loop nominally will run between 2.8kHz -
     * 4.2kHz. This lets us have a little room to add extra computations and
     * remain above 2kHz, without needing to retune all of our filtering
     * parameters.
     */
    float invFreq = 1.0/LOOP_RATE*1000000.0;
    auto checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}



static void debug(const uint32_t current_time)
{

    static uint32_t previous_time;

    if (current_time - previous_time > 10000) {

        previous_time = current_time;

        //debugAccel();  
        //debugGyro();  
        //debugQuat();  
        debugState();  
        //debugMotorCommands(); 
        //debugLoopRate();      
        //debugRangefinder();      
    }
}

static float invSqrt(const float x)
{
    return 1 / sqrt(x);
}

static void runMadgwick(const bool init)
{
    static float _q0;
    static float _q1;
    static float _q2;
    static float _q3;

    _q0 = init ? 1 : _q0;

    // Tunable parameter
    const auto b_madgwick = 0.04f;

    // Convert gyroscope degrees/sec to radians/sec
    const auto ggx = stream_gyro_x * 0.0174533;
    const auto ggy = stream_gyro_y * 0.0174533;
    const auto ggz = stream_gyro_z * 0.0174533;
    
    const auto ax = stream_accel_x;
    const auto ay = stream_accel_y;
    const auto az = stream_accel_z;

    // Normalize accelerometer measurement
    const auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    const auto aax = ax * recipNorm;
    const auto aay = ay * recipNorm;
    const auto aaz = az * recipNorm;

    // Rate of change of quaternion from gyroscope
    const auto qDot1 = 0.5 * (-_q1 * ggx - _q2 * ggy - _q3 * ggz);
    const auto qDot2 = 0.5 * (_q0 * ggx + _q2 * ggz - _q3 * ggy);
    const auto qDot3 = 0.5 * (_q0 * ggy - _q1 * ggz + _q3 * ggx);
    const auto qDot4 = 0.5 * (_q0 * ggz + _q1 * ggy - _q2 * ggx);

    //Auxiliary variables to avoid repeated arithmetic
    const auto _2q0 = 2 * _q0;
    const auto _2q1 = 2 * _q1;
    const auto _2q2 = 2 * _q2;
    const auto _2q3 = 2 * _q3;
    const auto _4q0 = 4 * _q0;
    const auto _4q1 = 4 * _q1;
    const auto _4q2 = 4 * _q2;
    const auto _8q1 = 8 * _q1;
    const auto _8q2 = 8 * _q2;
    const auto q0q0 = _q0 * _q0;
    const auto q1q1 = _q1 * _q1;
    const auto q2q2 = _q2 * _q2;
    const auto q3q3 = _q3 * _q3;

    // Gradient decent algorithm corrective step
    const auto s0 = _4q0 * q2q2 + _2q2 * aax + _4q0 * q1q1 - _2q1 * aay;
    const auto s1 = _4q1 * q3q3 - _2q3 * aax + 4 * q0q0 * _q1 - _2q0 * aay - _4q1 +
        _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * aaz;
    const auto s2 = 4 * q0q0 * _q2 + _2q0 * aax + _4q2 * q3q3 - _2q3 * aay - 
        _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * aaz;
    const auto s3 = 4 * q1q1 * _q3 - _2q1 * aax + 4 * q2q2 * _q3 - _2q2 * aay;

    const auto recipNorm1 = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

    const auto isAccelOkay = !(ax == 0 && ay == 0 && az == 0);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    const auto qqDot1 = qDot1 - (isAccelOkay ? b_madgwick * s0 * recipNorm1 : 0);
    const auto qqDot2 = qDot2 - (isAccelOkay ? b_madgwick * s1 * recipNorm1 : 0);
    const auto qqDot3 = qDot3 - (isAccelOkay ? b_madgwick * s2 * recipNorm1 : 0);
    const auto qqDot4 = qDot4 - (isAccelOkay ? b_madgwick * s3 * recipNorm1 : 0);

    _q0 = _q0 + qqDot1 * stream_dt;
    _q1 = _q1 + qqDot2 * stream_dt;
    _q2 = _q2 + qqDot3 * stream_dt;
    _q3 = _q3 + qqDot4 * stream_dt;

    // Compute angles in degrees
    _phi = atan2(_q0*_q1 + _q2*_q3, (0.5 - _q1*_q1 - _q2*_q2)) * 57.29577951;
    _theta = -asin(constrain(-2 * (_q1*_q3 - _q0*_q2), -0.999999, 0.999999)) * 57.29577951;
    _psi = -atan2(_q1*_q2 + _q0*_q3, (0.5 - _q2*_q2 - _q3*_q3)) * 57.29577951;
}

void setup()
{
    Serial.begin(115200);

    initImu();

    stream_radio_failsafe = false;

    delay(5);

    sbus.Begin();

    initLed();

    delay(5);

    motors.arm();

    runMadgwick(true);

} // setup

void loop()
{
    auto currentTime = updateTime();

    blinkLed(currentTime);

    debug(currentTime);

    readImu();

    runMotors();

    runMadgwick(false);

    readReceiver();

    maintainLoopRate(currentTime); 

}  // loop

// Debugging -----------------------------------------------------------------

void debugAccel(void) 
{
    Serial.printf("accelX:%+3.3f accelY:%+3.3f accelZ:%+3.3f\n", 
            stream_accel_x, stream_accel_y, stream_accel_z);
}

void debugGyro(void) 
{
    Serial.printf("gyroX:%+03.3f gyroY:%+03.3f gyroZ:%+03.3f\n", 
            stream_gyro_x, stream_gyro_y, stream_gyro_z);
}

void debugLoopRate(void) 
{
    Serial.printf("dt:%f\n", stream_dt*1e6);
}

void debugMotorCommands(void) 
{
    Serial.printf(
            "m1_command:%d m2_command:%d m3_command:%d m4_command:%d\n",
            m1_command, m2_command,
            m3_command, m4_command);
}

void debugQuat(void) 
{
    Serial.printf("%+3.3f %+3.3f %+3.3f %+3.3f\n",
            stream_quat_w, stream_quat_x, stream_quat_y, stream_quat_z);
}

void debugRadio(void) 
{
    Serial.printf("ch1:%4.0f ch2:%4.0f ch3:%4.0f ch4:%4.0f ch5:%4.0f\n",
            stream_channel1_raw, stream_channel2_raw, stream_channel3_raw, 
            stream_channel4_raw, stream_channel5_raw);
}

void debugRangefinder(void) 
{
    Serial.printf("dist: %f\n", stream_rangefinder_distance);
}

void debugState(void) 
{
    Serial.printf("roll:%2.2f pitch:%2.2f yaw:%2.2f alt:0\n", 
            _phi, _theta, _psi);
}

