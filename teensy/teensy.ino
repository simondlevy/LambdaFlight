/*
   Based on https://github.com/nickrehm/dRehmFlight
*/

#include <Wire.h> 

#include <sbus.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#include <oneshot125.hpp>
#include <vector>

// Gyro and accel full scale value selection and scale factor
static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;

// Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
static const uint32_t LOOP_RATE = 2000;

// ---------------------------------------------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = {0, 1, 2, 3};

static auto motors = OneShot125(MOTOR_PINS);

static MPU6050 mpu6050;

bfs::SbusRx sbus(&Serial5);

// Streams read by Copilot.hs
float dt;
float channel1_raw;
float channel2_raw;
float channel3_raw;
float channel4_raw;
float channel5_raw;
bool radio_failsafe;
float AcX;
float AcY;
float AcZ;
float GyX;
float GyY;
float GyZ;

// Stream written and read by Copilot.hs
float statePhi, stateTheta, statePsi;

// Motors
static int m1_command_PWM;
static int m2_command_PWM;
static int m3_command_PWM;
static int m4_command_PWM;

static void IMUinit() 
{
    //DESCRIPTION: Initialize IMU
    /*
     * Don't worry about how this works.
     */
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    mpu6050.initialize();

    if (mpu6050.testConnection() == false) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

static void getIMUdata() 
{
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    AcX = ax;
    AcY = ay;
    AcZ = az;
    GyX = gx;
    GyY = gy;
    GyZ = gz;
}

static void getOpenLoopDemands() 
{
    if (sbus.Read()) {

        channel1_raw = sbus.data().ch[0];
        channel2_raw = sbus.data().ch[1];
        channel3_raw = sbus.data().ch[2];
        channel4_raw = sbus.data().ch[3];
        channel5_raw = sbus.data().ch[4];
    }

    if (sbus.data().failsafe) {
        radio_failsafe = true;
    }
}

static void runMotors() 
{
    motors.set(0, m1_command_PWM);
    motors.set(1, m2_command_PWM);
    motors.set(2, m3_command_PWM);
    motors.set(3, m4_command_PWM);

    motors.run();
}

static void maintainLoopRate(const uint32_t freq, const uint32_t current_time) 
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
    float invFreq = 1.0/freq*1000000.0;
    auto checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time)) {
        checker = micros();
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
        digitalWrite(13, blinkAlternate); //Pin 13 is built in LED

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

static void setupBlink(int numBlinks,int upTime, int downTime) 
{
    //DESCRIPTION: Simple function to make LED on board blink as desired
    for (int j = 1; j<= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

static void debug(const uint32_t current_time)
{
    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //static uint32_t print_counter;
    //debugState(print_counter, current_time);  
    //debugMotorCommands(print_counter, current_time); 
    //debugLoopRate(print_counter, current_time);      
}

void debugState(uint32_t & print_counter, const uint32_t current_time) 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("roll:%2.2f pitch:%2.2f yaw:%2.2f\n", 
                statePhi, stateTheta, statePsi);
    }
}

void debugMotorCommands(uint32_t & print_counter, const uint32_t current_time) 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf(
                "m1_command:%d m2_command:%d m3_command:%d m4_command:%d\n",
                m1_command_PWM, m2_command_PWM,
                m3_command_PWM, m4_command_PWM);
    }
}

void debugLoopRate(uint32_t & print_counter, const uint32_t current_time) 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("dt:%f\n", dt*1e6);
    }
}

void setup() 
{
    Serial.begin(500000); //USB serial
    delay(500);

    radio_failsafe = false;

    //Pin 13 LED blinker on board, do not modify     
    pinMode(13, OUTPUT); 
    digitalWrite(13, HIGH);

    delay(5);

    //Initialize radio communication
    sbus.Begin();

    //Initialize IMU communication
    IMUinit();

    delay(5);

    //Arm OneShot125 motors
    motors.arm();

    //Indicate entering main loop with 3 quick blinks
    setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
}

void loop() 
{
    static uint32_t _current_time;
    static uint32_t _prev_time;

    //Keep track of what time it is and how much time has elapsed since the last loop
    _prev_time = _current_time;      
    _current_time = micros();      
    dt = (_current_time - _prev_time)/1e6;

    blinkLed(_current_time);

    debug(_current_time);

    getIMUdata(); 

    // Run Haskell Copilot
    void copilot_step(void);
    copilot_step(); 

    runMotors();

    getOpenLoopDemands();

    maintainLoopRate(LOOP_RATE, _current_time); 
}

// Called by Copilot ---------------------------------------------------------

void setMotors(const float m1, const float m2, const float m3, const float m4)
{
    m1_command_PWM = m1;
    m2_command_PWM = m2;
    m3_command_PWM = m3;
    m4_command_PWM = m4;
}

void setAngles(const float phi, const float theta, const float psi)
{
    statePhi = phi;
    stateTheta = theta;
    statePsi = psi;
}
