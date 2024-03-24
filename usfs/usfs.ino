/*
   Based on https://github.com/nickrehm/dRehmFlight
 */

#include <Wire.h> 

#include <sbus.h>

#include <usfs.hpp>

#include <oneshot125.hpp>
#include <vector>

// USFS settings ------------------------------------------------------------

static const uint8_t USFS_ACCEL_BANDWIDTH = 3;
static const uint8_t USFS_GYRO_BANDWIDTH  = 3;
static const uint8_t USFS_QUAT_DIVISOR    = 1;
static const uint8_t USFS_MAG_RATE        = 100;
static const uint8_t USFS_ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t USFS_GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t USFS_BARO_RATE       = 50;


static const uint8_t USFS_INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
Usfs::INTERRUPT_ERROR |
Usfs::INTERRUPT_QUAT;

static const bool USFS_VERBOSE = false;

// ---------------------------------------------------------------------------

// Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
static const uint32_t LOOP_RATE = 2000;

static const std::vector<uint8_t> MOTOR_PINS = {0, 1, 2, 3};

static auto motors = OneShot125(MOTOR_PINS);

static Usfs usfs;

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

// Streams written and read by Copilot.hs
float statePhi;
float stateTheta;
float statePsi;

// Motors
static int m1_command_PWM;
static int m2_command_PWM;
static int m3_command_PWM;
static int m4_command_PWM;

static void IMUinit() 
{
    Serial.begin(115200);
    delay(4000);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(1000);

    usfs.reportChipId();        

    usfs.loadFirmware(USFS_VERBOSE); 

    usfs.begin(
            USFS_ACCEL_BANDWIDTH,
            USFS_GYRO_BANDWIDTH,
            USFS_QUAT_DIVISOR,
            USFS_MAG_RATE,
            USFS_ACCEL_RATE,
            USFS_GYRO_RATE,
            USFS_BARO_RATE,
            USFS_INTERRUPT_ENABLE,
            USFS_VERBOSE); 

    // Clear interrupts
    Usfs::checkStatus();
}

static void readImu() 
{
    auto eventStatus = Usfs::checkStatus(); 

    if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

        usfs.readAccelerometerScaled(AcX, AcY, AcZ);

        AcY = -AcY;
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        usfs.readGyrometerScaled(GyX, GyY, GyZ);

        GyX = -GyX;
        GyZ = -GyZ;
    }
}

static void readReceiver() 
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

static void setupBlink(
        const uint32_t numBlinks, 
        const uint32_t upTime, 
        const uint32_t downTime) 
{
    for (uint32_t j = 1; j<= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

static void debug(const uint32_t current_time)
{
    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //static uint32_t count; debugAccel(count, current_time);  
    //static uint32_t count; debugGyro(count, current_time);  
    //static uint32_t count; debugState(count, current_time);  
    //static uint32_t count; debugMotorCommands(count, current_time); 
    //static uint32_t count; debugLoopRate(count, current_time);      
}

void debugAccel(uint32_t & print_counter, const uint32_t current_time) 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("accelX:%+3.3f accelY:%+3.3f accelZ:%+3.3f\n", 
                AcX, AcY, AcZ);
    }
}

void debugGyro(uint32_t & print_counter, const uint32_t current_time) 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("gyroX:%+3.3f gyroY:%+3.3f gyroZ:%+3.3f\n", 
                GyX, GyY, GyZ);
    }
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

    // Pin 13 LED blinker on board, do not modify     
    pinMode(13, OUTPUT); 
    digitalWrite(13, HIGH);

    delay(5);

    // Initialize radio communication
    sbus.Begin();

    // Initialize IMU communication
    IMUinit();

    delay(5);

    // Arm OneShot125 motors
    motors.arm();

    // Indicate entering main loop with 3 quick blinks
    setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
}

void loop() 
{
    static uint32_t _current_time;
    static uint32_t _prev_time;

    // Keep track of what time it is and how much time has elapsed since the last loop
    _prev_time = _current_time;      
    _current_time = micros();      
    dt = (_current_time - _prev_time)/1e6;

    blinkLed(_current_time);

    debug(_current_time);

    readImu(); 

    // Run Haskell Copilot
    void copilot_step(void);
    copilot_step(); 

    runMotors();

    readReceiver();

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
