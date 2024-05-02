/*
   Based on https://github.com/nickrehm/dRehmFlight
 */


#include <Wire.h> 

#include <sbus.h>

#include <usfs.hpp>
#include <vl53l1_arduino.h>
#include <oneshot125.hpp>

#include <vector>

#define _MAIN

#include <streams.h>
#include <new_teensy_ekf.hpp>

void copilot_step(void);

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

static const std::vector<uint8_t> MOTOR_PINS = {0, 1, 2, 3};

static auto motors = OneShot125(MOTOR_PINS);

bfs::SbusRx sbus(&Serial2);

uint32_t stream_now_msec;
float stream_dt;
float stream_channel1_raw;
float stream_channel2_raw;
float stream_channel3_raw;
float stream_channel4_raw;
float stream_channel5_raw;
bool stream_radio_failsafe;

// Motors set by Haskell
static int m1_command_PWM;
static int m2_command_PWM;
static int m3_command_PWM;
static int m4_command_PWM;

static vehicleState_t _vehicleState;

// ---------------------------------------------------------------------------

static const uint8_t REPORT_HZ = 2;

static Usfs usfs;

static auto vl53l1 = VL53L1_Arduino(&Wire1);

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
    delay(100);

    vl53l1.begin();

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
                stream_accel.x, stream_accel.y, stream_accel.z);

        stream_ekfAction = EKF_UPDATE_WITH_ACCEL;

        ekf_step();
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        usfs.readGyrometerScaled(
                stream_gyro.x, stream_gyro.y, stream_gyro.z);

        stream_ekfAction = EKF_UPDATE_WITH_GYRO;

        ekf_step();
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

static void readRangefinder(void)
{
    
    const auto msec_curr = millis();
    static uint32_t msec_prev;

    if (msec_curr - msec_prev > (1000 / RANGEFINDER_FREQ)) {
        stream_rangefinder_distance =  vl53l1.readDistance();
        msec_prev = msec_curr;

        stream_ekfAction = EKF_UPDATE_WITH_RANGE;
        ekf_step();
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
        debugState();  
        //debugMotorCommands(); 
        //debugLoopRate();      
        //debugRangefinder();      
    }
}


void debugRangefinder(void) 
{
    Serial.printf("dist: %f\n", stream_rangefinder_distance);
}


void debugRadio(void) 
{
    Serial.printf("ch1:%4.0f ch2:%4.0f ch3:%4.0f ch4:%4.0f ch5:%4.0f\n",
            stream_channel1_raw, stream_channel2_raw, stream_channel3_raw, 
            stream_channel4_raw, stream_channel5_raw);
}

void debugAccel(void) 
{
    Serial.printf("accelX:%+3.3f accelY:%+3.3f accelZ:%+3.3f\n", 
            stream_accel.x, stream_accel.y, stream_accel.z);
}

void debugGyro(void) 
{
    Serial.printf("gyroX:%+03.3f gyroY:%+03.3f gyroZ:%+03.3f\n", 
            stream_gyro.x, stream_gyro.y, stream_gyro.z);
}

void debugState(void) 
{
    Serial.printf("roll:%2.2f pitch:%2.2f yaw:%2.2f\n", 
            _vehicleState.phi, _vehicleState.theta, _vehicleState.psi);
}

void debugMotorCommands(void) 
{
    Serial.printf(
            "m1_command:%d m2_command:%d m3_command:%d m4_command:%d\n",
            m1_command_PWM, m2_command_PWM,
            m3_command_PWM, m4_command_PWM);
}

void debugLoopRate(void) 
{
    Serial.printf("dt:%f\n", stream_dt*1e6);
}

static void ekfInit(void)
{
    stream_ekfAction = EKF_INIT;
    ekf_step();
}

static void ekfStep(void)
{
    stream_ekfAction = EKF_PREDICT;
    ekf_step();

    stream_ekfAction = EKF_FINALIZE;
    ekf_step();

    stream_ekfAction = EKF_GET_STATE;
    ekf_step();
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

    ekfInit();

    motors.arm();

} // setup

void loop()
{
    static uint32_t _current_time;
    static uint32_t _prev_time;

    // Keep track of what time it is and how much time has elapsed since the last loop
    _prev_time = _current_time;      
    _current_time = micros();      
    stream_dt = (_current_time - _prev_time)/1e6;

    blinkLed(_current_time);

    debug(_current_time);

    readImu();

    readRangefinder();

    ekfStep();

    copilot_step(); 

    runMotors();

    readReceiver();

    maintainLoopRate(_current_time); 

}  // loop

// Called by Copilot ---------------------------------------------------------

void foo(void)
{
}

void setState(const vehicleState_t & vehicleState)
{
    memcpy(&_vehicleState, &vehicleState, sizeof(vehicleState));
}

void setStateIsInBounds(const bool inBounds)
{
}
