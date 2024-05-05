/*
   Based on https://github.com/nickrehm/dRehmFlight
 */


#include <Wire.h> 

#include <sbus.h>

#define _MAIN
#include <teensy_streams.h>

#include <teensy_ekf.hpp>
#include <usfs.hpp>
//#include <vl53l1_arduino.h>
#include <oneshot125.hpp>

#include <vector>

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

//static auto vl53l1 = VL53L1_Arduino(&Wire1);

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

    //vl53l1.begin();

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

    // We swap quaterion qw/qx, qy/qz to accommodate upside-down USFS mounting
    if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
        usfs.readQuaternion(
                stream_quat_x, stream_quat_w, stream_quat_z, stream_quat_y);
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

static void readRangefinder(void)
{
    const auto msec_curr = millis();
    static uint32_t msec_prev;

    if (msec_curr - msec_prev > (1000 / RANGEFINDER_FREQ)) {
        stream_rangefinder_distance =  0;//vl53l1.readDistance();
        msec_prev = msec_curr;
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

static void runEkf(const ekfAction_e action)
{
    stream_ekf_action = action;
    stream_now_msec = millis();
    ekf_step();
}

static void debug(const uint32_t current_time)
{

    static uint32_t previous_time;

    if (current_time - previous_time > 10000) {

        previous_time = current_time;

        //debugAccel();  
        //debugGyro();  
        //debugQuat();  
        //debugState();  
        //debugMotorCommands(); 
        //debugLoopRate();      
        //debugRangefinder();      
    }
}

void setup()
{
    Serial.begin(115200);

    initImu();

    stream_radio_failsafe = false;

    delay(5);

    sbus.Begin();

    initLed();

    runEkf(EKF_INIT);

    delay(5);

    motors.arm();

} // setup

void loop()
{
    auto currentTime = updateTime();

    blinkLed(currentTime);

    debug(currentTime);

    readImu();

    readRangefinder();

    runEkf(EKF_PREDICT);

    copilot_step(); 

    runMotors();

    readReceiver();

    maintainLoopRate(currentTime); 

}  // loop

// Called by Copilot ---------------------------------------------------------

void setVehicleState(const float phi, const float theta, const float psi)
{
    _phi = phi;
    _theta = theta;
    _psi = psi;
}

void setMotors(const float m1, const float m2, const float m3, const float m4)
{
    m1_command = m1;
    m2_command = m2;
    m3_command = m3;
    m4_command = m4;
}

// Called by EKF ---------------------------------------------------------

void setStateIsInBounds(const bool isInBounds)
{
}

void setState(const vehicleState_t & state)
{
}

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
    Serial.printf("qw:%+3.3f qx:%+3.3f qy:%+3.3f qz:%+3.3f\n",
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

