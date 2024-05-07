/*
   Based on https://github.com/nickrehm/dRehmFlight
 */


#include <Wire.h> 

#include <sbus.h>

#include <usfs.hpp>
#include <VL53L1X.h>
#include <oneshot125.hpp>

#include <vector>

// Power pin settings -------------------------------------------------------

static const uint8_t PWR_PIN = 21;
static const uint8_t GND_PIN = 22;

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


// Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
static const uint32_t LOOP_RATE = 2000;

static Usfs usfs;

static float accel_x, accel_y, accel_z;

static void powerPin(const uint8_t pin, const bool hilo)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, hilo);
}


static void readImu(void)
{
    const auto eventStatus = Usfs::checkStatus(); 

    if (Usfs::eventStatusIsError(eventStatus)) { 

        Usfs::reportError(eventStatus);
    }


    if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

        usfs.readAccelerometerScaled(
                accel_x, accel_y, accel_z);

        // We negate accel Z to accommodate upside-down USFS mounting
        accel_z = -accel_z;
    }

}

static uint32_t updateTime(void)
{
    static uint32_t _current_time;
    static uint32_t _prev_time;

    _prev_time = _current_time;      
    _current_time = micros();      
    return _current_time;
}


static void debug(const uint32_t current_time)
{

    static uint32_t previous_time;

    if (current_time - previous_time > 10000) {

        previous_time = current_time;

    Serial.printf("accelX:%+3.3f accelY:%+3.3f accelZ:%+3.3f\n", 
            accel_x, accel_y, accel_z);
    }
}

void setup()
{
    Serial.begin(115200);

    powerPin(PWR_PIN, HIGH);
    powerPin(GND_PIN, LOW);

    Wire.begin(); 
    Wire.setClock(400000);
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

void loop()
{
    const auto currentTime = updateTime();

    debug(currentTime);

    readImu();

}
