/*
   Based on https://github.com/nickrehm/dRehmFlight
 */


#include <Wire.h> 

#include <sbus.h>

#include <usfs.hpp>
#include <VL53L1X.h>
#include <oneshot125.hpp>

#include <vector>

static const uint8_t PWR_PIN = 21;
static const uint8_t GND_PIN = 22;

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

static const uint32_t REPS = 1000;

static Usfs usfs;

static void powerPin(const uint8_t pin, const bool hilo)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, hilo);
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
    static uint32_t _count;

    static int32_t _ax, _ay, _az;

    if (_count < REPS) {

        const auto eventStatus = Usfs::checkStatus(); 

        if (Usfs::eventStatusIsError(eventStatus)) { 

            Usfs::reportError(eventStatus);
        }

        if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

            int16_t raw[3] = {};

            usfs.readAccelerometerRaw(raw);

            _count++;

            _ax += raw[0];
            _ay += raw[1];
            _az += (2048 + raw[2]);
        }
    }

    else if (_count == REPS) {

        Serial.printf("static const int16_t ACCEL_X_OFFSET = %d;\n", _ax/REPS);
        Serial.printf("static const int16_t ACCEL_Y_OFFSET = %d;\n", _ay/REPS);
        Serial.printf("static const int16_t ACCEL_Z_OFFSET = %d;\n", _az/REPS);

        _count++;
    }
}
