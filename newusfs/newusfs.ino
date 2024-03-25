#include <math.h>

#include <Wire.h>

#include "usfs.hpp"

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = 
Usfs::INTERRUPT_RESET_REQUIRED |
Usfs::INTERRUPT_ERROR |
Usfs::INTERRUPT_QUAT;

static const bool VERBOSE = true;

static const uint8_t REPORT_HZ = 2;

static Usfs usfs;

static void powerPin(const uint8_t pin, const uint8_t val)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, val);
}

void setup()
{
    powerPin(21, HIGH); // 3.3V
    powerPin(22, LOW);  // GND

    Serial.begin(115200);
    delay(4000);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(1000);

    usfs.reportChipId();        

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

} // setup

void loop()
{
    uint8_t eventStatus = Usfs::checkStatus(); 

    if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 
        Serial.printf("Gyro\n");

        //usfs.readAccelerometerScaled(ax, ay, az);
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        Serial.printf("Accel\n");
        //usfs.readGyrometerScaled(gx, gy, gz);
    }

}  // loop
