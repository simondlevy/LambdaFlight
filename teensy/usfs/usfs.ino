 /*
   Based on https://github.com/nickrehm/dRehmFlight
 */


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

static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
                                        Usfs::INTERRUPT_ERROR |
                                        Usfs::INTERRUPT_QUAT;

static const bool VERBOSE = false;

static const uint8_t REPORT_HZ = 2;

static Usfs usfs;

static void powerPin(const uint8_t pin, const bool hilo)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, hilo);
}

void setup()
{
    powerPin(21, HIGH);
    powerPin(22, LOW);

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
    static uint32_t _interruptCount;

    static float temperature, pressure;
    static float ax, ay, az;
    static float gx, gy, gz;

    uint8_t eventStatus = Usfs::checkStatus(); 

    if (Usfs::eventStatusIsError(eventStatus)) { 

        Usfs::reportError(eventStatus);
    }

    if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

        usfs.readAccelerometerScaled(ax, ay, az);
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        usfs.readGyrometerScaled(gx, gy, gz);
    }

    static uint32_t _msec;

    uint32_t msec = millis();

    static float yaw, pitch, roll;

    if (msec-_msec > 1000/REPORT_HZ) { 

        _interruptCount = 0;
        _msec = msec;

        Serial.print("Ax = ");
        Serial.print((int)1000 * ax);
        Serial.print(" Ay = ");
        Serial.print((int)1000 * ay);
        Serial.print(" Az = ");
        Serial.print((int)1000 * az);
        Serial.println(" g");
        Serial.print("Gx = ");
        Serial.print( gx, 2);
        Serial.print(" Gy = ");
        Serial.print( gy, 2);
        Serial.print(" Gz = ");
        Serial.print( gz, 2);
        Serial.println(" deg/s");
    } 

}  // loop
