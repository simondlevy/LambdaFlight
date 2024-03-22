/*
   Based on https://github.com/nickrehm/dRehmFlight
*/

#include <Wire.h> 

#include <sbus.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#include <oneshot125.hpp>
#include <vector>


//Setup gyro and accel full scale value selection and scale factor

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static const float GYRO_SCALE_FACTOR = 131;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless
//you know what you are doing:

//Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
static const float B_gyro = 0.1;       

// ---------------------------------------------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = {0, 1, 2, 3};

static auto motors = OneShot125(MOTOR_PINS);

static MPU6050 mpu6050;

bfs::SbusRx sbus(&Serial5);

// Streams read by Copilot.hs
float gyroX;
float gyroY;
float gyroZ;
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

// Timing
static uint32_t current_time;

// LED control
static uint32_t print_counter;
static uint32_t blink_counter;
static uint32_t blink_delay;
static bool blinkAlternate;

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
    static float gyroX_prev, gyroY_prev, gyroZ_prev;

    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    AcX = ax;
    AcY = ay;
    AcZ = az;
    GyX = gx;
    GyY = gy;
    GyZ = gz;

    //Gyro (DPS),  corrected with the calculated error values
    gyroX = GyX / GYRO_SCALE_FACTOR; 
    gyroY = GyY / GYRO_SCALE_FACTOR;
    gyroZ = GyZ / GYRO_SCALE_FACTOR;

    //LP filter gyro data
    gyroX = (1.0 - B_gyro)*gyroX_prev + B_gyro*gyroX;
    gyroY = (1.0 - B_gyro)*gyroY_prev + B_gyro*gyroY;
    gyroZ = (1.0 - B_gyro)*gyroZ_prev + B_gyro*gyroZ;
    gyroX_prev = gyroX;
    gyroY_prev = gyroY;
    gyroZ_prev = gyroZ;
}

static void getCommands() 
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

static void commandMotors() 
{
    motors.set(0, m1_command_PWM);
    motors.set(1, m2_command_PWM);
    motors.set(2, m3_command_PWM);
    motors.set(3, m4_command_PWM);

    motors.run();
}

static void loopRate(int freq) 
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
    uint32_t checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

static void loopBlink() 
{
    //DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
     * It looks cool.
     */
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

static void debug(void)
{
    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //debugRadioData();     
    //debugGyroData();      
    //debugRollPitchYaw();  
    //debugMotorCommands(); 
    //debugLoopRate();      
}

void debugGyroData() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("gyroX:%f gyroY:%f gyroZ:%f\n",
                gyroX, gyroY, gyroZ);
    }
}

void debugRollPitchYaw() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf("roll:%2.2f pitch:%2.2f yaw:%2.2f\n", 
                statePhi, stateTheta, statePsi);
    }
}

void debugMotorCommands() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.printf(
                "m1_command:%d m2_command:%d m3_command:%d m4_command:%d\n",
                m1_command_PWM, m2_command_PWM,
                m3_command_PWM, m4_command_PWM);
    }
}

void debugLoopRate() 
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
    m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;
    motors.arm();

    //Indicate entering main loop with 3 quick blinks
    setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
}

void loop() 
{
    static uint32_t _prev_time;

    //Keep track of what time it is and how much time has elapsed since the last loop
    _prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - _prev_time)/1e6;

    loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

    debug();

    // Get vehicle state
    getIMUdata(); 

    // Run Haskell Copilot
    void copilot_step(void);
    copilot_step(); 

    // Run the motors
    commandMotors();

    // Get vehicle commands for next loop iteration
    getCommands();

    // Regulate loop rate
    loopRate(2000); //Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
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
