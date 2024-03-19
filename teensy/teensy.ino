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
static const float ACCEL_SCALE_FACTOR = 16384;

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
static const uint16_t channel_1_failsafe = 1000; //thro
static const uint16_t channel_2_failsafe = 1500; //ail
static const uint16_t channel_3_failsafe = 1500; //elev
static const uint16_t channel_4_failsafe = 1500; //rudd
static const uint16_t channel_5_failsafe = 2000; //gear, greater than 1500 = throttle cut
static const uint16_t channel_6_failsafe = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
static const float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
static const float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
static const float AccErrorX = 0.0;
static const float AccErrorY = 0.0;
static const float AccErrorZ = 0.0;
static const float GyroErrorX = 0.0;
static const float GyroErrorY= 0.0;
static const float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
static const float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
static const float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
static const float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
static const float maxYaw = 160.0;     //Max yaw rate in deg/sec

static const std::vector<uint8_t> MOTOR_PINS = {0, 1, 2, 3};

static auto motors = OneShot125(MOTOR_PINS);

static MPU6050 mpu6050;

bfs::SbusRx sbus(&Serial5);

// Streams
float thro_des, roll_des, pitch_des, yaw_des;
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float roll_IMU, pitch_IMU, yaw_IMU;
float dt;
bool throttle_is_down;

// General stuff
static uint32_t current_time;
static uint32_t print_counter;
static uint32_t blink_counter;
static uint32_t blink_delay;
static bool blinkAlternate;

// Radio communication:
static uint16_t channel_1_pwm;
static uint16_t channel_2_pwm;
static uint16_t channel_3_pwm;
static uint16_t channel_4_pwm;
static uint16_t channel_5_pwm;
static uint16_t channel_6_pwm;

// Normalized desired state:
static float roll_passthru, pitch_passthru, yaw_passthru;

// Motors
static float m1_command_scaled;
static float m2_command_scaled;
static float m3_command_scaled;
static float m4_command_scaled;
static int m1_command_PWM;
static int m2_command_PWM;
static int m3_command_PWM;
static int m4_command_PWM;

//Flight status
static bool armedFly;

static void armedStatus() 
{
    //DESCRIPTION: Check if the throttle cut is off and the throttle input is
    //low to prepare for flight.
    if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
        armedFly = true;
    }
}

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
    static float accX_prev, accY_prev, accZ_prev;
    static float gyroX_prev, gyroY_prev, gyroZ_prev;

    int16_t AcX = 0, AcY = 0, AcZ = 0, GyX = 0, GyY = 0, GyZ = 0;

    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    //Accelerometer (Gs), corrected with the calculated error values
    accX = AcX / ACCEL_SCALE_FACTOR - AccErrorX;
    accY = AcY / ACCEL_SCALE_FACTOR - AccErrorY;
    accZ = AcZ / ACCEL_SCALE_FACTOR - AccErrorZ;

    //LP filter accelerometer data
    accX = (1.0 - B_accel)*accX_prev + B_accel*accX;
    accY = (1.0 - B_accel)*accY_prev + B_accel*accY;
    accZ = (1.0 - B_accel)*accZ_prev + B_accel*accZ;
    accX_prev = accX;
    accY_prev = accY;
    accZ_prev = accZ;

    //Gyro (DPS),  corrected with the calculated error values
    gyroX = GyX / GYRO_SCALE_FACTOR - GyroErrorX; 
    gyroY = GyY / GYRO_SCALE_FACTOR - GyroErrorY;
    gyroZ = GyZ / GYRO_SCALE_FACTOR - GyroErrorZ;

    //LP filter gyro data
    gyroX = (1.0 - B_gyro)*gyroX_prev + B_gyro*gyroX;
    gyroY = (1.0 - B_gyro)*gyroY_prev + B_gyro*gyroY;
    gyroZ = (1.0 - B_gyro)*gyroZ_prev + B_gyro*gyroZ;
    gyroX_prev = gyroX;
    gyroY_prev = gyroY;
    gyroZ_prev = gyroZ;
}

static void getDesState() 
{
    //DESCRIPTION: Normalizes desired control values to appropriate values
    /*
     * Updates the desired state variables thro_des, roll_des, pitch_des, and
     * yaw_des. These are computed by using the raw RC pwm commands and scaling
     * them to be within our limits defined in setup. thro_des stays within 0
     * to 1 range.  roll_des and pitch_des are scaled to be within max
     * roll/pitch amount in either degrees (angle mode) or degrees/sec (rate
     * mode). yaw_des is scaled to be within max yaw in degrees/sec. Also
     * creates roll_passthru, pitch_passthru, and yaw_passthru variables, to be
     * used in commanding motors/servos with direct unstabilized commands in
     * controlMixer().
     */
    thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
    roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
    yaw_des = -(channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
    roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
    pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
    yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
    pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

static void scaleCommands() {
    //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo
    //protocol
    /*
     * mX_command_scaled variables from the mixer function are scaled to
     * 125-250us for OneShot125 protocol. sX_command_scaled variables from the
     * mixer function are scaled to 0-180 for the servo library using standard
     * PWM.  mX_command_PWM are updated here which are used to command the
     * motors in commandMotors(). sX_command_PWM are updated which are used to
     * command the servos.
     */

    //Scaled to 125us - 250us for oneshot125 protocol
    m1_command_PWM = m1_command_scaled*125 + 125;
    m2_command_PWM = m2_command_scaled*125 + 125;
    m3_command_PWM = m3_command_scaled*125 + 125;
    m4_command_PWM = m4_command_scaled*125 + 125;

    //Constrain commands to motors within oneshot125 bounds
    m1_command_PWM = constrain(m1_command_PWM, 125, 250);
    m2_command_PWM = constrain(m2_command_PWM, 125, 250);
    m3_command_PWM = constrain(m3_command_PWM, 125, 250);
    m4_command_PWM = constrain(m4_command_PWM, 125, 250);
}

static void getCommands() 
{
    //DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
     * Updates radio PWM commands in loop based on current available commands.
     * channel_x_pwm is the raw command used in the rest of the loop. If using
     * a PWM or PPM receiver, the radio commands are retrieved from a function
     * in the readPWM file separate from this one which is running a bunch of
     * interrupts to continuously update the radio readings. If using an SBUS
     * receiver, the alues are pulled from the SBUS library directly.  The raw
     * radio commands are filtered with a first order low-pass filter to
     * eliminate any really high frequency noise. 
     */

    if (sbus.Read()) {

        //sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;  
        float bias  = 895.0; 

        channel_1_pwm = sbus.data().ch[0] * scale + bias;
        channel_2_pwm = sbus.data().ch[1] * scale + bias;
        channel_3_pwm = sbus.data().ch[2] * scale + bias;
        channel_4_pwm = sbus.data().ch[3] * scale + bias;
        channel_5_pwm = sbus.data().ch[4] * scale + bias;
        channel_6_pwm = sbus.data().ch[5] * scale + bias; 
    }

    static uint16_t channel_1_pwm_prev, channel_2_pwm_prev,
                         channel_3_pwm_prev, channel_4_pwm_prev;

    //Low-pass the critical commands and update previous values
    float b = 0.7; //Lower=slower, higher=noiser
    channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
    channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
    channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
    channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
}

static void failSafe() 
{
    //DESCRIPTION: If radio gives garbage values, set all commands to default
    //values
    /*
     * Radio connection failsafe used to check if the getCommands() function is
     * returning acceptable pwm values. If any of the commands are lower than
     * 800 or higher than 2200, then we can be certain that there is an issue
     * with the radio connection (most likely hardware related). If any of the
     * channels show this failure, then all of the radio commands channel_x_pwm
     * are set to default failsafe values specified in the setup. Comment out
     * this function when troubleshooting your radio connection in case any
     * extreme values are triggering this function to overwrite the printed
     * variables.
     */
    unsigned minVal = 800;
    unsigned maxVal = 2200;

    //Triggers for failure criteria
    int check1 = (channel_1_pwm > maxVal || channel_1_pwm < minVal) ? 1 : 0;
    int check2 = (channel_2_pwm > maxVal || channel_2_pwm < minVal) ? 1 : 0;
    int check3 = (channel_3_pwm > maxVal || channel_3_pwm < minVal) ? 1 : 0;
    int check4 = (channel_4_pwm > maxVal || channel_4_pwm < minVal) ? 1 : 0;
    int check5 = (channel_5_pwm > maxVal || channel_5_pwm < minVal) ? 1 : 0;
    int check6 = (channel_6_pwm > maxVal || channel_6_pwm < minVal) ? 1 : 0;

    //If any failures, set to default failsafe values
    if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
        channel_1_pwm = channel_1_failsafe;
        channel_2_pwm = channel_2_failsafe;
        channel_3_pwm = channel_3_failsafe;
        channel_4_pwm = channel_4_failsafe;
        channel_5_pwm = channel_5_failsafe;
        channel_6_pwm = channel_6_failsafe;
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

static void throttleCut() 
{
    //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
    /*
       Monitors the state of radio command channel_5_pwm and directly sets the
       mx_command_PWM values to minimum (120 is minimum for oneshot125
       protocol, 0 is minimum for standard PWM servo library used) if channel 5
       is high. This is the last function called before commandMotors() is
       called so that the last thing checked is if the user is giving
       permission to command the motors to anything other than minimum value.
       Safety first.

       channel_5_pwm is HIGH then throttle cut is OFF and throttle value can
       change. (ThrottleCut is DEACTIVATED) channel_5_pwm is LOW then throttle
       cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED),
       (drone is DISARMED)
     */
    if ((channel_5_pwm < 1500) || (armedFly == false)) {
        armedFly = false;
        m1_command_PWM = 120;
        m2_command_PWM = 120;
        m3_command_PWM = 120;
        m4_command_PWM = 120;
    }
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

void debugRadioData() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F(" CH1:"));
        Serial.print(channel_1_pwm);
        Serial.print(F(" CH2:"));
        Serial.print(channel_2_pwm);
        Serial.print(F(" CH3:"));
        Serial.print(channel_3_pwm);
        Serial.print(F(" CH4:"));
        Serial.print(channel_4_pwm);
        Serial.print(F(" CH5:"));
        Serial.print(channel_5_pwm);
        Serial.print(F(" CH6:"));
        Serial.println(channel_6_pwm);
    }
}

void debugDesiredState() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("thro_des:"));
        Serial.print(thro_des);
        Serial.print(F(" roll_des:"));
        Serial.print(roll_des);
        Serial.print(F(" pitch_des:"));
        Serial.print(pitch_des);
        Serial.print(F(" yaw_des:"));
        Serial.println(yaw_des);
    }
}

void debugGyroData() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("gyroX:"));
        Serial.print(gyroX);
        Serial.print(F(" gyroY:"));
        Serial.print(gyroY);
        Serial.print(F(" gyroZ:"));
        Serial.println(gyroZ);
    }
}

void debugAccelData() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("accX:"));
        Serial.print(accX);
        Serial.print(F(" accY:"));
        Serial.print(accY);
        Serial.print(F(" accZ:"));
        Serial.println(accZ);
    }
}

void debugRollPitchYaw() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("roll:"));
        Serial.print(roll_IMU);
        Serial.print(F(" pitch:"));
        Serial.print(pitch_IMU);
        Serial.print(F(" yaw:"));
        Serial.println(yaw_IMU);
    }
}


void debugMotorCommands() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("m1_command:"));
        Serial.print(m1_command_PWM);
        Serial.print(F(" m2_command:"));
        Serial.print(m2_command_PWM);
        Serial.print(F(" m3_command:"));
        Serial.print(m3_command_PWM);
        Serial.print(F(" m4_command:"));
        Serial.print(m4_command_PWM);
    }
}

void debugLoopRate() 
{
    if (current_time - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("dt:"));
        Serial.println(dt*1000000.0);
    }
}

static void debug(void)
{
    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //debugRadioData();     
    //debugDesiredState();  
    //debugGyroData();      
    //debugAccelData();     
    debugRollPitchYaw();  
    //debugMotorCommands(); 
    //debugServoCommands(); 
    //debugLoopRate();      
}

void setup() 
{
    Serial.begin(500000); //USB serial
    delay(500);

    //Pin 13 LED blinker on board, do not modify     
    pinMode(13, OUTPUT); 
    digitalWrite(13, HIGH);

    delay(5);

    //Initialize radio communication
    sbus.Begin();

    //Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = channel_1_failsafe;
    channel_2_pwm = channel_2_failsafe;
    channel_3_pwm = channel_3_failsafe;
    channel_4_pwm = channel_4_failsafe;
    channel_5_pwm = channel_5_failsafe;
    channel_6_pwm = channel_6_failsafe;

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

    // Get arming status
    armedStatus(); //Check if the throttle cut is off and throttle is low.

    // Get vehicle state
    getIMUdata(); 

    // Copilot state estimator
    void copilot_step_estimator(void);
    copilot_step_estimator(); 

    // Compute desired state
    getDesState(); 

    // Copilot core
    void copilot_proxy_step_core(void);
    copilot_proxy_step_core(); 

    void copilot_step_core(void);
    copilot_step_core(); 

    scaleCommands(); 

    // Throttle cut check
    throttleCut(); //Directly sets motor commands to low based on state of ch5

    // Command actuators
    commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol

    // Get vehicle commands for next loop iteration
    getCommands();
    failSafe(); 

    // Flag for resetting PIDs
    throttle_is_down = channel_1_pwm < 1060;

    // Regulate loop rate
    loopRate(2000); //Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
}

// Called by Copilot ---------------------------------------------------------

void setMotorsProxy(const float m1, const float m2, const float m3, const float m4)

{
    m1_command_scaled = m1;
    m2_command_scaled = m2;
    m3_command_scaled = m3;
    m4_command_scaled = m4;
}
void setMotors(const float m1, const float m2, const float m3, const float m4)

{
}


void setState(const float phi, const float theta, const float psi)
{
    roll_IMU = phi;
    pitch_IMU = theta;
    yaw_IMU = psi;
}
