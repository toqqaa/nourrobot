#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>
#include <Wire.h>

// MPU6050 includes
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "nour_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "nour_msgs/PID.h"
//header file for IMU quaternion
#include "geometry_msgs/Quaternion.h"

#include "lib/config/nour_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 1000

// MPU control/status vars
bool dmpReady = false;          // set true if DMP init was successful
uint8_t mpuIntStatus;           // holds actual interrupt status byte from MPU
uint8_t devStatus;              // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;            // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer
bool imu_is_initialized = false;

// MPU6050 configuration
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 11       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

// MPU6050 object - default address 0x68
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// Encoders setup
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 

Servo steering_servo;

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::NOUR_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

// Timing variables
unsigned long prev_control_time = 0;
unsigned long prev_imu_time = 0;
unsigned long prev_debug_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const nour_msgs::PID& pid);

// Interrupt detection routine for MPU6050
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<nour_msgs::PID> pid_sub("pid", PIDCallback);

// ROS publishers
nour_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

// IMU quaternion publisher
geometry_msgs::Quaternion quat_msg;
ros::Publisher quat_pub("quaternion", &quat_msg);

extern "C" {
    // Provide the missing _write function for Teensy
    int _write(int file, char *ptr, int len) {
        if (file == 1) { // stdout
            Serial.write(ptr, len);
        }
        return len;
    }
}

bool initIMU() {
    // initialize MPU6050
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    // verify connection
    if (!mpu.testConnection()) {
        return false;
    }
    
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(228);
    mpu.setYGyroOffset(93);
    mpu.setZGyroOffset(-64);
    mpu.setZAccelOffset(1421); // 1688 factory default for my test chip
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        
        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        return true;
    }
    
    return false;
}

void publishIMU() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        
        // publish to ROS
        quat_msg.w = q.w;
        quat_msg.x = q.x;
        quat_msg.y = q.y;
        quat_msg.z = q.z;
        
        quat_pub.publish(&quat_msg);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void setup()
{
    // initialize i2c bus
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    
    // setup servo
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // initialize ROS node
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(quat_pub);  // advertise the quaternion topic

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("NOUR BASE CONNECTED");
    
    // Initialize IMU
    if (initIMU()) {
        nh.loginfo("IMU Initialized");
        imu_is_initialized = true;
    } else {
        nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    
    delay(1);
}

void loop()
{
    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

    //call all the callbacks waiting to be called
    nh.spinOnce();
}

void PIDCallback(const nour_msgs::PID& pid)
{
    //callback function every time PID constants are received from nour_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));    

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        
        current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

float steer(float steering_angle)
{
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}