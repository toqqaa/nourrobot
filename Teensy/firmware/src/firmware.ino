#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "nour_msgs/Velocities.h"
#include "nour_msgs/PID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Motor and Encoder
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

// Configuration
#include "nour_base_config.h"

// Constants
#define COMMAND_RATE 20 // Hz
#define IMU_PUBLISH_RATE 50 // Hz
#define DEBUG_RATE 5 // Hz

// MPU6050 Configuration
#define INTERRUPT_PIN 2  // D2 on Arduino Uno

// Hardware Definitions
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

// ROS Node
ros::NodeHandle nh;

// IMU Variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/Motion vars
Quaternion q;
VectorInt16 aa;         // Accelerometer measurements
VectorInt16 aaReal;     // Gravity-free accelerometer measurements
VectorFloat gravity;    // Gravity vector

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/quaternion", &imu_msg);

// Velocity Control
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

// ROS Messages
nour_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

// Function Prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const nour_msgs::PID& pid);
void moveBase();
void stopBase();
float steer(float steering_angle);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void printDebug();
void dmpDataReady();
bool initIMU();
void publishIMU();

// ROS Subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<nour_msgs::PID> pid_sub("pid", PIDCallback);

// Interrupt Detection Routine
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

bool initIMU() {
    // Initialize I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    if (!mpu.testConnection()) {
        nh.logerror("MPU6050 connection failed");
        return false;
    }

    // Initialize DMP
    devStatus = mpu.dmpInitialize();

    // Set your own offsets here:
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        return true;
    } else {
        nh.logerror("DMP Initialization failed");
        return false;
    }
}

void publishIMU() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        // Wait for MPU interrupt or extra packet(s) available
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Get IMU data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // Fill IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu_link";
        
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        
        imu_msg.linear_acceleration.x = aaReal.x * 9.81 / 16384.0; // Convert to m/s²
        imu_msg.linear_acceleration.y = aaReal.y * 9.81 / 16384.0;
        imu_msg.linear_acceleration.z = aaReal.z * 9.81 / 16384.0;
        
        VectorInt16 gyro;
        mpu.dmpGetGyro(&gyro, fifoBuffer);
        imu_msg.angular_velocity.x = gyro.x * (M_PI / 180.0); // Convert to rad/s
        imu_msg.angular_velocity.y = gyro.y * (M_PI / 180.0);
        imu_msg.angular_velocity.z = gyro.z * (M_PI / 180.0);
        
        imu_pub.publish(&imu_msg);
    }
}

void setup() {
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(imu_pub);

    // Initialize IMU
    if (!initIMU()) {
        nh.logfatal("IMU initialization failed");
    }

    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("NOUR BASE CONNECTED");
    delay(1);
}

void loop() {
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;

    // Motor control
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
        moveBase();
        prev_control_time = millis();
    }

    // Stop motors if no command received
    if ((millis() - g_prev_command_time) >= 400) {
        stopBase();
    }

    // Publish IMU data
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE)) {
        publishIMU();
        prev_imu_time = millis();
    }

    // Debug output
    if (DEBUG && (millis() - prev_debug_time) >= (1000 / DEBUG_RATE)) {
        printDebug();
        prev_debug_time = millis();
    }

    nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist& cmd_msg) {
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;
    g_prev_command_time = millis();
}

void PIDCallback(const nour_msgs::PID& pid) {
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void moveBase() {
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));    

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1) {
        float current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    } else {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase() {
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

float steer(float steering_angle) {
    float servo_steering_angle;
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);
    steering_servo.write(servo_steering_angle);
    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug() {
    char buffer[50];
    sprintf(buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf(buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
    nh.loginfo(buffer);
    sprintf(buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
    nh.loginfo(buffer);
    sprintf(buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
    nh.loginfo(buffer);
}
