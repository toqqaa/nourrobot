/**
 * Modified ApplyCalib Node
 * 
 * This is a modified version of the ApplyCalib class with added debugging
 * and error handling to help diagnose issues.
 */

#include "imu_calib/apply_calib.h"

namespace imu_calib
{

ApplyCalib::ApplyCalib() :
  gyro_sample_count_(0),
  gyro_bias_x_(0.0),
  gyro_bias_y_(0.0),
  gyro_bias_z_(0.0),
  calib_ready_(false)  // Flag to track calibration status
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string calib_file;
  nh_private.param<std::string>("calib_file", calib_file, "imu_calib.yaml");

  // Try to load calibration, but don't exit if it fails
  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    ROS_ERROR("Calibration could not be loaded from %s", calib_file.c_str());
    ROS_WARN("Will run without acceleration calibration");
  }
  else
  {
    calib_ready_ = true;
    ROS_INFO("Calibration loaded successfully from %s", calib_file.c_str());
  }

  nh_private.param<bool>("calibrate_gyros", calibrate_gyros_, true);
  nh_private.param<int>("gyro_calib_samples", gyro_calib_samples_, 100);
  
  // Get frame ID from parameter
  nh_private.param<std::string>("frame_id", frame_id_, "imu_link");
  ROS_INFO("Using frame_id: %s", frame_id_.c_str());

  int queue_size;
  nh_private.param<int>("queue_size", queue_size, 5);

  raw_sub_ = nh.subscribe("raw_imu", queue_size, &ApplyCalib::rawImuCallback, this);
  corrected_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", queue_size);
  mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", queue_size);
  
  ROS_INFO("ApplyCalib node initialized. Waiting for raw_imu messages...");
}

void ApplyCalib::rawImuCallback(nour_msgs::Imu::ConstPtr raw)
{
  static int message_count = 0;
  message_count++;
  
  // Log the first few messages and then every 100th message
  if (message_count <= 5 || message_count % 100 == 0) {
    ROS_INFO("Received raw IMU message #%d", message_count);
  }

  if (calibrate_gyros_)
  {
    ROS_INFO_ONCE("Calibrating gyros; do not move the IMU");

    // recursively compute mean gyro measurements
    gyro_sample_count_++;
    gyro_bias_x_ = ((gyro_sample_count_ - 1) * gyro_bias_x_ + raw->angular_velocity.x) / gyro_sample_count_;
    gyro_bias_y_ = ((gyro_sample_count_ - 1) * gyro_bias_y_ + raw->angular_velocity.y) / gyro_sample_count_;
    gyro_bias_z_ = ((gyro_sample_count_ - 1) * gyro_bias_z_ + raw->angular_velocity.z) / gyro_sample_count_;

    if (gyro_sample_count_ >= gyro_calib_samples_)
    {
      ROS_INFO("Gyro calibration complete! (bias = [%.3f, %.3f, %.3f])", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
      calibrate_gyros_ = false;
    }
    else if (gyro_sample_count_ % 10 == 0) {
      ROS_DEBUG("Gyro calibration progress: %d/%d samples", gyro_sample_count_, gyro_calib_samples_);
    }
    
    // Don't return early - still publish uncalibrated data
    // This was preventing any messages from being published during calibration
  }

  // Create calibrated data object
  sensor_msgs::Imu corrected;

  corrected.header.stamp = ros::Time::now();
  corrected.header.frame_id = frame_id_;

  if (calib_ready_) {
    // Input array for acceleration calibration
    double raw_accel[3];
    // Output array for calibrated acceleration
    double corrected_accel[3];

    // Pass received acceleration to input array
    raw_accel[0] = raw->linear_acceleration.x;
    raw_accel[1] = raw->linear_acceleration.y;
    raw_accel[2] = raw->linear_acceleration.z;

    // Apply calibration
    calib_.applyCalib(raw_accel, corrected_accel);

    // Pass calibrated acceleration to corrected IMU data object
    corrected.linear_acceleration.x = corrected_accel[0];
    corrected.linear_acceleration.y = corrected_accel[1];
    corrected.linear_acceleration.z = corrected_accel[2];
  } else {
    // No calibration available, pass through raw data
    corrected.linear_acceleration = raw->linear_acceleration;
    ROS_DEBUG_THROTTLE(10.0, "Using uncalibrated acceleration data");
  }

  // Add calibration bias to received angular velocity and pass to corrected IMU data object
  corrected.angular_velocity.x = raw->angular_velocity.x - gyro_bias_x_;
  corrected.angular_velocity.y = raw->angular_velocity.y - gyro_bias_y_;
  corrected.angular_velocity.z = raw->angular_velocity.z - gyro_bias_z_;

  // Set covariance as in original code
  corrected.orientation_covariance[0] = 0.0025;
  corrected.orientation_covariance[4] = 0.0025;
  corrected.orientation_covariance[8] = 0.0025;

  corrected.angular_velocity_covariance[0] = 0.000001;
  corrected.angular_velocity_covariance[4] = 0.000001;
  corrected.angular_velocity_covariance[8] = 0.000001;

  corrected.linear_acceleration_covariance[0] = 0.0001;
  corrected.linear_acceleration_covariance[4] = 0.0001;
  corrected.linear_acceleration_covariance[8] = 0.0001;

  // Publish calibrated IMU data
  corrected_pub_.publish(corrected);
  
  if (message_count % 100 == 0) {
    ROS_INFO("Published calibrated IMU data, message #%d", message_count);
  }

  // Process magnetometer data
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros::Time::now();
  mag_msg.header.frame_id = frame_id_;

  // Scale received magnetic field data (if needed)
  mag_msg.magnetic_field.x = raw->magnetic_field.x;
  mag_msg.magnetic_field.y = raw->magnetic_field.y;
  mag_msg.magnetic_field.z = raw->magnetic_field.z;

  mag_pub_.publish(mag_msg);
}

} // namespace imu_calib
