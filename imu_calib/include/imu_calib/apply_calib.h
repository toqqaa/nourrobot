/**
 * IMU Calibration - Apply Calibration
 * 
 * Header file for the ApplyCalib class
 */

#ifndef IMU_CALIB_APPLY_CALIB_H
#define IMU_CALIB_APPLY_CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "nour_msgs/Imu.h"
#include "imu_calib/accel_calib.h"

namespace imu_calib
{

class ApplyCalib
{
public:
  ApplyCalib();

private:
  // Callback for IMU data
  void rawImuCallback(nour_msgs::Imu::ConstPtr raw);

  // ROS objects
  ros::Subscriber raw_sub_;
  ros::Publisher corrected_pub_;
  ros::Publisher mag_pub_;

  // Calibration objects
  AccelCalib calib_;
  bool calibrate_gyros_;
  int gyro_calib_samples_;
  int gyro_sample_count_;
  double gyro_bias_x_;
  double gyro_bias_y_;
  double gyro_bias_z_;
  
  // New variables that were missing
  bool calib_ready_;    // Flag to track calibration status
  std::string frame_id_; // Frame ID for the IMU messages
};

} // namespace imu_calib

#endif // IMU_CALIB_APPLY_CALIB_H
