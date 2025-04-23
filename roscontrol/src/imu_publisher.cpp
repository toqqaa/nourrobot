#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "nour_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher imu_pub;
ros::Subscriber imu_raw_sub;
nour_msgs::Imu imu_data;
ros::Time current_time, last_time;

// Create a Quaternion
tf2::Quaternion quaternion;

double roll = 0.0;  // roll angle
double pitch = 0.0; // pitch angle
double yaw = 0.0;   // yaw angle

double linear_acc_x = 0.0;
double linear_acc_y = 0.0;
double linear_acc_z = 0.0;

void IMUCallback(const std_msgs::Float32MultiArray::ConstPtr &raw_imu_msg)
{
  yaw = raw_imu_msg->data[0];
  pitch = raw_imu_msg->data[1];
  roll = raw_imu_msg->data[2];

  linear_acc_x = 0;
  linear_acc_y = 0;
  linear_acc_z = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;

  imu_raw_sub = nh.subscribe("raw_imu", 50, IMUCallback);
  imu_pub = nh.advertise<nour_msgs::Imu>("imu", 50);

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate loop_rate(50);
  
  while (ros::ok())
  {
    ros::spinOnce();

    // Set the quaternion using Euler angles
    quaternion.setRPY(roll, pitch, yaw);

    // Create geometry_msgs::Vector3 objects for each field
    geometry_msgs::Vector3 linear_acc;
    linear_acc.x = linear_acc_x;
    linear_acc.y = linear_acc_y;
    linear_acc.z = linear_acc_z;

    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = 0;  // Set your angular velocity values here if available
    angular_vel.y = 0;
    angular_vel.z = 0;

    geometry_msgs::Vector3 mag_field;
    mag_field.x = 0;  // Set your magnetic field values here if available
    mag_field.y = 0;
    mag_field.z = 0;

    // Assign to the nour_msgs/Imu message
    imu_data.linear_acceleration = linear_acc;
    imu_data.angular_velocity = angular_vel;
    imu_data.magnetic_field = mag_field;

    imu_data.header.frame_id = "imu_link";
    current_time = ros::Time::now();

    imu_data.header.stamp = current_time;
    imu_pub.publish(imu_data);
    last_time = current_time;

    loop_rate.sleep();
  }

  return 0;
}

  return 0;
}
